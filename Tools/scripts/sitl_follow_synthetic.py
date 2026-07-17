#!/usr/bin/env python3
import math
import time

import cv2
import numpy as np
from pymavlink import mavutil

# ---------------------------------------------------------
# USER PARAMETERS
# ---------------------------------------------------------

# SITL / MAVLink connection
MAVLINK_CONNECTION = "udp:127.0.0.1:14550"

# Synthetic frame size
FRAME_WIDTH = 640
FRAME_HEIGHT = 480

# Camera / object geometry (for vertical distance estimate)
FOV_Y_DEG = 50.0
H_OBJECT_M = 0.4
Z_DESIRED = 2.0

# Control gains
Kx = 0.002
Ky = 0.002
Kz = 0.5

MAX_VXY = 0.5
MAX_VZ = 0.3
MAX_LOST_FRAMES = 10

# Smoothing (EMA)
ALPHA_CENTER = 0.3
ALPHA_Z = 0.3

FOV_Y_RAD = math.radians(FOV_Y_DEG)
FY_PIX = FRAME_HEIGHT / (2.0 * math.tan(FOV_Y_RAD / 2.0))


# ---------------------------------------------------------
# MAVLINK / ARDUPILOT
# ---------------------------------------------------------

def connect_mavlink():
    print(f"[INFO] Connecting to ArduPilot at {MAVLINK_CONNECTION} ...")
    master = mavutil.mavlink_connection(MAVLINK_CONNECTION)
    master.wait_heartbeat()
    print(f"[INFO] Heartbeat from system {master.target_system}, "
          f"component {master.target_component}")
    return master


def set_mode_guided(master):
    mode = 'GUIDED'
    mode_map = master.mode_mapping()
    mode_id = mode_map[mode]

    master.mav.set_mode_send(
        master.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode_id
    )

    print("[INFO] Changing mode to GUIDED...")
    while True:
        hb = master.recv_match(type='HEARTBEAT', blocking=True, timeout=5)
        if hb and hb.custom_mode == mode_id:
            print("[INFO] Mode is now GUIDED")
            break


def send_body_velocity(master, vx, vy, vz):
    # time_boot_ms must be 0..4294967295. Using 0 is fine.
    time_boot_ms = 0

    type_mask = 0b0000111111000111  # velocity only

    master.mav.set_position_target_local_ned_send(
        time_boot_ms,
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_FRAME_BODY_NED,
        type_mask,
        0, 0, 0,
        vx, vy, vz,
        0, 0, 0,
        0, 0
    )


# ---------------------------------------------------------
# SYNTHETIC IMAGE GENERATION
# ---------------------------------------------------------

def synthetic_frame(t):
    h, w = FRAME_HEIGHT, FRAME_WIDTH
    frame = np.full((h, w, 3), 255, dtype=np.uint8)

    cx = int(w / 2 + 120 * math.sin(t / 20.0))
    cy = int(h / 3 + 60 * math.cos(t / 30.0))

    cv2.circle(frame, (cx, cy), 25, (0, 0, 0), -1)

    return frame


def detect_object_sky(frame):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    gray = cv2.GaussianBlur(gray, (5, 5), 0)

    edges = cv2.Canny(gray, 80, 160)
    edges = cv2.dilate(edges, None, iterations=1)

    contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL,
                                   cv2.CHAIN_APPROX_SIMPLE)

    if not contours:
        return None

    largest = max(contours, key=cv2.contourArea)
    if cv2.contourArea(largest) < 80:
        return None

    return cv2.boundingRect(largest)


def bbox_center(bbox):
    x, y, w, h = bbox
    return x + w / 2.0, y + h / 2.0


def estimate_distance_from_bbox_height(h_pixels):
    if h_pixels <= 0:
        return None
    return (FY_PIX * H_OBJECT_M) / float(h_pixels)


# ---------------------------------------------------------
# SMOOTHING
# ---------------------------------------------------------

def ema_update(prev, new, alpha):
    if prev is None:
        return new
    return alpha * new + (1.0 - alpha) * prev


# ---------------------------------------------------------
# MAIN
# ---------------------------------------------------------

def main():
    master = connect_mavlink()
    set_mode_guided(master)

    cx_smooth = None
    cy_smooth = None
    Z_smooth = None
    lost_frames = 0
    t = 0

    cx0 = FRAME_WIDTH / 2.0
    cy0 = FRAME_HEIGHT / 2.0

    print("[INFO] Starting synthetic tracking loop...")

    try:
        while True:
            # --- Get synthetic frame ---
            frame = synthetic_frame(t)
            t += 1

            # --- Detect object ---
            bbox = detect_object_sky(frame)

            if bbox is None:
                lost_frames += 1
                if lost_frames > MAX_LOST_FRAMES:
                    send_body_velocity(master, 0, 0, 0)
                    cx_smooth = cy_smooth = Z_smooth = None
                cv2.imshow("Tracking", frame)
                if cv2.waitKey(1) == 27:
                    break
                time.sleep(0.05)
                continue

            lost_frames = 0
            x, y, w, h = bbox
            cx_raw, cy_raw = bbox_center(bbox)

            # --- Smooth center ---
            cx_smooth = ema_update(cx_smooth, cx_raw, ALPHA_CENTER)
            cy_smooth = ema_update(cy_smooth, cy_raw, ALPHA_CENTER)

            cx = cx_smooth
            cy = cy_smooth

            # Pixel errors
            ex_pixels = cx - cx0
            ey_pixels = cy - cy0

            # --- Horizontal control ---
            vy_body = Kx * ex_pixels
            vx_body = Ky * ey_pixels

            vx_body = max(-MAX_VXY, min(MAX_VXY, vx_body))
            vy_body = max(-MAX_VXY, min(MAX_VXY, vy_body))

            # --- Vertical control ---
            Z_est_raw = estimate_distance_from_bbox_height(h)
            if Z_est_raw is not None:
                Z_smooth = ema_update(Z_smooth, Z_est_raw, ALPHA_Z)
                eZ = Z_smooth - Z_DESIRED
                vz_body = -Kz * eZ     # NED frame: up = negative
                vz_body = max(-MAX_VZ, min(MAX_VZ, vz_body))
            else:
                vz_body = 0

            # --- Send MAVLink velocity ---
            send_body_velocity(master, vx_body, vy_body, vz_body)

            # --- Debug print ---
            z_text = f"{Z_smooth:.2f}" if Z_smooth is not None else "None"
            print(f"ex={ex_pixels:.1f}, ey={ey_pixels:.1f}, "
                  f"Z={z_text}, vx={vx_body:.2f}, vy={vy_body:.2f}, vz={vz_body:.2f}")

            # --- Visualization Window ---
            # Bounding box
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)

            # Raw center (red)
            cv2.circle(frame, (int(cx_raw), int(cy_raw)), 4, (0, 0, 255), -1)

            # Smoothed center (blue)
            cv2.circle(frame, (int(cx), int(cy)), 4, (255, 0, 0), -1)

            # Image center (yellow)
            cv2.circle(frame, (int(cx0), int(cy0)), 4, (0, 255, 255), -1)

            cv2.imshow("Tracking", frame)
            if cv2.waitKey(1) == 27:  # ESC to quit
                break

            time.sleep(0.05)

    except KeyboardInterrupt:
        print("\n[INFO] Stopping (Ctrl+C).")
    finally:
        send_body_velocity(master, 0, 0, 0)
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
