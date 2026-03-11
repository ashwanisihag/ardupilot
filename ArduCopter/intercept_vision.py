#!/usr/bin/env python3
import os
import sys
import time
import argparse

import cv2
import numpy as np

# ---------------------------------------------------------------------
# MAVLink import (make sure this path points to your ArduPilot checkout)
# ---------------------------------------------------------------------

os.environ["MAVLINK_DIALECT"] = "ardupilotmega"
# TODO: change this path if your ArduPilot repo is elsewhere:
sys.path.insert(0, r"E:\Github\Ardupilot\modules\mavlink")

from pymavlink import mavutil


# -------------------- CLI --------------------

def parse_args():
    ap = argparse.ArgumentParser(
        description="Vision-based target generator for INTERCEPT mode using pi_follow_camera-style detection"
    )
    ap.add_argument(
        "--conn",
        default="udpin:0.0.0.0:14555",
        help="MAVLink connection string (default: udpin:0.0.0.0:14555)",
    )
    ap.add_argument(
        "--cam",
        type=int,
        default=0,
        help="Camera index for cv2.VideoCapture (default: 0)",
    )
    ap.add_argument(
        "--width",
        type=int,
        default=640,
        help="Processing frame width (default: 640)",
    )
    ap.add_argument(
        "--height",
        type=int,
        default=480,
        help="Processing frame height (default: 480)",
    )
    ap.add_argument(
        "--send-hz",
        type=float,
        default=20.0,
        help="INTERCEPT_TARGET send rate in Hz (default: 20.0)",
    )
    return ap.parse_args()


ARGS = parse_args()

MAVLINK_CSTR = ARGS.conn
FRAME_WIDTH = ARGS.width
FRAME_HEIGHT = ARGS.height


# ---------------------------------------------------------
# pi_follow_camera.py–style object detection
#   - Sky / non-sky segmentation
#   - Dark-object filter
#   - Central-ROI filter
# ---------------------------------------------------------

def detect_object_sky(frame):
    """
    Sky / non-sky segmentation based object detector.

    Returns (bbox, frame_resized) where:
      - bbox is (x, y, w, h) or None if no valid object.
      - frame_resized is the frame resized to (FRAME_WIDTH, FRAME_HEIGHT).
    """
    frame_resized = cv2.resize(frame, (FRAME_WIDTH, FRAME_HEIGHT))

    hsv = cv2.cvtColor(frame_resized, cv2.COLOR_BGR2HSV)
    _, s, v = cv2.split(hsv)

    # Use top 1/3 of the image as "sky" reference
    sky_rows = FRAME_HEIGHT // 3
    sky_s = s[:sky_rows, :]
    sky_v = v[:sky_rows, :]

    s_mean = float(np.mean(sky_s))
    v_mean = float(np.mean(sky_v))

    # Pad thresholds a bit
    S_PAD = 20.0
    V_PAD = 30.0

    # Sky mask: low saturation & reasonably bright
    sky_mask = np.zeros_like(s, dtype=np.uint8)
    sky_pixels = (s.astype(np.float32) < (s_mean + S_PAD)) & (
        v.astype(np.float32) > (v_mean - V_PAD)
    )
    sky_mask[sky_pixels] = 255

    # Non-sky mask = candidate object region
    obj_mask = cv2.bitwise_not(sky_mask)

    # Clean up with morphological open/close
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    obj_mask = cv2.morphologyEx(obj_mask, cv2.MORPH_OPEN, kernel, iterations=1)
    obj_mask = cv2.morphologyEx(obj_mask, cv2.MORPH_CLOSE, kernel, iterations=1)

    # Restrict to dark pixels only (e.g. a dark object against the sky)
    DARK_V = 80
    dark_pixels = (v < DARK_V)
    dark_mask = np.zeros_like(obj_mask)
    dark_mask[dark_pixels] = 255

    obj_mask = cv2.bitwise_and(obj_mask, dark_mask)

    # If almost everything is sky (few non-sky pixels), bail out
    non_sky_pixels = np.count_nonzero(obj_mask)
    total_pixels = obj_mask.size
    frac_non_sky = non_sky_pixels / float(total_pixels)

    if frac_non_sky < 0.02:
        return None, frame_resized

    # Find contours in the object mask
    contours, _ = cv2.findContours(
        obj_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
    )
    if not contours:
        return None, frame_resized

    img_area = FRAME_WIDTH * FRAME_HEIGHT
    min_area = 0.03 * img_area   # ignore tiny blobs

    # Prefer objects near the center of the image
    center_x_min = FRAME_WIDTH * 0.15
    center_x_max = FRAME_WIDTH * 0.85
    center_y_min = FRAME_HEIGHT * 0.2

    largest = None
    largest_area = 0.0

    for c in contours:
        area = cv2.contourArea(c)
        if area < min_area:
            continue

        x, y, w, h = cv2.boundingRect(c)
        cx = x + w / 2.0
        cy = y + h / 2.0

        # Discard blobs too high (in the sky region) or too far sideways
        if cy < center_y_min:
            continue
        if cx < center_x_min or cx > center_x_max:
            continue

        if area > largest_area:
            largest_area = area
            largest = (x, y, w, h)

    if largest is None:
        return None, frame_resized

    return largest, frame_resized


# ---------------------------------------------------------
# MAVLink helpers
# ---------------------------------------------------------

def connect_mavlink():
    print(f"[MAV] Connecting to {MAVLINK_CSTR}")
    master = mavutil.mavlink_connection(MAVLINK_CSTR)
    master.wait_heartbeat()
    print(f"[MAV] Heartbeat from system {master.target_system}, component {master.target_component}")
    return master


def send_intercept_target(master, x_norm: float, y_norm: float, quality: int = 1):
    """
    Send INTERCEPT_TARGET:
      x_norm: normalized horizontal offset [-1,1], +right
      y_norm: normalized vertical offset [-1,1], +up
      quality: 0=invalid, >0=valid
    """
    x_norm = max(-1.0, min(1.0, float(x_norm)))
    y_norm = max(-1.0, min(1.0, float(y_norm)))
    quality = int(max(0, min(255, quality)))
    master.mav.intercept_target_send(x_norm, y_norm, quality)


# ---------------------------------------------------------
# Main loop (with sticky bbox to reduce flicker)
# ---------------------------------------------------------

def main():
    master = connect_mavlink()

    cap = cv2.VideoCapture(ARGS.cam)
    if not cap.isOpened():
        print("[VISION] ERROR: Could not open camera index", ARGS.cam)
        sys.exit(1)

    win_name = "INTERCEPT vision (pi_follow_camera detector)"
    cv2.namedWindow(win_name)

    send_dt = 1.0 / ARGS.send_hz
    last_send_t = 0.0

    print("[VISION] Press ESC or 'q' to quit.")

    # --- sticky bbox state ---
    last_bbox = None
    last_seen_time = 0.0
    STICKY_SEC = 0.4  # keep last bbox for 0.4 s after losing it

    while True:
        ret, frame = cap.read()
        if not ret or frame is None:
            print("[VISION] WARNING: Failed to grab frame")
            time.sleep(0.05)
            continue

        now_sec = time.time()

        bbox, frame_resized = detect_object_sky(frame)

        if bbox is not None:
            # fresh detection
            last_bbox = bbox
            last_seen_time = now_sec
        else:
            # no new detection: maybe use sticky bbox
            if last_bbox is not None and (now_sec - last_seen_time) < STICKY_SEC:
                bbox = last_bbox  # keep aiming at last known position
            else:
                last_bbox = None  # fully lost

        # Default values: no target
        x_norm = 0.0
        y_norm = 0.0
        quality = 0

        draw_frame = frame_resized.copy()
        h, w = frame_resized.shape[:2]
        cx0 = w / 2.0
        cy0 = h / 2.0

        # Draw center crosshair
        cv2.line(draw_frame, (int(cx0) - 10, int(cy0)),
                 (int(cx0) + 10, int(cy0)), (0, 0, 255), 1)
        cv2.line(draw_frame, (int(cx0), int(cy0) - 10),
                 (int(cx0), int(cy0) + 10), (0, 0, 255), 1)

        if bbox is not None:
            x, y, bw, bh = bbox
            cv2.rectangle(draw_frame, (int(x), int(y)),
                          (int(x + bw), int(y + bh)), (0, 255, 0), 2)

            cx = x + bw / 2.0
            cy = y + bh / 2.0

            # Map pixel → normalized coordinates
            x_norm = (cx - cx0) / float(cx0)      # -1..+1
            x_norm = max(-1.0, min(1.0, x_norm))

            y_norm = (cy0 - cy) / float(cy0)      # +1 top, -1 bottom
            y_norm = max(-1.0, min(1.0, y_norm))

            # Simple quality: based on bbox area
            area = bw * bh
            img_area = w * h
            frac = area / float(img_area)
            quality = int(max(1, min(255, frac * 4000.0)))

            cv2.circle(draw_frame, (int(cx), int(cy)), 6, (255, 255, 0), -1)
        else:
            # fully lost: keep x_norm/y_norm=0, quality=0
            pass

        # HUD text
        cv2.putText(draw_frame, f"x_norm={x_norm:+.2f}", (10, 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 1)
        cv2.putText(draw_frame, f"y_norm={y_norm:+.2f}", (10, 55),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 1)
        cv2.putText(draw_frame, f"quality={quality}", (10, 85),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 1)
        cv2.putText(draw_frame, "Sky/non-sky detector (sticky)",
                    (10, h - 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)

        cv2.imshow(win_name, draw_frame)
        key = cv2.waitKey(1) & 0xFF
        if key == 27 or key == ord('q'):
            break

        now = time.time()
        if now - last_send_t >= send_dt:
            send_intercept_target(master, x_norm, y_norm, quality=quality)
            last_send_t = now

            print(f"VISION: bbox={'None' if bbox is None else bbox} "
                  f"x_norm={x_norm:+.2f} y_norm={y_norm:+.2f} q={quality}")

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
