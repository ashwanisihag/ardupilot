#!/usr/bin/env python3
import os
import sys
import time
import argparse

# --- Use repo pymavlink with updated ardupilotmega dialect ---
os.environ["MAVLINK_DIALECT"] = "ardupilotmega"
sys.path.insert(0, os.path.expanduser(
    "~/ardupilot/modules/mavlink"
))

import cv2
import numpy as np
from pymavlink import mavutil

# -------------------- CLI --------------------
def parse_args():
    ap = argparse.ArgumentParser(
        description="Mouse-driven target generator for INTERCEPT mode (SITL)"
    )
    ap.add_argument(
        "--conn",
        default="udp:127.0.0.1:14550",
        help="MAVLink connection string (default: udp:127.0.0.1:14550)",
    )
    return ap.parse_args()

ARGS = parse_args()
MAVLINK_CSTR = ARGS.conn

FRAME_WIDTH = 640
FRAME_HEIGHT = 480

mouse_x = FRAME_WIDTH // 2
mouse_y = FRAME_HEIGHT // 2


def on_mouse(event, x, y, flags, param):
    global mouse_x, mouse_y
    if event == cv2.EVENT_MOUSEMOVE:
        mouse_x, mouse_y = x, y


def connect_mavlink():
    print(f"[MAV] Connecting to {MAVLINK_CSTR}")
    master = mavutil.mavlink_connection(MAVLINK_CSTR)
    master.wait_heartbeat()
    print(f"[MAV] Heartbeat from system {master.target_system}, component {master.target_component}")
    return master


def send_intercept_target(master, x_norm: float, y_norm: float, quality: int = 1):
    """Send INTERCEPT_TARGET message: x_norm (lateral), y_norm (vertical)."""
    x_norm = max(-1.0, min(1.0, float(x_norm)))
    y_norm = max(-1.0, min(1.0, float(y_norm)))
    master.mav.intercept_target_send(x_norm, y_norm, quality)


def main():
    master = connect_mavlink()

    win_name = "INTERCEPT mouse"
    cv2.namedWindow(win_name)
    cv2.setMouseCallback(win_name, on_mouse)

    cx0 = FRAME_WIDTH // 2
    cy0 = FRAME_HEIGHT // 2

    last_send_t = 0.0
    SEND_DT = 1.0 / 20.0  # 20 Hz

    while True:
        frame = np.zeros((FRAME_HEIGHT, FRAME_WIDTH, 3), dtype=np.uint8)

        # Draw center crosshair
        cv2.line(frame, (cx0 - 10, cy0), (cx0 + 10, cy0), (0, 0, 255), 1)
        cv2.line(frame, (cx0, cy0 - 10), (cx0, cy0 + 10), (0, 0, 255), 1)

        # Draw mouse dot
        cv2.circle(frame, (mouse_x, mouse_y), 6, (0, 255, 0), -1)

        # --- LATERAL x_norm ---
        # x_norm = -1 at left edge, +1 at right edge, 0 at center
        x_norm = (mouse_x - cx0) / float(cx0)
        x_norm = max(-1.0, min(1.0, x_norm))

        # --- VERTICAL y_norm ---
        # y_norm = +1.0 at TOP, 0.0 at center, -1.0 at BOTTOM
        y_norm = (cy0 - mouse_y) / float(cy0)
        y_norm = max(-1.0, min(1.0, y_norm))

        # Debug print to confirm mouse mapping
        # (watch this in the Python console)
        print(f"MOUSE: x={mouse_x:3d} y={mouse_y:3d}  x_norm={x_norm:+.2f} y_norm={y_norm:+.2f}")

        # Display HUD text
        cv2.putText(frame, f"x_norm={x_norm:+.2f}", (10, 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 1)
        cv2.putText(frame, f"y_norm={y_norm:+.2f}", (10, 55),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 1)
        cv2.putText(frame, "Move mouse: X=lateral, Y=forward/back (ESC to quit)",
                    (10, FRAME_HEIGHT - 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)

        cv2.imshow(win_name, frame)
        key = cv2.waitKey(1) & 0xFF
        if key == 27 or key == ord('q'):
            break

        now = time.time()
        if now - last_send_t >= SEND_DT:
            send_intercept_target(master, x_norm, y_norm, quality=1)
            last_send_t = now

    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
