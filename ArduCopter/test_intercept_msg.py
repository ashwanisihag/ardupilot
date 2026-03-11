import os
import sys

# 1) Point to your ArduPilot mavlink tree on Windows
os.environ["MAVLINK_DIALECT"] = "ardupilotmega"
sys.path.insert(0, r"E:\Github\Ardupilot\modules\mavlink")

from pymavlink import mavutil

print("[DEBUG] mavutil loaded from:", mavutil.__file__)

master = mavutil.mavlink_connection("udpin:0.0.0.0:14555")
print("[TEST] Waiting for heartbeat...")
master.wait_heartbeat()
print(f"[TEST] Heartbeat from system {master.target_system}, component {master.target_component}")

print("[DEBUG] dir(master.mav) contains intercept_target_send?",
      any("intercept_target" in name for name in dir(master.mav)))

print("[TEST] Sending INTERCEPT_TARGET...")
master.mav.intercept_target_send(0.0, 0.0, 100)
print("[TEST] Sent OK")
