import time
from pymavlink import mavutil
import sys
import os
sys.path.append(os.path.abspath("../../"))
from mission_2030.common.mavlink_utils import arm_vehicle, get_lidar_alt

DRONE_PORT = "/dev/ttyACM0"
BAUD_RATE = 57600
TARGET_ALT = 1.3

def set_velocity_body(master, vx, vy, vz):
    master.mav.set_position_target_local_ned_send(
        0, master.target_system, master.target_component,
        mavutil.mavlink.MAV_FRAME_BODY_NED, 0b0000111111000111,
        0, 0, 0, vx, vy, vz, 0, 0, 0, 0, 0
    )

def main():
    print("--- Test 5: Pacing Forward ---")
    master = mavutil.mavlink_connection(DRONE_PORT, baud=BAUD_RATE)
    master.wait_heartbeat()
    
    mapping = master.mode_mapping()
    print("Arming...")
    if not arm_vehicle(master):
        return

    print("Takeoff → 1.3m")
    master.mav.command_long_send(master.target_system, master.target_component, 
                                 mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, TARGET_ALT)
    
    # Wait for altitude
    t_takeoff = time.time()
    while time.time() - t_takeoff < 15:
        alt = get_lidar_alt(master)
        if alt and alt >= TARGET_ALT * 0.9:
            print("Altitude reached ✓")
            break
        time.sleep(0.2)

    print("Pacing UGV FORWARD (+X) for 10 seconds")
    start_t = time.time()
    while time.time() - start_t < 10:
        set_velocity_body(master, 0.5, 0.0, 0.0)
        time.sleep(0.1)

    print("Landing...")
    master.mav.set_mode_send(master.target_system, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, mapping["LAND"])

if __name__ == "__main__":
    main()
