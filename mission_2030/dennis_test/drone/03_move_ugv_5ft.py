import time
from pymavlink import mavutil
import sys
import os

# Allow importing the bridge from the main directory
sys.path.append(os.path.abspath("../../"))
from mission_2030.radio.v2v_bridge import V2VBridge

DRONE_PORT = "/dev/ttyACM0"
ESP32_PORT = "/dev/ttyUSB0"
BAUD_RATE = 57600

def main():
    print("--- Test 3: Drone Send Command to UGV ---")
    bridge = V2VBridge(ESP32_PORT)
    bridge.connect()

    from mission_2030.common.mavlink_utils import arm_vehicle, wait_disarm, get_lidar_alt

    master = mavutil.mavlink_connection(DRONE_PORT, baud=BAUD_RATE)
    master.wait_heartbeat()
    
    mapping = master.mode_mapping()
    master.mav.set_mode_send(master.target_system, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, mapping["GUIDED"])
    time.sleep(1)

    print("Arming...")
    if not arm_vehicle(master):
        return

    print("Takeoff -> 1.3m")
    master.mav.command_long_send(master.target_system, master.target_component, 
                                 mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, 1.3)
    
    # Wait for altitude
    t0 = time.time()
    while time.time() - t0 < 15:
        alt = get_lidar_alt(master)
        if alt and alt >= 1.3 * 0.9:
            print("Altitude reached ✓")
            break
        time.sleep(0.2)

    print("Commanding UGV to move (Sending phase=5)...")
    bridge.send_uav_heartbeat(0, int(time.time()*1000), 5, False)
    
    print("Waiting 10s for UGV to accomplish movement...")
    time.sleep(10)

    print("Switching to LAND...")
    master.mav.set_mode_send(master.target_system, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, mapping["LAND"])

    if wait_disarm(master, 90):
        print("Test 03 complete ✓")
    else:
        print("Warning: landing timeout reached.")

    bridge.stop()

if __name__ == "__main__":
    main()
