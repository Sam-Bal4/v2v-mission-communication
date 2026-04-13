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

    master = mavutil.mavlink_connection(DRONE_PORT, baud=BAUD_RATE)
    master.wait_heartbeat()
    
    mapping = master.mode_mapping()
    master.mav.set_mode_send(master.target_system, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, mapping["GUIDED"])
    time.sleep(1)

    print("Arming & Takeoff...")
    master.mav.command_long_send(master.target_system, master.target_component, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0,0,0,0,0,0)
    time.sleep(2)
    master.mav.command_long_send(master.target_system, master.target_component, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, 1.3)
    time.sleep(5) 

    print("Commanding UGV to move (Sending phase=5)...")
    bridge.send_uav_heartbeat(0, int(time.time()*1000), 5, False)
    
    # Wait for UGV to say it's done (Phase 6 or whatever)
    # Since we are mock isolating, we will just wait 10 seconds.
    print("Waiting 10s for UGV to accomplish movement...")
    time.sleep(10)

    print("Landing...")
    master.mav.set_mode_send(master.target_system, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, mapping["LAND"])

    landed = False
    land_start = time.time()
    while not landed and time.time() - land_start < 90:
        msg = master.recv_match(type='HEARTBEAT', blocking=False)
        if msg and msg.get_srcSystem() == master.target_system:
            if not (msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
                print("Landed and Disarmed successfully.")
                landed = True
                break
        time.sleep(0.1)
    
    if not landed:
        print("Warning: landing timeout reached.")

    bridge.stop()

if __name__ == "__main__":
    main()
