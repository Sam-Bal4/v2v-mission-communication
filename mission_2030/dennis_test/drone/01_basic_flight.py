import time
from pymavlink import mavutil

DRONE_PORT = "/dev/ttyACM0"
BAUD_RATE = 57600
TARGET_ALT = 1.3

def get_lidar_alt(master):
    msg = master.recv_match(type='DISTANCE_SENSOR', blocking=False)
    if msg: return msg.current_distance / 100.0
    return 0.0

def main():
    print("--- Test 1: Basic Flight ---")
    master = mavutil.mavlink_connection(DRONE_PORT, baud=BAUD_RATE)
    master.wait_heartbeat()
    print("Heartbeat OK!")

    # Switch to GUIDED
    mapping = master.mode_mapping()
    master.mav.set_mode_send(master.target_system, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, mapping["GUIDED"])
    time.sleep(1)

    print("Arming...")
    master.mav.command_long_send(master.target_system, master.target_component, 
                                 mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0,0,0,0,0,0)
    time.sleep(2)

    print(f"Taking off to {TARGET_ALT}m")
    master.mav.command_long_send(master.target_system, master.target_component,
                                 mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, TARGET_ALT)
    
    start_t = time.time()
    while time.time() - start_t < 15:
        if get_lidar_alt(master) >= TARGET_ALT * 0.9: break
        time.sleep(0.1)

    print("Hovering for 5 seconds...")
    time.sleep(5)

    print("Landing...")
    master.mav.set_mode_send(master.target_system, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, mapping["LAND"])

    landed = False
    while not landed:
        msg = master.recv_match(type='HEARTBEAT', blocking=False)
        if msg and msg.get_srcSystem() == master.target_system:
            if not (msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
                print("Landed and Disarmed successfully.")
                landed = True
                break
        time.sleep(0.1)

if __name__ == "__main__":
    main()
