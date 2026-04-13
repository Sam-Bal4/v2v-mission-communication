import time
from pymavlink import mavutil

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
    print("--- Test 6: Pacing LEFT ---")
    master = mavutil.mavlink_connection(DRONE_PORT, baud=BAUD_RATE)
    master.wait_heartbeat()
    
    mapping = master.mode_mapping()
    master.mav.set_mode_send(master.target_system, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, mapping["GUIDED"])
    time.sleep(1)

    master.mav.command_long_send(master.target_system, master.target_component, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0,0,0,0,0,0)
    time.sleep(2)
    master.mav.command_long_send(master.target_system, master.target_component, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, TARGET_ALT)
    time.sleep(5)

    print("Pacing UGV LEFT (-Y) for 10 seconds")
    start_t = time.time()
    while time.time() - start_t < 10:
        set_velocity_body(master, 0.0, -0.5, 0.0) # Sway left
        time.sleep(0.1)

    print("Landing...")
    master.mav.set_mode_send(master.target_system, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, mapping["LAND"])

if __name__ == "__main__":
    main()
