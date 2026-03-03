from pymavlink import mavutil
import time
import v2v_bridge

# ------------------- SET THESE PORTS -------------------
# Restore to TTY for Jetson/Linux
CONNECTION_STRING = "/dev/ttyACM0" 
BAUD_RATE = 115200
ESP32_PORT = "/dev/ttyACM1"

# ------------------- Connect -------------------
def main():
    print(f"[Mission 1] Connecting to UAV Controller at {CONNECTION_STRING}...")
    try:
        master = mavutil.mavlink_connection(CONNECTION_STRING, baud=BAUD_RATE)
        master.wait_heartbeat()
        print("[Mission 1] Drone Heartbeat found.")
    except Exception as e:
        print(f"!!! Error connecting to Drone: {e} !!!")
        print("Continuing with Bridge only...")
        master = None

    # Initialize V2V Bridge
    print(f"[Mission 1] Starting V2V Bridge on {ESP32_PORT}...")
    bridge = v2v_bridge.V2VBridge(ESP32_PORT, name="UAV-Bridge")
    try:
        bridge.connect()
    except Exception as e:
        print(f"!!! Error connecting to ESP32: {e} !!!")
        return

    # ------------------- Helpers -------------------
    def arm_drone():
        if master:
            master.mav.command_long_send(master.target_system, master.target_component,
                                         mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 
                                         0, 0, 0, 0, 0, 0)
            print("[Mission 1] Arming props (GROUND ONLY)...")

    def disarm_drone():
        if master:
            master.mav.command_long_send(master.target_system, master.target_component,
                                         mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 0, 
                                         0, 0, 0, 0, 0, 0)
            print("[Mission 1] Disarming.")

    # ------------------- EXECUTION 0-------------------
    try:
        # 1. Arm Drone on Ground
        arm_drone()
        time.sleep(2)
        
        # 2. COORDINATED ACTION: Tell UGV to move!
        print("[Mission 1] Commanding UGV to MOVE 10ft...")
        bridge.send_command(cmdSeq=1, cmd=v2v_bridge.CMD_MOVE_FORWARD, estop=0) 

        # 3. Simulation of "Moving" time (staying armed for 5 seconds)
        print("[Mission 1] Drone is staying armed while UGV moves...")
        for i in range(50): # 5 seconds
            t_ms = int(time.time() * 1000) & 0xFFFFFFFF
            bridge.send_telemetry(i, t_ms, 0.0, 0.0, 0, 0)
            time.sleep(0.1)

        # 4. Clean up
        print("[Mission 1] Test Complete.")
        disarm_drone()

    except KeyboardInterrupt:
        print("Stopping Mission 1...")
        disarm_drone()
    finally:
        bridge.stop()

if __name__ == "__main__":
    main()
