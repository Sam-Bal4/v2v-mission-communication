import time

# Import our common bridge!
from v2v_bridge import V2VBridge

def main():
    # Set the port to whatever your Pi/PC uses for the UGV ESP32
    PORT = "/dev/ttyACM0" 
    
    # 1. Start the Bridge
    bridge = V2VBridge(PORT, name="Ground-Station")
    bridge.connect()

    print("[Ground] Listening for Drone Mission...")
    
    try:
        while True:
            # 2. Check for Telemetry
            telem = bridge.get_telemetry()
            if telem:
                seq, t_ms, vx, vy, marker, estop = telem
                print(f"[TELEM] seq={seq} SpeedX={vx:.2f} E-Stop={estop}")

            # 3. Check for Commands (from the Air!)
            cmd = bridge.get_command()
            if cmd:
                cmdSeq, cmdVal, eStop = cmd
                if cmdVal == 5:
                    print("************************************")
                    print("!!! [AIR COMMAND] MOVE 10 FT FRONT !!!")
                    print("************************************")
                    # TRIGGER MOTOR CONTROLS HERE
                elif eStop == 1:
                    print("[ABORT] Emergency Land triggered from ground!")
            
            time.sleep(0.01)
            
    except KeyboardInterrupt:
        print("[Ground] Stopping...")
    finally:
        bridge.stop()

if __name__ == "__main__":
    main()
