from dronekit import connect, VehicleMode
import time
import v2v_bridge
from pymavlink import mavutil 

# ------------------- SET THESE PORTS -------------------
# User manual ports for RPi
UGV_CONTROL_PORT = "/dev/ttyACM0"   # UGV Flight Controller
ESP32_BRIDGE_PORT = "/dev/ttyUSB0"  # UGV ESP32 Bridge

# ------------------- Mission params -------------------
DIST_M = 3.048       # 10 ft
SPEED_MPS = 1.0      # Drive speed
TELEM_SEND_HZ = 5    # Broadcast frequency

# ------------------- UGV Setup (DroneKit) -------------------
print("==========================================")
print("   UGV GROUND STATION - HYBRID ARMING")
print("==========================================")
print(f"[Ground] Connecting to UGV at {UGV_CONTROL_PORT}...")

try:
    vehicle = connect(UGV_CONTROL_PORT, wait_ready=True, baud=115200)
    print(f"[Ground] Connected! Ready to sync.")
except Exception as e:
    print(f"!!! Error connecting to UGV: {e} !!!")
    exit()

def broadcast_status(bridge, seq):
    """Sends current state back to the UAV."""
    armed_val = 1 if vehicle.armed else 0
    
    # Mode mapping
    m = vehicle.mode.name
    mode_idx = v2v_bridge.MODE_INITIAL
    if m == "GUIDED": mode_idx = v2v_bridge.MODE_GUIDED
    elif m == "AUTO": mode_idx = v2v_bridge.MODE_AUTO
    elif m == "LAND": mode_idx = v2v_bridge.MODE_LAND
    
    # Safety Bitfield (estop byte)
    # Bit 0-3: Mode
    # Bit 4: is_armable
    # Bit 5: GPS (1 if fix > 0)
    armable_bit = 0x10 if vehicle.is_armable else 0x00
    gps_bit = 0x20 if (vehicle.gps_0.fix_type > 0) else 0x00
    safety_byte = (mode_idx & 0x0F) | armable_bit | gps_bit
    
    t_ms = int(time.time() * 1000) & 0xFFFFFFFF
    bridge.send_telemetry(seq, t_ms, 0.0, 0.0, armed_val, safety_byte)

def arm_and_move(bridge):
    """Hybrid Arming: Arm in current mode first, then switch to GUIDED."""
    print("\n[Ground] >>> INITIATING REMOTE-STYLE FORCE ARM SEQUENCE")
    
    # 1. Non-blocking Safety Check
    if not vehicle.is_armable:
        print(f"!!! [WARNING] PRE-ARM CHECKS FAILED (GPS: {vehicle.gps_0.fix_type}) !!!")
        print("    [NOTICE] Mimicking Remote Control: Attempting to bypass safety...")
    
    # 2. ARM SEQUENCE (Start in Current Mode)
    print(f"  [ARM] Stage 1: Attempting to ARM in {vehicle.mode.name} mode...")
    
    for attempt in ["FIRST ARM", "RESET DISARM", "FINAL ARM"]:
        state = True if "ARM" in attempt else False
        print(f"  [FORCE-SYNC] Initiating {attempt}...")
        
        for retry in range(3):
            print(f"    - Attempt {retry+1}/3: Setting vehicle.armed to {state}")
            vehicle.armed = state
            
            # Wait for confirmation
            timeout = time.time() + 3
            while vehicle.armed != state:
                if time.time() > timeout: break
                broadcast_status(bridge, 0)
                time.sleep(0.1)
            
            if vehicle.armed == state:
                print(f"    - Success: Vehicle is now {'Armed' if state else 'Disarmed'}")
                break
            else:
                print(f"    - Hardware unresponsive, retrying...")
        
        time.sleep(1.0) 

    if not vehicle.armed:
        print("!!! [CRITICAL] UGV failed to ARM despite bypass. Check Hardware. !!!")
        return

    # 3. Mode Switch (Switch to GUIDED only AFTER arming)
    print(f"  [MODE] Switching {vehicle.mode.name} -> GUIDED for Move...")
    vehicle.mode = VehicleMode("GUIDED")
    m_timeout = time.time() + 5
    while vehicle.mode.name != "GUIDED" and time.time() < m_timeout:
        broadcast_status(bridge, 0)
        time.sleep(0.1)
    
    if vehicle.mode.name != "GUIDED":
        print(f"!!! [WARNING] Mode Switch to GUIDED Failed. Moving in {vehicle.mode.name} anyway. !!!")

    print("************************************")
    print("!!! UGV FULLY ARMED AND SYNCED !!!")
    print("************************************")
    
    # 4. Move forward
    print(f"[Ground] Moving Forward {DIST_M}m...")
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0, 0, 0, mavutil.mavlink.MAV_FRAME_BODY_NED, 0b0000111111000111,
        0, 0, 0, SPEED_MPS, 0, 0, 0, 0, 0, 0, 0)
    
    start_t = time.time()
    duration = DIST_M / SPEED_MPS
    
    while (time.time() - start_t) < duration:
        vehicle.send_mavlink(msg)
        broadcast_status(bridge, 0)
        time.sleep(0.1)
    
    # 5. Stop and Safety Disarm
    print("[Ground] Stop. Safety Disarm.")
    stop_msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0, 0, 0, mavutil.mavlink.MAV_FRAME_BODY_NED, 0b0000111111000111,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0)
    vehicle.send_mavlink(stop_msg)
    vehicle.armed = False
    broadcast_status(bridge, 0)
    print("[Ground] Mission Complete.\n")

# ------------------- MAIN LOOP -------------------
def main():
    print(f"[Ground] Starting Bridge on {ESP32_BRIDGE_PORT}...")
    bridge = v2v_bridge.V2VBridge(ESP32_BRIDGE_PORT, name="Ground-Station")
    try:
        bridge.connect()
        print("[Ground] Sending Hello to Air...")
        bridge.send_message("hi flying rat . we connected")
    except Exception as e:
        print(f"!!! Bridge Serial Error: {e} !!!")
        return

    print("[Ground] READY: WAITING FOR AIR COMMAND...")
    
    seq = 0
    try:
        while True:
            # 1. READ AIR STATUS & MESSAGES
            msg = bridge.get_message()
            if msg: print(f"\n>>> [AIR MESSAGE]: {msg}\n")
            
            air_telem = bridge.get_telemetry()
            if air_telem:
                # air_status_str = "ARMED" if air_telem[4] == 1 else "DISARMED"
                pass

            # 2. BROADCAST HEARTBEAT
            if seq % 10 == 0:
                print(f"[Ground] Heartbeat... Mode: {vehicle.mode.name} | Armed: {vehicle.armed} | GPS: {vehicle.gps_0.fix_type}")
            broadcast_status(bridge, seq)
            seq += 1

            # 3. Listen for mission command
            cmd = bridge.get_command()
            if cmd:
                cmdSeq, cmdVal, eStopFlag = cmd
                print(f"!!! [RADIO] Received CMD type {cmdVal} seq {cmdSeq} !!!")
                
                if cmdVal == v2v_bridge.CMD_MOVE_FORWARD:
                    if vehicle.armed:
                        print("    - Already Armed. Skipping sequence, moving now.")
                    arm_and_move(bridge)
                elif eStopFlag == 1:
                    print("!!! [ABORT] EMERGENCY DISARM !!!")
                    vehicle.armed = False
            
            time.sleep(1.0 / TELEM_SEND_HZ)
            
    except KeyboardInterrupt:
        print("[Ground] Shutdown.")
    finally:
        bridge.stop()
        vehicle.close()

if __name__ == "__main__":
    main()
