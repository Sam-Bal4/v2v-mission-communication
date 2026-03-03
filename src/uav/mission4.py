from pymavlink import mavutil # grabs mavlink to talk to drone
import time # for clocking things
import v2v_bridge # our custom radio talk translator

# UAV ugv circle x2 test script
# This is the "Brain" script. It tells the ground vehicle
# to make a full circle twice.

################################# config stuff
# where the fly controller and radio are plugged in
CONNECTION_STRING = "/dev/ttyACM0"   # UAV Flight Controller wire
BAUD_RATE = 115200                   # serial speed
ESP32_PORT = "/dev/ttyUSB0"          # The radio bridge usb wire

def main(): # the main circle boss function
    print("==========================================") # header
    print("   UAV MISSION 4 - CIRCLE MANEUVERS") # title
    print("==========================================") # footer
    
    # tie into the drones flight controller
    print(f"[Mission 4] Connecting to UAV Controller at {CONNECTION_STRING}...") # log connect
    try: # try to connect
        master = mavutil.mavlink_connection(CONNECTION_STRING, baud=BAUD_RATE) # open mavlink
        master.wait_heartbeat() # wait for buzz
        print("[Mission 4] Drone Heartbeat found. Sensors Check: OK") # success
    except Exception as e: # if fail
        print(f"!!! Error connecting to Drone: {e} !!!") # log error
        master = None # drone is ghosting

    # start the bridge link (v2v_bridge.py)
    print(f"[Mission 4] Starting V2V Bridge on {ESP32_PORT}...") # log radio start
    bridge = v2v_bridge.V2VBridge(ESP32_PORT, name="UAV-Bridge") # bridge object
    try: # try to open radio
        bridge.connect() # open serial wires
        bridge.send_message("MISSION 4 STARTING: DOUBLE CIRCLE") # yell over air
    except Exception as e: # if fail
        print(f"!!! Error connecting to ESP32: {e} !!!") # log fail
        return # bail out

    def broadcast_uav_status(): # tells drone stats to radio
        # shoves drone arm status into the radio telemetry link
        armed_val = 0 # default off
        if master: # if drone live
            msg = master.recv_match(type='HEARTBEAT', blocking=False) # check status
            if msg: # if we got one
                armed_val = 1 if (msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED) else 0 # check arm bit
        t_ms = int(time.time() * 1000) & 0xFFFFFFFF # current clock
        bridge.send_telemetry(0, t_ms, 0.0, 0.0, armed_val, 0) # bridge pulse

    ############################ Mission Logic

    try: # try sequence
        # 1. wait for the wheel ugv to stop being a ghost and sync up
        print("[Mission 4] Waiting for UGV Sync...") # log wait
        ugv_ready = False # reset flag
        while not ugv_ready: # loop until talk
            data = bridge.get_telemetry() # pull mailbox
            if data: # if real packet
                seq_u, t_ms_u, v_u, _, armed_u, safety_byte = data # unpack
                is_armable = bool(safety_byte & 0x10) # check armable
                print(f"    [RADIO] UGV Sync: ARMED={armed_u}, Armable={is_armable}") # log sync
                ugv_ready = True # done
            broadcast_uav_status() # heartbeat
            time.sleep(1.0) # wait

        # 2. THE CIRCLE MISSION
        print("\n[Mission 4] >>> INITIATING DOUBLE CIRCLE (2x360)") # start chores
        
        # Shout the CIRCLE command at the bridge
        bridge.send_command(cmdSeq=400, cmd=v2v_bridge.CMD_CIRCLE, estop=0) # blast command
        
        start_mission = time.time() # start clock
        # duration for 2 laps
        total_duration = 18.0 # set timeout
        
        while (time.time() - start_mission) < total_duration: # loop for duration
            # grab speed status coming back from the radio while it spins
            data = bridge.get_telemetry() # check status
            if data: # if we got it
                v_mps = data[2] # check speed
                elapsed = time.time() - start_mission # calc time
                print(f"  Driving Circle... T+{elapsed:.1f}s | Speed: {v_mps:.2f} m/s", end='\r') # log speed
            time.sleep(0.5) # 2hz

        print("\n[Mission 4] CIRCLE SEQUENCE COMPLETE.") # total success

    except KeyboardInterrupt: # someone hit ctrl+c
        print("[Mission 4] Interrupted.") # log abort
    finally: # clean up
        bridge.stop() # close bridge

if __name__ == "__main__": # entry point
    main() # run it
