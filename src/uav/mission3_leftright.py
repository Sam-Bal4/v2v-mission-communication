from pymavlink import mavutil # i had to grab mavlink to actually boss the drone around
import time # i pulled in time so i can sleep the loops and not melt the cpu
import v2v_bridge # the custom radio talk translator i built

# uav left right ugv test
# i made this brain script to basically bully the ground rover
# into doing a 5 stage zig zag maneuver just to see if it works

################################# config stuff i setup
# where i physically plugged in the flight controller and radio
CONNECTION_STRING = "/dev/ttyACM0"   # the wire for the drone flight controller
BAUD_RATE = 115200                   # the serial speed i picked cause it works
ESP32_PORT = "/dev/ttyUSB0"          # the usb wire going to the radio bridge

def main(): # the main zig zag boss function that runs everything
    print("==========================================") # pretty header i added
    print("   UAV MISSION 3 - ZIG-ZAG MANEUVERS") # the script title
    print("==========================================") # pretty footer to close it
    
    # i had to tie into the drone flight controller here
    print(f"[Mission 3] Tying into Drone at {CONNECTION_STRING}...") # logging the connection attempt
    try: # trying to connect without crashing the whole script
        master = mavutil.mavlink_connection(CONNECTION_STRING, baud=BAUD_RATE) # opening the mavlink connection
        master.wait_heartbeat() # waiting around for the drone to buzz me back
        print("[Mission 3] Drone Heartbeat found. Sensors Check: OK") # logging the massive success
    except Exception as e: # if it completely fails
        print(f"!!! Error connecting to Drone: {e} !!!") # printing the massive error
        master = None # the drone is completely ghosting me

    # time to start the radio bridge i wrote
    print(f"[Mission 3] Starting Radio Bridge on {ESP32_PORT}...") # logging that the radio is starting
    bridge = v2v_bridge.V2VBridge(ESP32_PORT, name="UAV-Bridge") # creating the bridge object
    try: # trying to force the radio to open
        bridge.connect() # actually opening the serial wires
        bridge.send_message("MISSION 3 STARTING...") # yelling over the air that we are starting
    except Exception as e: # if the radio is missing or dead
        print(f"!!! Error connecting to ESP32: {e} !!!") # logging the failure
        return # just bail out entirely

    def broadcast_uav_status(): # i made this to shove drone stats to the radio
        # it basically tells the ground rover if the drone is armed or just sitting there
        armed_val = 0 # i defaulted it to safe and off
        if master: # if the drone is actually alive
            msg = master.recv_match(type='HEARTBEAT', blocking=False) # checking the heartbeat status real quick
            if msg: # if we actually caught a packet
                armed_val = 1 if (msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED) else 0 # checking the arm bit with some math
        t_ms = int(time.time() * 1000) & 0xFFFFFFFF # grabbing the current clock time
        bridge.send_telemetry(0, t_ms, 0.0, 0.0, armed_val, 0) # shoving the pulse down the bridge

    ############################ the main mission logic i built

    try: # wrapping it in a try catch so it doesnt die
        # step 1 i just wait for the ground rover to yell its status back
        print("[Mission 3] Waiting for UGV Sync...") # logging that i am stuck waiting
        ugv_ready = False # resetting the flag so the loop starts
        while not ugv_ready: # endless loop until it talks
            data = bridge.get_telemetry() # pulling junk from the mailbox
            if data: # if it is a real packet and not static
                seq_u, t_ms_u, v_u, _, armed_u, safety_byte = data # unpacking all the variables
                is_armable = bool(safety_byte & 0x10) # checking if it is armable
                has_gps = bool(safety_byte & 0x20) # checking the gps bit
                print(f"    [RADIO] UGV Sync: ARMED={armed_u}, Armable={is_armable}, GPS={has_gps}") # logging the sync data
                ugv_ready = True # flipping the flag cause we are done
            broadcast_uav_status() # sending my own heartbeat out
            time.sleep(1.0) # wait a second so it doesnt fry

        # step 2 is the actual zig zag sequence
        # this is the literal list of chores the ground rover has to do
        print("\n[Mission 3] >>> INITIATING ZIG-ZAG MANEUVERS") # starting the chores
        
        sequence = [ # the chore list i made
            (v2v_bridge.CMD_MOVE_2FT, "DRIVE FORWARD 2FT"), # chore step 1
            (v2v_bridge.CMD_TURN_RIGHT, "TURN RIGHT 90"), # chore step 2
            (v2v_bridge.CMD_MOVE_2FT, "DRIVE FORWARD 2FT"), # chore step 3
            (v2v_bridge.CMD_TURN_LEFT, "TURN LEFT 90"), # chore step 4
            (v2v_bridge.CMD_MOVE_2FT, "DRIVE FORWARD 2FT (FINAL)") # the final chore step
        ]
        
        for i, (cmd, desc) in enumerate(sequence): # looping through every single chore
            print(f"\n[Mission 3] Task {i+1}/5: {desc}") # logging which task we are on
            # i shout the command at the bridge so it packs it up into binary
            bridge.send_command(cmdSeq=300+i, cmd=cmd, estop=0) # blasting the command over the air
            
            # then i just sit here and wait for the stupid wheels to finish moving
            wait_time = 4.0 if "DRIVE" in desc else 6.0 # setting the timeout based on what the chore is
            start_segment = time.time() # starting the segment clock
            while (time.time() - start_segment) < wait_time: # looping for the duration
                # catching the speed telemetry data from the radio
                data = bridge.get_telemetry() # checking the status box
                if data: # if we got something
                    v_mps = data[2] # checking the exact speed
                    print(f"  UGV Moving... Speed: {v_mps:.2f} m/s", end='\r') # logging the speed on the same line
                time.sleep(0.5) # sleeping so it checks twice a second
            print(f"\n[Mission 3] Task {i+1} Complete.") # declaring task success

        print("\n[Mission 3] FULL ZIG-ZAG SEQUENCE COMPLETE.") # total mission success

    except KeyboardInterrupt: # if someone panics and hits ctrl c
        print("[Mission 3] Interrupted.") # log the abort
    finally: # always run this clean up
        bridge.stop() # safely close the bridge so the port is free

if __name__ == "__main__": # standard python entry point
    main() # run the boss function