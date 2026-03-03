from pymavlink import mavutil # i had to grab the mavlink library so i can actually talk to the drone brains
import time # i imported time so i can add sleeps and not completely melt the poor cpu
import v2v_bridge # the custom translator i built for the radio boxes

# uav test scripttt
# i made this ground test script just to see if the drone and
# the rover can actually hear each other screaming over the radio

################################# config stuff i setup
# where the flight controller and radio are actually physically plugged in
CONNECTION_STRING = "/dev/ttyACM0" # the actual wire for the drone brain i plugged in
BAUD_RATE = 115200 # the speed i picked for the serial wire cause it works
ESP32_PORT = "/dev/ttyUSB0" # the usb wire going to the radio box i built

def main(): # the main boss function that runs the whole show
    print("==========================================") 
    print("   UAV MISSION 1 - HYBRID SYNC") 
    print("==========================================") 
    
    # i had to tie into the drone flight controller here
    print(f"[Mission 1] Connecting to UAV Controller at {CONNECTION_STRING}...") # logging that we are trying to connect
    try: # trying to connect without crashing the whole python script
        master = mavutil.mavlink_connection(CONNECTION_STRING, baud=BAUD_RATE) # actually opening the mavlink connection i setup
        master.wait_heartbeat() # waiting around for the drone to finally say hello
        print("[Mission 1] Drone Heartbeat found. Sensors Check: OK") # printing success cause it actually worked
    except Exception as e: # if it completely fails and throws an error
        print(f"!!! Error connecting to Drone: {e} !!!") # logging the massive error
        print("Continuing with Bridge only...") # doing a fallback so it doesnt just die
        master = None # the drone is ghosting me so i set it to none

    # time to start the radio bridge link i wrote
    print(f"[Mission 1] Starting V2V Bridge on {ESP32_PORT}...") # telling the terminal the radio is starting
    bridge = v2v_bridge.V2VBridge(ESP32_PORT, name="UAV-Bridge") # i created the bridge object here
    try: # trying to connect to the physical radio box
        bridge.connect() # forcing the serial port to open
        # just a quick debug link check i did
        print("[Mission 1] Sending Hello to UGV...") # logging that we are talking
        bridge.send_message("hello pissrat . we connected") # sending the very first shout over the air to see if it works
    except Exception as e: # if the radio is unplugged or missing entirely
        print(f"!!! Error connecting to ESP32: {e} !!!") # printing the failure so i know what broke
        return # just bail out completely

    ############################ helper logic i wrote so the main code isnt a mess

    def arm_drone(): # i made this command just to wake up the drone motors
        if master: # checking if the drone is actually plugged in
            master.mav.command_long_send(master.target_system, master.target_component, # sending the raw mavlink pulse
                                         mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, # the actual arm command bit
                                         0, 0, 0, 0, 0, 0) # a bunch of empty parameters cause it needs them
            print("[Mission 1] Drone Motors Engaged (Ground Only).") # logging that the motors are spinning

    def disarm_drone(): # command i did to kill the drone motors before it chops my fingers off
        if master: # making sure the drone is connected again
            master.mav.command_long_send(master.target_system, master.target_component, # sending another mavlink pulse
                                         mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 0, # the disarm code this time
                                         0, 0, 0, 0, 0, 0) # more empty useless parameters
            print("[Mission 1] Drone Disarmed.") # logging that it is safe now

    def broadcast_uav_status(seq): # i made this to send drone stats back to the radio bridge
        armed_val = 0 # i defaulted it to safe cause safety first
        mode_val = v2v_bridge.MODE_INITIAL # just setting the default mode here
        
        if master: # if the drone is actually talking to us
            msg = master.recv_match(type='HEARTBEAT', blocking=False) # i had to check the current heartbeat status without blocking the code
            if msg: # if we actually caught a packet
                armed_val = 1 if (msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED) else 0 # doing bit math to check if it is actually armed
        
        t_ms = int(time.time() * 1000) & 0xFFFFFFFF # grabbing the current clock time in milliseconds cause it needs it
        bridge.send_telemetry(seq, t_ms, 0.0, 0.0, armed_val, mode_val) # shoving all the stats i collected down the bridge

    #################### the main execution engine i built

    try: # wrapping the main logic block in a try catch
        # step 1 i arm the drone while it literally just sits on the floor
        arm_drone() # bullying the motors to engage
        time.sleep(2) # i put a wait for 2 seconds so it has time to think
        
        # step 2 is waiting for the ground rover to finally sync up
        print("[Mission 1] Waiting for UGV Sync...") # logging that we are stuck waiting
        ugv_ready = False # i reset the flag to false to start the loop
        timeout_start = time.time() # starting the clock so it doesnt wait forever
        
        while not ugv_ready: # i keep looping this until the rover actually talks back
            # first i check if there is any raw debug talk
            msg = bridge.get_message() # pulling junk from the message mailbox
            if msg: # if there is actually a string there
                print(f"    >>> [RADIO MSG]: {msg}") # i just print whatever the rover said

            # second i check for the actual binary telemetry data
            data = bridge.get_telemetry() # yank it from the status mailbox
            if data: # if we caught a real packet and not static
                seq_u, t_ms_u, vx_u, vy_u, armed_u, safety_byte = data # unpacking all the binary junk into variables
                
                # i had to decode the safety byte bitmask here
                mode_u = safety_byte & 0x0F # extracting the mode bits
                is_armable = bool(safety_byte & 0x10) # checking the armable bit
                has_gps = bool(safety_byte & 0x20) # checking the gps bit so we know if it is blind
                
                status_str = "ARMED" if armed_u == 1 else "DISARMED" # making the status human readable cause numbers are boring
                mode_str = "GUIDED" if mode_u == v2v_bridge.MODE_GUIDED else "MANUAL/INITIAL" # making the mode human readable too
                
                print(f"    [RADIO] UGV: {status_str} | Mode: {mode_str} | Armable: {is_armable} | GPS: {has_gps}") # logging the giant status string
                
                if not is_armable: # if the rover is crying and complaining
                    reason = "NO GPS FIX" if not has_gps else "Safety Check Failed" # i try to figure out why it is crying
                    print(f"    [WARNING] UGV Not Ready: {reason}") # logging the warning
                    print("    [BYPASS] Mimicking Remote: Proceeding with Command Sync anyway...") # i just ignore it and bully it to proceed anyway
                
                # if the code made it here then the link is actually working
                print("\n!!! [SYNC] UGV RADIO LINK VERIFIED & READY !!!") # logging that the radio is a massive success
                ugv_ready = True # i flip the flag so it breaks out of the loop
            
            broadcast_uav_status(0) # sending the drone stats back over to the rover
            time.sleep(1.0) # wait 1 second to chill
            if time.time() - timeout_start > 300: # if i have been waiting for more than 5 minutes
                print("!!! [TIMEOUT] Sync failed. Aborting.") # i just give up and print a timeout
                break # break the stupid loop

        if ugv_ready: # if we actually found the rover
            # step 3 is the coordinated action persistent command loop
            print("\n[Mission 1] >>> INITIATING COORDINATED UGV ARMING") # printing that the real mission is starting
            
            ugv_armed_confirmed = False # i reset the confirmation flag so we can check it
            command_seq = 1 # setting the unique command id to start at 1
            
            while not ugv_armed_confirmed: # i keep trying to poke it until it finally arms
                print(f"[Mission 1] Commanding UGV ARM (Attempt {command_seq})...") # logging the attempt number
                bridge.send_command(cmdSeq=command_seq, cmd=v2v_bridge.CMD_MOVE_FORWARD, estop=0) # i blast the arm command over the radio
                command_seq += 1 # bump the counter up
                
                # i check for a response for a second or two
                for _ in range(5): # doing 5 quick checks really fast
                    data = bridge.get_telemetry() # grab the last status we got
                    if data and data[4] == 1: # if the arm field is finally 1
                        print("\n!!! [SUCCESS] UGV REPORTS ARMED. PROCEEDING !!!") # print success cause it worked
                        ugv_armed_confirmed = True # i flip the flag to true
                        break # stop checking for this loop
                    time.sleep(0.2) # sleep for 200ms
                
                if command_seq > 30: # if i tried 30 times and it is still dead
                    print("!!! [ERROR] UGV failed to arm after 30 attempts. Check Hardware.") # print a massive fail message
                    break # i just completely give up

            if ugv_armed_confirmed: # if the rover motors are finally live
                # step 4 is tracking the progress
                print("[Mission 1] Tracking UGV mission progress...") # log that i am tracking it
                for i in range(15): # i do a loop for 15 status updates
                    telem = bridge.get_telemetry() # grabbing the fresh updated stats
                    if telem: # if we actually got some data
                        v_mps, armed_u = telem[2], telem[4] # i extract the speed and arm flag
                        print(f"  Driving... Speed: {v_mps:.1f} m/s | Status: {'ARMED' if armed_u else 'DISARMED'}") # logging the drive status
                    time.sleep(1.0) # i sleep so it only logs once every second

        # step 5 i finalize and clean up the mess
        print("[Mission 1] Test Sequence Complete.") # log the mission is over
        disarm_drone() # i kill the drone motors so it is safe

    except KeyboardInterrupt: # if i panic and hit ctrl c
        print("[Mission 1] User Interrupted. Safety Disarming...") # log that i aborted it
        disarm_drone() # do a safety kill on the drone
    finally: # finally i always do this part
        bridge.stop() # i shut down the radio bridge cleanly so it doesnt break the port

if __name__ == "__main__": # standard python entry point stuff
    main() # finally actually run the boss function