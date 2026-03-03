from pymavlink import mavutil # i had to grab mavlink to actually talk to the drone flight controller brain
import time # i imported time for timing and loop sleeps so the cpu doesnt completely melt
import math # i added math for doing boring degree to radian math
import v2v_bridge # the custom translator i built for the radio bridge binary talk

# uav flight and ugv script
# i made this flight test script where the drone takes off
# and bosses the ground rover around to move while it flies

################################# config stuff i setup
# where i physically plugged in the flight controller and radio
CONNECTION_STRING = "/dev/ttyACM0" # the actual wire for the drone controller
BAUD_RATE = 115200 # the serial wire speed i picked

# the uav radio box port i set
ESP32_PORT = "/dev/ttyUSB0"

# mission params i tuned
DIST_M = 3.048       # the 10 ft move target cause imperial units are annoying
SPEED_MPS = 0.5      # slow and steady wins the race so it doesnt crash
TAKEOFF_ALT_M = 2.0  # the target height i want it to hover at
MAX_ALT_ALLOWED = 3.5 # safety fence height around 11 ft so it doesnt hit the ceiling

# time to start the drone brain connection
print(f"[Mission 2] Connecting to {CONNECTION_STRING}...") # logging that i am trying to connect
master = mavutil.mavlink_connection(CONNECTION_STRING, baud=BAUD_RATE) # actually opening the mavlink link i setup
master.wait_heartbeat() # waiting forever for the drone to say hello
print("[Mission 2] Heartbeat found. Optical Flow/Lidar Ready.") # checking if it was a success

# i initialize the radio link translator here
bridge = v2v_bridge.V2VBridge(ESP32_PORT, name="UAV-Bridge") # creating the bridge object
bridge.connect() # opening the serial wire straight to the radio box

############################ the mavlink helpers i wrote

def change_mode(mode: str): # i made this to change the drone flight mode
    mapping = master.mode_mapping() # asking the drone for the valid mode list
    if mode not in mapping: return # abort everything if the mode is totally fake
    mode_id = mapping[mode] # i had to find the secret id for the mode
    master.mav.set_mode_send(master.target_system, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, mode_id) # blasting the mode pulse to the brain
    print(f"Mode set: {mode}") # logging the change so i can see it

def arm_drone(): # i wrote this to engage the scary drone motors
    master.mav.command_long_send(master.target_system, master.target_component, # sending the actual mavlink pulse
                                 mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, # shoving the arm flag in here
                                 0, 0, 0, 0, 0, 0) # a bunch of empty parameters cause the code needs them
    print("Arming...") # logging that it is about to get dangerous

def takeoff(alt): # this tells the drone to actually lift off the floor
    print(f"Takeoff command: {alt}m") # logging the exact height
    master.mav.command_long_send(master.target_system, master.target_component, # sending another mavlink pulse
                                 mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, alt) # the takeoff pulse with the altitude i passed in

def get_altitude(): # i did this to check the current height from the floor
    msg = master.recv_match(type='VFR_HUD', blocking=True, timeout=1.0) # grabbing the altitude packet from the stream
    if msg: # if we actually caught a real message
        return msg.alt # return the height in meters
    return 0.0 # just return zero if it completely fails

def send_velocity(vx, vy, vz=0.0): # i use this to move the drone by setting the speed
    master.mav.set_position_target_local_ned_send( # sending the mavlink velocity pulse
        0, master.target_system, master.target_component, # making sure it targets the drone
        mavutil.mavlink.MAV_FRAME_BODY_NED, 0b0000111111000111, # setting the velocity mask bitmask thing
        0, 0, 0, vx, vy, vz, 0, 0, 0, 0, 0) # plugging in the speed vectors i want

#################### the main mission 2 logic i built

def main(): # the main boss function that runs it all
    try: # trying to fly without crashing into the wall
        # step 1 is the takeoff sequence
        change_mode("GUIDED") # telling the drone to ignore the sticks and listen to my code
        time.sleep(1) # wait a second for the mode to actually shift
        arm_drone() # spin up the propellers
        time.sleep(1) # wait for the motors to settle down
        takeoff(TAKEOFF_ALT_M) # lift off the floor finally

        # i make it wait until the target height is reached
        print("Climbing...") # log the takeoff progress
        while True: # loop forever until it is high enough
            alt = get_altitude() # check exactly how high we are right now
            if alt >= TAKEOFF_ALT_M * 0.9: # if we reached 90 percent of the target
                print(f"Altitude reached: {alt:.2f}m") # log that it was a huge success
                break # break out of the climbing loop
            time.sleep(0.5) # sleep so we only check every half second

        # step 2 the coordinated action where i tell the ground rover to drive
        print("[Mission] Commanding UGV to MOVE 10ft") # logging the chore
        bridge.send_command(cmdSeq=1, cmd=v2v_bridge.CMD_MOVE_FORWARD, estop=0) # blasting the command over the air

        # step 3 the actual flight maneuver
        print(f"[Mission] Flying {DIST_M}m forward...") # log the flight chore
        start_t = time.time() # i start the clock here
        telem_seq = 0 # setting up the status counter
        abort_triggered = False # i reset the safety flag just in case

        duration = DIST_M / SPEED_MPS # doing math to calculate the total fly time

        while (time.time() - start_t) < duration: # looping for the whole duration of the fly
            # safety check i added to listen for abort from the radio
            cmd = bridge.get_command() # yank the data from the command mailbox
            if cmd: # if we actually got a radio instruction
                cmdSeq, cmdVal, eStopFlag = cmd # i unpack the variables
                if eStopFlag == 1: # if some idiot hit the abort button
                    print("!!! ABORT RECEIVED: SWITCHING TO LAND !!!") # log the absolute panic
                    abort_triggered = True # flip the flag to abort
                    break # completely stop everything

            # another safety check i did so it doesnt fly out of the room into the ceiling
            alt = get_altitude() # checking the height again
            if alt > MAX_ALT_ALLOWED: # if we are way too high
                print(f"Altitude Warning: {alt:.2f}m. Forced Land.") # log that we hit the virtual fence
                abort_triggered = True # flip the panic flag
                break # stop flying forward immediately

            # forcing it to move forward
            send_velocity(SPEED_MPS, 0.0, 0.0) # pushing air back so we move forward

            # i tell the radio box that it is actually working
            t_ms = int(time.time() * 1000) & 0xFFFFFFFF # grabbing the drone time in milliseconds
            bridge.send_telemetry(telem_seq, t_ms, SPEED_MPS, 0.0, 0, 0) # shoving the pulse down the bridge
            telem_seq += 1 # bump the counter up
            
            time.sleep(0.1) # sleep so the logic runs at 10hz

        # step 4 landing where the autopilot handles sensing the ground
        print("[Mission] Finalizing: Landing now...") # log that we are finishing up
        change_mode("LAND") # forcing the mode to land for a slow descent
        
        # wait around until we finally hit the floor
        while True: # endless loop until it is safe
            alt = get_altitude() # check the altitude again
            if alt < 0.3: # if we are barely 30cm off the floor
                print("Landed safely.") # declare success cause we landed
                break # break out of the landing loop
            time.sleep(1) # check the height every single second
            
    except Exception as e: # if literally anything goes wrong
        print(f"Mission Error: {e}. Attempting EMERGENCY LAND.") # log the massive error panic
        change_mode("LAND") # try to come down safe by forcing land
    finally: # i always do this cleanup part
        bridge.stop() # closing the radio bridge wire so it doesnt get stuck
        print("Bridge stopped. Mission sequence closed.") # log the final cleanup

if __name__ == "__main__": # the standard python entry point
    main() # actually run the main boss function