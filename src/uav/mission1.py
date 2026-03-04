######### mission 1 - autonomous launch & moving landing

# MISSION 1: OPERATION TOUCHDOWN 
# objective: takeoff from ugv, hold height, track moving ugv, and land on it
# constraint: no gps allowed . vision and optical flow only
######### mission 1 autonomous launch and moving landing

import time # i pulled in the time library so i can do timing and print logs
import cv2 # i had to get opencv so the drone can actually see the aruco marker
import numpy as np # standard math junk i need for the vision stuff
from pymavlink import mavutil # library i need to actually talk to the drone flight controller
import v2v_bridge # the custom translator i built for the radio bridge talk

################################# competition config i set up
ESP32_PORT = "/dev/ttyUSB0" # the actual wire going to the radio box
DRONE_PORT = "/dev/ttyACM0" # the wire going straight to the drone controller
ARUCO_ID = 0                # the specific marker id i stuck on the rover deck
TARGET_ALT = 1.3            # making it go roughly 4.2 feet so it passes the 4ft minimum rule
RIDE_TIME_REQ = 30          # the stupid 30 seconds we have to just sit there after landing
MISSION_TIMEOUT = 420       # 7 minutes max time before the judges yell at us

# the official logging file i had to make for the competition refs
LOG_FILE = "mission1_official_log.txt"

def log_event(text): # helper function i wrote just to do the required logs
    timestamp = time.strftime("%H:%M:%S") # yanking the current clock time
    line = f"[{timestamp}] {text}\n" # formatting the line so it looks official
    print(line.strip()) # shoving it in the console so i can see it
    with open(LOG_FILE, "a") as f: f.write(line) # physically saving it to the text file

#################### vision brain for the zed x camera

class Tracker: # the helper class i wrote to find the rover in the camera feed
    def __init__(self): # setting up the vision detector engine
        # i am using the standard aruco dictionary cause that is what they use for competition markers
        self.dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.params = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(self.dict, self.params)

    def find_ugv(self, frame): # the math to actually find the marker center x and y
        if frame is None: return None # just abort if the camera is completely dead
        corners, ids, _ = self.detector.detectMarkers(frame) # doing a sweep to look for any markers
        if ids is not None and ARUCO_ID in ids: # if we actually found our specific marker
            idx = list(ids.flatten()).index(ARUCO_ID) # i find exactly where it is in the array
            c = corners[idx][0] # grabbing the 4 corner points of the square
            center_x = np.mean(c[:, 0]) # calculating the horizontal center pixel
            center_y = np.mean(c[:, 1]) # calculating the vertical center pixel
            return (center_x, center_y) # spitting out the exact pixel coordinates
        return None # spitting out nothing if we lost it

############################ drone control stuff

def send_velocity(master, vx, vy, vz): # i made this to physically move the drone by setting its speed
    # i used the body ned frame so vx is forward and back and vy is right and left relative to where the drone is looking
    master.mav.set_position_target_local_ned_send(
        0, master.target_system, master.target_component,
        mavutil.mavlink.MAV_FRAME_BODY_NED, 0b0000111111000111,
        0, 0, 0, vx, vy, vz, 0, 0, 0, 0, 0)

def main(): # the main boss engine that runs the mission
    log_event("--- MISSION 1 STARTING: OPERATION TOUCHDOWN ---") # yelling that we are starting
    
    # step 1 i setup the basic radio and drone connections
    bridge = v2v_bridge.V2VBridge(ESP32_PORT, name="UAV-Mission-1") # creating the radio bridge object
    try: bridge.connect() # forcing the serial wires to open
    except: return # just completely fail and bail out if there is no wire

    log_event("UAV Starting. Connecting to Cube Orange...") # logging the startup sequence
    master = mavutil.mavlink_connection(DRONE_PORT, baud=115200) # opening the actual drone connection
    master.wait_heartbeat() # waiting around for the drone pulse
    log_event("Drone Heartbeat OK. Sensors Ready.") # printing that we are good to go

    # step 2 i force it to sync up with the ground rover link
    log_event("Waiting for UGV radio link...") # hunting for the rover over the air
    ugv_synced = False # setting flag to false cause we havent found it yet
    while not ugv_synced: # endless loop until it talks back
        if bridge.get_telemetry(): # if a status packet finally comes in
            log_event("UGV Found. Communications verified.") # logging that the logic is ok
            ugv_synced = True # flip the flag to stop the loop
        time.sleep(1.0) # sleep a sec so we dont fry the cpu

    # step 3 is the scary autonomous launch part
    log_event("UAV START TIME: Initiating Launch...") # printing the official log required by rules
    master.mav.set_mode_send(master.target_system, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 4) # forcing it into GUIDED mode
    time.sleep(1) # i let the mode settle for a bit
    master.mav.command_long_send(master.target_system, master.target_component, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0,0,0,0,0,0) # bullying it to ARM
    time.sleep(1) # waiting for the props to actually spin up
    master.mav.command_long_send(master.target_system, master.target_component, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0,0,0,0,0,0, TARGET_ALT) # screaming at it to TAKEOFF
    
    launch_t = time.time() # i start the clock right here
    while True: # loop to just watch it climb until it hits the target height
        # asking the flight controller for the raw LIDAR distance from the floor
        msg = master.recv_match(type='DISTANCE_SENSOR', blocking=True, timeout=1.0)
        if msg and (msg.current_distance / 100.0) >= (TARGET_ALT * 0.9): break # check height in meters
        time.sleep(0.5) # checking again in half a second
    
    hold_duration = time.time() - launch_t # doing math to calculate the total climb time
    log_event(f"UAV reached {TARGET_ALT}m altitude. Duration: {hold_duration:.1f}s") # logging the height and time
    if hold_duration < 5.0: # doing a competition check cause they have rules about this
        time.sleep(5.0 - hold_duration) # i make it wait extra time if the climb was way too fast

    # step 4 is tracking the rover and praying it lands
    log_event("UGV START TIME: Commanding UGV to move...") # log that i am telling the rover to start driving
    bridge.send_command(cmdSeq=1, cmd=v2v_bridge.CMD_MISSION_1, estop=0) # i blast the command to actually start the rover wheels
    
    tracker = Tracker() # i spin up the vision logic i wrote earlier
    # cam = cv2.VideoCapture(0) # this is where the zed x camera stream goes but i stubbed it out for testing
    
    log_event("Tracking UGV. Alignment Sequence Running...") # logging that the tracking just started
    landing_start_t = time.time() # i start the clock again so it doesnt track forever
    is_home = False # setting the landed flag to false cause we are obviously still flying
    
    while not is_home: # the massive alignment loop
        if (time.time() - launch_t) > MISSION_TIMEOUT: # checking if we hit the 7 minute mark
            log_event("!!! FAILURE: TIMEOUT EXCEEDED (7 MIN) !!!") # log a massive fail
            break # just completely give up and break
            
        # step 4 part 1 is actually finding the rover deck
        # ret, frame = cam.read() # yanking a frame from the camera
        target = tracker.find_ugv(None) # running my math to find the marker in the frame
        
        if target: # if we actually see it with the camera
            # log_event("DESTINATION DISCOVERY: UGV Deck found.")
            # the logic i had to do for tracking 
            # if the target is left of center then vy has to be negative
            # if the target is ahead of center then vx has to be positive
            send_velocity(master, 0.2, 0.0, 0.1) # sending a fake tracking pulse just to drift down for now
        else: # if we totally lost sight of it
            send_velocity(master, 0.0, 0.0, 0.0) # force it to do a station hold if lost so it doesnt fly away

        # checking if we actually touched down on the deck
        msg = master.recv_match(type='HEARTBEAT', blocking=False) # checking the pulse real quick
        if msg and not (msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED): # if it is disarmed that literally means it landed
            log_event(f"UAV LANDING TIME: Touchdown confirmed.") # log the successful landing
            is_home = True # flip the flag cause we are done
            break # completely exit the tracking loop
            
        time.sleep(0.1) # making this a 10hz loop so it is smooth

    # step 5 is just riding it out
    if is_home: # if we actually landed and didnt crash
        log_event("Initiating 30-Second Ride Duration...") # start the stupid ride timer
        ride_start = time.time() # start the clock again
        while (time.time() - ride_start) < RIDE_TIME_REQ: # loop until we hit the 30 seconds
            time.sleep(1.0) # sleep a second at a time
        
        log_event("RIDE DURATION TIME COMPLETE: 30s achieved.") # log that the ride is a success
        log_event("MISSION 1 SUCCESSFUL") # log that we are completely done done
        bridge.send_command(cmdSeq=2, cmd=v2v_bridge.CMD_STOP, estop=0) # scream at the rover to finally stop driving
    
    bridge.stop() # safely close the bridge so the port doesnt get stuck

if __name__ == "__main__": # standard entry point junk
    main() # run the whole thing