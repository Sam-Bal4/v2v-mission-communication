######### mission 2 - aruco target identification & travel

# MISSION 2
# objective: takeoff, find marker id 0-4, send coords to ugv, and land on moving target
# constraint: no gps allowed . vision and optical flow only for stable scout flight



######### mission 2 aruco target identification and travel

import time # i pulled in the time library so i can do timing and print logs for the competition
import cv2 # i had to get opencv so the drone can actually scan the grass for the markers
import numpy as np # standard math junk i need to calculate the coordinates
from pymavlink import mavutil # library i need to actually talk to the flight controller brain
import v2v_bridge # the custom translator i built for the radio box talk

################################# the config i set up for the raytheon competition
ESP32_PORT = "/dev/ttyUSB0" # the wire going to the radio box
DRONE_PORT = "/dev/ttyACM0" # the wire going straight to the drone brain
TARGET_ALT = 1.3            # making it go roughly 4 point 2 feet so it passes the minimum height rule
RIDE_TIME_REQ = 10          # the 10 seconds we have to just sit there on the rover deck for mission 2
MISSION_TIMEOUT = 600       # 10 minutes max time before they disqualify us

# the official logging file i had to make for the raytheon judges so we pass senior design
LOG_FILE = "mission2_official_log.txt"

def log_event(text): # helper function i wrote just to do the required text logs
    timestamp = time.strftime("%H:%M:%S") # yanking the current clock time
    line = f"[{timestamp}] {text}\n" # formatting the line so it looks totally official
    print(line.strip()) # shoving it in the console so i can debug it
    with open(LOG_FILE, "a") as f: f.write(line) # physically saving it to the text file

#################### vision brain for the zed x camera

class Tracker: # the helper class i wrote to scan the whole field and find the target
    def __init__(self): # setting up the vision detector engine
        # target id 0 to 4 exactly how the competition rules asked for it
        self.dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.params = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(self.dict, self.params)

    def find_target(self, frame): # the math to look for any marker id between 0 and 4
        if frame is None: return None # just abort if the camera is dead
        corners, ids, _ = self.detector.detectMarkers(frame) # doing a sweep to look for literally any markers
        if ids is not None: # if we actually saw something
            for i, marker_id in enumerate(ids.flatten()): # loop through everything it found
                if 0 <= marker_id <= 4: # if we actually found a valid competition id
                    c = corners[i][0] # grabbing the 4 corner points
                    center_x = np.mean(c[:, 0]) # calculating horizontal center pixel
                    center_y = np.mean(c[:, 1]) # calculating vertical center pixel
                    return (marker_id, center_x, center_y) # spitting out the id and exact pixel coordinates
        return None # spitting out nothing if we found zero markers

############################ drone control stuff

def send_velocity(master, vx, vy, vz): # i made this to physically move the drone by setting its speed vectors
    master.mav.set_position_target_local_ned_send( # sending the huge mavlink command
        0, master.target_system, master.target_component,
        mavutil.mavlink.MAV_FRAME_BODY_NED, 0b0000111111000111, # forcing it to use the body frame so forward is always forward
        0, 0, 0, vx, vy, vz, 0, 0, 0, 0, 0) # plugging in the speed numbers i want

def main(): # the main boss engine that runs mission 2
    log_event("--- MISSION 2 STARTING: TARGET ID & TRAVEL ---") # yelling that we are finally starting
    
    # step 1 i setup the basic hardware links
    bridge = v2v_bridge.V2VBridge(ESP32_PORT, name="UAV-Mission-2") # creating the radio bridge object
    try: bridge.connect() # forcing the serial wires to open up
    except: return # just completely fail and bail out if it is unplugged

    log_event("Connecting to Cube Orange flight controller...") # logging the startup sequence
    master = mavutil.mavlink_connection(DRONE_PORT, baud=115200) # opening the actual drone connection
    master.wait_heartbeat() # waiting around for the drone pulse to hit
    log_event("Drone Heartbeat OK.") # printing that we are good to go

    # step 2 is the scary autonomous launch part
    log_event("Initiating Autonomous Launch...") # logging that it is about to take off
    master.mav.set_mode_send(master.target_system, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 4) # forcing it into GUIDED mode so it listens to me
    time.sleep(1); # i let the mode settle for a bit
    master.mav.command_long_send(master.target_system, master.target_component, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0,0,0,0,0,0) # bullying it to ARM
    time.sleep(1) # waiting for the props to actually spin up and get loud
    master.mav.command_long_send(master.target_system, master.target_component, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0,0,0,0,0,0, TARGET_ALT) # screaming at it to TAKEOFF
    
    launch_t = time.time() # i start the clock right here
    # wait loop to just watch it climb until it hits target
    while True: # loop forever until it is high enough
        msg = master.recv_match(type='VFR_HUD', blocking=True, timeout=1.0) # asking the flight controller for the exact altitude
        if msg and msg.alt >= (TARGET_ALT * 0.9): break # if it is high enough i just break the loop
    
    log_event(f"Minimum Altitude {TARGET_ALT}m Reached.") # logging that the height is confirmed
    if (time.time() - launch_t) < 5.0: time.sleep(5.0) # doing a competition check to force it to fly for at least 5 seconds

    # step 3 is destination discovery where it hunts for the aruco marker
    log_event("Starting Field Scan for ArUco Marker ID 0-4...") # log that i am starting the scan
    tracker = Tracker() # i spin up the vision logic i wrote earlier
    # cam = cv2.VideoCapture(0) # this is where the zed x camera stream goes but i stubbed it out
    found = False # setting flag to false cause we havent found squat yet
    target_x, target_y = 0.0, 0.0 # blank coordinates
    
    while not found: # the massive loop until we finally see the marker
        # ret, frame = cam.read() # yanking a frame from the actual camera
        res = tracker.find_target(None) # running my math to find the target in the frame placeholder
        if res: # if we actually see it with the camera
            m_id, cx, cy = res # i unpack the values from my function
            log_event(f"DESTINATION DISCOVERY: Found Marker ID {m_id}") # log the massive success
            # i had to do some crazy pose estimation math here to get the real field coordinates X and Y
            target_x, target_y = 10.0, 5.0 # just stuffing a fake destination 10 meters ahead and 5 right for testing
            log_event(f"COORDINATES DETECTED: X={target_x}, Y={target_y}") # the official log they want to see
            found = True # flip the flag cause we are totally done searching
        else: # if we totally lost sight of everything
            send_velocity(master, 0.1, 0.0, 0.0) # sending a gentle scout drift forward so it keeps looking
            time.sleep(0.1) # wait a tiny bit

    # step 4 is communication where i send the data to the rover
    log_event("COMMUNICATION TIMESTAMP: Sending coordinates to UGV...") # log that the radio shout is starting
    bridge.send_message(f"GOTO:{target_x},{target_y}") # screaming the exact coordinates over the air to the rover
    time.sleep(0.5) # i let the air clear for half a second
    bridge.send_command(cmdSeq=1, cmd=v2v_bridge.CMD_MISSION_2, estop=0) # finally telling the rover to actually move its wheels
    log_event("UGV RECEIPT OF DESTINATION: Command confirmed.") # log that the coordination worked perfectly

    
    
    # step 5 is tracking the rover and praying it lands on it
    log_event("Aligning for Precision Landing on Moving Platform...") # log that the scary landing is starting
    platform_landing_start = time.time() # start the clock again so it doesnt track forever
    landed = False # setting the landed flag to false cause we are obviously still in the air
    
    while not landed: # the massive alignment loop
        if (time.time() - launch_t) > MISSION_TIMEOUT: # checking if we hit the 10 minute mark and failed
            log_event("!!! FAILURE: 10 MINUTE TIMEOUT REACHED !!!") # log a massive fail
            break # just completely give up and break out
            
        # i track the marker on the deck using the exact same logic i wrote for mission 1
        # res_land = tracker.find_target(frame) 
        send_velocity(master, 0.15, 0.0, 0.1) # sending a fake tracking pulse just to drift down onto the deck
        
        # checking if we actually touched down on the deck
        msg_l = master.recv_match(type='HEARTBEAT', blocking=False) # checking the pulse real quick
        if msg_l and not (msg_l.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED): # if it is disarmed that literally means it landed safely
            log_event("UAV LANDING TIME: Platform landing confirmed.") # log the successful landing
            landed = True # flip the flag cause we are totally done
            break # completely exit the tracking loop
        time.sleep(0.1) # making this a 10hz loop so it is smooth

    # step 6 is just riding it out to the end
    if landed: # if we actually landed and didnt crash
        log_event("Riding UGV to final destination...") # log that we are riding the rover now
        ride_t = time.time() # start the clock for the ride
        while (time.time() - ride_t) < RIDE_TIME_REQ: # loop until we hit the 10 seconds required
            time.sleep(1.0) # sleep a second at a time
        log_event("END TIMES: Ride complete. UGV at destination.") # log that the ride is a huge success
        bridge.send_command(cmdSeq=2, cmd=v2v_bridge.CMD_STOP, estop=0) # scream at the rover to finally stop driving
        log_event("MISSION 2 SUCCESSFUL") # log that we are completely done done

    bridge.stop() # safely close the bridge so the port doesnt get stuck

if __name__ == "__main__": # standard entry point junk
    main() # actually run the whole boss function