from dronekit import connect, VehicleMode # i am using dronekit now cause its way easier to boss the drone around
import time # pulling this in so i can do sleeps and not melt the brain
import v2v_bridge # our custom radio translator for talking to the ground rover
from pymavlink import mavutil # still need this just to catch the raw lidar packets

# uav mission 4 - autonomous flight + ugv circle
# the goal is to takeoff and hang in the air while the rover does circles

################################# config stuff i setup
# where the hardware is actually plugged into the jetson
CONNECTION_STRING = "/dev/ttyACM0"   # the wire for the cube orange controller
BAUD_RATE = 115200                   # standard speed for the serial wire
ESP32_PORT = "/dev/ttyUSB0"          # the usb wire for the radio bridge box

# mission params
TARGET_ALT = 1.3  # hovering at 4.2 feet so it passes the 4ft rule
CIRCLE_TIME = 18.0 # letting the rover spin for 18 seconds total

def get_lidar_alt(vehicle): # i wrote this to check the real distance from the floor
    # i have to grab the raw distance sensor packet from the stream
    msg = vehicle._master.recv_match(type='DISTANCE_SENSOR', blocking=True, timeout=1.0)
    if msg: # if we actually caught a real signal
        return msg.current_distance / 100.0 # convert cm to meters
    return 0.0 # return zero if the lidar is being a ghost

def arm_and_takeoff(vehicle, target_alt): # the sequence to get into the air
    print("Pre-arm checks...") # logging the start
    while not vehicle.is_armable: # loop until the sensors stop complaining
        print(" Waiting for vehicle to initialize...") # printing status
        time.sleep(1) # wait for hardware to settle
        
    print("Arming motors...") # telling it to spin the props
    vehicle.mode = VehicleMode("GUIDED") # forcing guided mode so the code has control
    vehicle.armed = True # flipping the arm switch
    
    while not vehicle.armed: # loop until it actually starts spinning
        print(" Waiting for arming...") # logging the wait
        time.sleep(1) # checking every second
        
    print("Taking off!") # shouting that we are lifting off
    vehicle.simple_takeoff(target_alt) # sending the takeoff pulse
    
    while True: # loop to watch the climb
        alt = get_lidar_alt(vehicle) # checking the lidar height
        print(f" Altitude: {alt:.2f}m", end='\r') # print progress on one line
        if alt >= target_alt * 0.95: # if we are at 95 percent height we stop
            print(f"\nTarget altitude reached: {alt:.2f}m") # declare success
            break # break the climb loop
        time.sleep(0.5) # pause so we dont spam the console

def main(): # the main boss function
    print("==========================================") # header
    print("   UAV MISSION 4 - DRONEKIT EDITION") # title
    print("==========================================") # footer
    
    # step 1: connect to the drone brain
    print(f"Connecting to UAV at {CONNECTION_STRING}...") # logging connection
    vehicle = connect(CONNECTION_STRING, wait_ready=True, baud=BAUD_RATE) # opening the dronekit link
    
    # step 2: start the radio link
    print(f"Starting V2V Bridge on {ESP32_PORT}...") # logging radio start
    bridge = v2v_bridge.V2VBridge(ESP32_PORT, name="UAV-Bridge") # bridge object
    try: # trying to open the wire
        bridge.connect() # opening the serial port
        bridge.send_message("MISSION 4: UAV AIRBORNE START") # yell over the air
    except Exception as e: # if the radio is missing
        print(f"Radio Fail: {e}") # log the fail
        return # bail out

    try: # wrapping the mission in a try catch
        # step 3: takeoff phase
        arm_and_takeoff(vehicle, TARGET_ALT) # lift off the floor
        
        # step 4: sync with rover
        print("Waiting for UGV sync...") # logging the wait
        while True: # loop until they talk
            data = bridge.get_telemetry() # pull mailbox
            if data: # if we got a real packet
                print("UGV Synced. Commanding Circles!") # log coordination
                break # stop waiting
            time.sleep(1) # check every second

        # step 5: command the maneuvers
        print(">>> INITIATING UGV CIRCLE SEQUENCE") # starting the work
        bridge.send_command(cmdSeq=400, cmd=v2v_bridge.CMD_CIRCLE, estop=0) # blast command
        
        start_mission = time.time() # start the clock
        while (time.time() - start_mission) < CIRCLE_TIME: # loop for duration
            data = bridge.get_telemetry() # check status
            if data: # if we got one
                v_mps = data[2] # pull speed
                elapsed = time.time() - start_mission # calc time
                print(f" Mission T+{elapsed:.1f}s | UGV Speed: {v_mps:.2f} m/s", end='\r') # log
            time.sleep(0.5) # slow loop

        print("\nCIRCLE SEQUENCE COMPLETE.") # mission done
        
        # step 6: come home
        print("Landing...") # start descent
        vehicle.mode = VehicleMode("LAND") # force landing mode
        while vehicle.armed: # loop while props are spinning
            alt = get_lidar_alt(vehicle) # check height
            print(f" Landing Alt: {alt:.2f}m", end='\r') # log progress
            time.sleep(1) # check once a second

    except KeyboardInterrupt: # if someone hits ctrl+c
        print("\nEmergency Interruption!") # log panic
        vehicle.mode = VehicleMode("LAND") # force land anyway
    finally: # always do cleanup
        bridge.stop() # close radio wire
        vehicle.close() # close drone link
        print("System shutdown. Mission closed.") # final log

if __name__ == "__main__": # standard entry point
    main() # run the whole show
