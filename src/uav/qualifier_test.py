######### official qualifier test - safety demo

# QUALIFIER TEST
# objective: prove autonomous flight, motion, and emergency kill switches
# constraint: must stop both vehicles within 10 seconds of kill command
# requires: prop guards on drone, official log of velocities

import time # library for timing and competitive logs
from pymavlink import mavutil # to talk to drone
import v2v_bridge # our radio bridge talker

################################# test config
ESP32_PORT = "/dev/ttyUSB0" # drone radio box
DRONE_PORT = "/dev/ttyACM0" # drone controller wire
TARGET_ALT = 1.3            # approx 4.2 feet
TEST_SPEED = 0.5            # m/s for straight flight/drive

# official log for the refs
LOG_FILE = "qualifier_results_log.txt"

def log_event(text): # helper to write required logs
    timestamp = time.strftime("%H:%M:%S")
    line = f"[{timestamp}] {text}\n"
    print(line.strip())
    with open(LOG_FILE, "a") as f: f.write(line)

def get_uav_velocity(master): # helper to pull speed from drone
    msg = master.recv_match(type='VFR_HUD', blocking=True, timeout=1.0)
    return msg.groundspeed if msg else 0.0

def get_ugv_velocity(bridge): # helper to pull speed from rover radio
    data = bridge.get_telemetry()
    return data['v_mps'] if data else 0.0

############################ control logic

def main(): # the main safety check engine
    log_event("--- SYSTEM QUALIFIER TEST STARTING ---") # log start
    
    # 1. connect wires
    bridge = v2v_bridge.V2VBridge(ESP32_PORT, name="UAV-Qualifier") # radio bridge
    try: bridge.connect() # open serial wires
    except: return # fail if no wire

    log_event("Connecting to Drone Cube Orange...") # log startup
    master = mavutil.mavlink_connection(DRONE_PORT, baud=115200) # drone wire
    master.wait_heartbeat() # wait for pulse
    log_event("UAV Heartbeat OK.") # good

    # 2. UAV TEST PHASE
    log_event("\n[PHASE 1] UAV AUTONOMOUS TAKEOFF & FLIGHT") # drone part
    master.mav.set_mode_send(master.target_system, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 4) # GUIDED
    time.sleep(1) # let settle
    master.mav.command_long_send(master.target_system, master.target_component, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0,0,0,0,0,0) # ARM
    time.sleep(1) # wait for spinup
    master.mav.command_long_send(master.target_system, master.target_component, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0,0,0,0,0,0, TARGET_ALT) # TAKEOFF
    
    # wait for altitude
    while True: # loops until high
        # asking for the lidar range finder distance from the floor
        msg = master.recv_match(type='DISTANCE_SENSOR', blocking=True, timeout=1.0)
        if msg and (msg.current_distance / 100.0) >= (TARGET_ALT * 0.9): break # if high enough stop
    
    log_event(f"UAV Takeoff Successful. Current Velocity: {get_uav_velocity(master)} m/s") # log stats
    
    # straight flight
    log_event("Initiating Straight Path Flight...") # move forward
    master.mav.set_position_target_local_ned_send(
        0, master.target_system, master.target_component,
        mavutil.mavlink.MAV_FRAME_BODY_NED, 0b0000111111000111,
        0, 0, 0, TEST_SPEED, 0, 0, 0, 0, 0, 0, 0) # speed pulse
    time.sleep(3) # fly for 3 seconds
    
    # KILL SWITCH TEST (UAV)
    log_event("KILL SWITCH ACTIVATION: Triggering UAV Emergency Stop...") # kill start
    kill_start_t = time.time() # start clock
    # force disarm (don't wait for landing)
    master.mav.command_long_send(master.target_system, master.target_component, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 0, 21196.0, 0,0,0,0,0) # DISARM NOW
    
    # verify stop
    stopped = False # not yet
    while (time.time() - kill_start_t) < 15: # check for up to 15s (requirement is 10s)
        # if the heartbeat says disarmed, it's "killed"
        msg = master.recv_match(type='HEARTBEAT', blocking=True, timeout=1.0) # get pulse
        if msg and not (msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED): # if dead
            stop_t = time.time() # stop clock
            delta = stop_t - kill_start_t # calc time
            log_event(f"STOP VERIFICATION: UAV DISARMED AT T+{delta:.2f}s (GOAL < 10s)") # log success
            stopped = True # done
            break # exit loop
    
    if not stopped: log_event("!!! FAILURE: UAV KILL SWITCH TIMED OUT !!!") # failure log

    # 3. UGV TEST PHASE
    log_event("\n[PHASE 2] UGV AUTONOMOUS NAVIGATION & KILL") # rover part
    log_event("Commanding UGV Straight Motion...") # move forward
    bridge.send_command(cmdSeq=1, cmd=v2v_bridge.CMD_MOVE_FORWARD, estop=0) # start rover drive
    time.sleep(2) # drive a bit
    log_event(f"UGV Velocity: {get_ugv_velocity(bridge)} m/s") # log speed
    time.sleep(2) # drive more

    # KILL SWITCH TEST (UGV)
    log_event("KILL SWITCH ACTIVATION: Triggering UGV Emergency Stop...") # kill start
    u_kill_start_t = time.time() # start clock
    bridge.send_command(cmdSeq=2, cmd=v2v_bridge.CMD_STOP, estop=1) # BLAST THE ESTOP BIT
    
    u_stopped = False # not yet
    while (time.time() - u_kill_start_t) < 15: # check timing
        data = bridge.get_telemetry() # pull status
        # if armed bit goes to 0, or groundspeed hits 0
        if data and (data['armed'] == 0 or data['v_mps'] < 0.05): # if dead
            u_stop_t = time.time() # stop clock
            u_delta = u_stop_t - u_kill_start_t # calc time
            log_event(f"STOP VERIFICATION: UGV HALTED AT T+{u_delta:.2f}s (GOAL < 10s)") # log success
            u_stopped = True # done
            break # exit loop
        time.sleep(0.5) # wait

    if not u_stopped: log_event("!!! FAILURE: UGV KILL SWITCH TIMED OUT !!!") # failure log

    log_event("\n--- QUALIFIER TEST COMPLETE ---") # test end
    bridge.stop() # close bridge

if __name__ == "__main__": # entry point
    main() # run it
