"""
Challenge 1 - UAV Precision Landing on Moving UGV
=================================================
Hardware: ZED X Camera, Cube Orange+, Holybro H-Flow, Lidar Lite V3
Logic: 
  - Stabilize using Optical Flow + Lidar
  - Detect nested ArUco markers (Large: ID 10, Small: ID 20)
  - Estimate 6D pose -> convert to Body Frame FRD
  - Stream LANDING_TARGET messages at 15Hz
  - Automate flight mode transitions: LOITER -> PRECISION LOITER -> LAND
"""
import time
import math
import cv2
import numpy as np
from pymavlink import mavutil

# ─── USER CONFIGURATION ────────────────────────────────────────────────────────
MAVLINK_CONN = "/dev/ttyACM0"      # Or udp:127.0.0.1:14550
BAUD_RATE    = 921600

# Nested ArUco Marker Setup
LARGE_MARKER_ID   = 10
SMALL_MARKER_ID   = 20
LARGE_MARKER_SIZE = 0.50   # 50 cm
SMALL_MARKER_SIZE = 0.16   # 16 cm

# Switch to the small marker when altitude is below this (meters)
USE_SMALL_BELOW_M = 1.6

# Send rate for LANDING_TARGET
SEND_HZ = 15.0

# Flight Logic
TARGET_HEIGHT_M = 2.0
PLND_STABLE_FRAMES = 15   # Frames of solid tracking before engaging Precision Loiter
LAND_LATERAL_ERR_M = 0.20 # Meters of allowed lateral error before switching to LAND

# Camera calibration variables (will be populated from ZED SDK dynamically)
CAM_MATRIX = None
DIST_COEFFS = None

# ─── MAVLINK HELPERS ───────────────────────────────────────────────────────────
master = mavutil.mavlink_connection(MAVLINK_CONN, baud=BAUD_RATE)
print("Waiting for heartbeats from UAV...")
master.wait_heartbeat()
print("Heartbeat received!")

def send_landing_target(x_b, y_b, z_b):
    """
    Send LANDING_TARGET to ArduPilot in Body Frame (FRD).
    x_b = Forward (meters)
    y_b = Right (meters)
    z_b = Down (meters)
    """
    master.mav.landing_target_send(
        int(time.time() * 1e6),              # time_usec
        0,                                   # target_num (0 = default)
        mavutil.mavlink.MAV_FRAME_BODY_FRD,  # frame
        0.0,                                 # angle_x (not used for body frame)
        0.0,                                 # angle_y (not used for body frame)
        abs(z_b),                            # distance 
        0.0, 0.0,                            # size_x, size_y (rad)
        x_b,                                 # x position
        y_b,                                 # y position
        abs(z_b),                            # z position
        (1.0, 0.0, 0.0, 0.0),                # q
        0,                                   # type
        1                                    # position_valid flag
    )

def change_mode(mode_name: str):
    mode_id = master.mode_mapping().get(mode_name)
    if mode_id is None:
        print(f"Unknown mode {mode_name}")
        return False
    master.set_mode(mode_id)
    return True

def get_rel_alt_m():
    msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=False)
    if msg:
        return msg.relative_alt / 1000.0
    return None

def arm_and_takeoff(alt):
    print("Arming motors...")
    master.arducopter_arm()
    master.motors_armed_wait()
    print("Armed! Taking off...")
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0, 0, 0, 0, 0, 0, 0, alt
    )

# ─── CAMERA SETUP ──────────────────────────────────────────────────────────────
# We are using PyZED exclusively based on working hardware config
try:
    import pyzed.sl as sl
except ImportError:
    print("CRITICAL ERROR: pyzed.sl is not installed. You must install the ZED SDK on this Jetson!")
    exit(1)

zed = sl.Camera()
init_params = sl.InitParameters()
init_params.camera_resolution = sl.RESOLUTION.HD1080
init_params.camera_fps = 30
init_params.depth_mode = sl.DEPTH_MODE.NONE

err = zed.open(init_params)
if err != sl.ERROR_CODE.SUCCESS:
    print(f"Failed to open ZED camera: {err}")
    exit(1)

print("ZED X initialized perfectly.")
zed_img = sl.Mat()

# Pull real hardware calibration parameters
cam_info = zed.get_camera_information()
calib = cam_info.camera_configuration.calibration_parameters.left_cam
CAM_MATRIX = np.array([
    [calib.fx, 0.0, calib.cx],
    [0.0, calib.fy, calib.cy],
    [0.0, 0.0, 1.0],
], dtype=np.float32)

dist = np.array(calib.disto, dtype=np.float32).flatten()
if dist.size >= 5:
    DIST_COEFFS = dist[:5].reshape(-1, 1)
else:
    DIST_COEFFS = dist.reshape(-1, 1)

aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
aruco_params = cv2.aruco.DetectorParameters()
try:
    aruco_detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)
except AttributeError:
    aruco_detector = None # OpenCV older version handle later

# ─── STATE MACHINE ─────────────────────────────────────────────────────────────
state = "TAKEOFF"
stable_count = 0
last_send = 0.0

print("Starting Precision Landing State Machine")
print("Press 'q' or Ctrl+C to quit.")

try:
    # Set to GUIDED (doesn't require GPS if Optical Flow is active)
    change_mode("GUIDED")
    arm_and_takeoff(TARGET_HEIGHT_M)
    state = "APPROACH"
    
    while True:
        # Grab frame with ZED SDK reliably
        if zed.grab() != sl.ERROR_CODE.SUCCESS:
            print("Failed to grab ZED frame.")
            time.sleep(0.01)
            continue
            
        zed.retrieve_image(zed_img, sl.VIEW.LEFT)
        frame_data = zed_img.get_data()
        
        # Exact channel fix to prevent the 'green screen' bug
        if frame_data.shape[-1] == 4:
            frame = cv2.cvtColor(frame_data, cv2.COLOR_BGRA2BGR)
        else:
            frame = frame_data.copy()

        # Detect markers
        if aruco_detector:
            corners, ids, _ = aruco_detector.detectMarkers(frame)
        else:
            corners, ids, _ = cv2.aruco.detectMarkers(frame, aruco_dict, parameters=aruco_params)

        rel_alt = get_rel_alt_m()
        if rel_alt is None: rel_alt = 2.0

        # Choose target size based on altitude
        marker_id = LARGE_MARKER_ID
        marker_size = LARGE_MARKER_SIZE
        if rel_alt < USE_SMALL_BELOW_M:
            marker_id = SMALL_MARKER_ID
            marker_size = SMALL_MARKER_SIZE

        found = False
        target_eb = None

        if ids is not None and len(ids) > 0:
            ids_flat = ids.flatten()
            if marker_id in ids_flat:
                idx = int(np.where(ids_flat == marker_id)[0][0])

                # 3D object points
                half_s = marker_size / 2.0
                obj_pts = np.array([
                    [-half_s,  half_s, 0],
                    [ half_s,  half_s, 0],
                    [ half_s, -half_s, 0],
                    [-half_s, -half_s, 0]
                ], dtype=np.float32)

                img_pts = corners[idx].reshape(4, 2).astype(np.float32)

                # Solve PnP
                ok, rvec, tvec = cv2.solvePnP(obj_pts, img_pts, CAM_MATRIX, DIST_COEFFS, flags=cv2.SOLVEPNP_IPPE_SQUARE)

                if ok:
                    # Convert to Body Frame (FRD)
                    # Downward camera: X_img(Right) -> Y_body, Y_img(Down_img/Back_veh) -> -X_body
                    x_cam, y_cam, z_cam = tvec[0][0], tvec[1][0], tvec[2][0]
                    x_b = -y_cam  # Forward
                    y_b = x_cam   # Right
                    z_b = z_cam   # Down (Depth)
                    
                    target_eb = (x_b, y_b, z_b)

                    # Send landing target at required frequency
                    now = time.time()
                    if now - last_send >= (1.0 / SEND_HZ):
                        send_landing_target(x_b, y_b, z_b)
                        last_send = now

                    stable_count += 1
                    found = True

                    # Visualization
                    cv2.aruco.drawDetectedMarkers(frame, [corners[idx]], np.array([[marker_id]]))
                    cv2.putText(frame, f"X:{x_b:.2f}m Y:{y_b:.2f}m Z:{z_b:.2f}m", (20, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

        if not found:
            stable_count = 0
            cv2.putText(frame, "TARGET LOST", (20, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

        # STATE MACHINE LOGIC
        if state == "APPROACH":
            if stable_count >= PLND_STABLE_FRAMES:
                print(">>> Target acquired and stable. Holding in GUIDED until perfectly aligned.")
                # We stay in GUIDED while sending TARGET messages. 
                state = "PREC_LOITER"
        
        elif state == "PREC_LOITER":
            if stable_count == 0:
                print(">>> Target lost during PREC_LOITER. Switching back to APPROACH.")
                state = "APPROACH"
            elif found and target_eb is not None:
                err_m = math.sqrt(target_eb[0]**2 + target_eb[1]**2)
                cv2.putText(frame, f"Error: {err_m:.2f}m", (20, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
                
                if err_m < LAND_LATERAL_ERR_M:
                    print(">>> Aligned perfectly! Switching to LAND.")
                    change_mode("LAND")
                    state = "LANDING"
        
        elif state == "LANDING":
            if not found:
                print(">>> WARNING: Target lost during LANDING. (Trusting PLND_STRICT parameters)")
            # You can check if disarmed here and trigger your electromagnet lock mechanism
            if not master.motors_armed():
                print(">>> TOUCHDOWN DETECTED. DISARMED.")
                state = "LANDED"
                break
        
        cv2.putText(frame, f"STATE: {state}", (20, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 100, 255), 2)
        cv2.imshow("Precision Landing", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

finally:
    print("Cleaning up...")
    if 'zed' in locals() and zed is not None:
        zed.close()
    cv2.destroyAllWindows()
    # Failsafe abort mode
    if state != "LANDED":
        change_mode("GUIDED")
