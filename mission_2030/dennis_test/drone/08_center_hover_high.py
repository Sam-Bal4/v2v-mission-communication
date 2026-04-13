import time
import math
import cv2
try:
    import pyzed.sl as sl
except ImportError:
    sl = None
from pymavlink import mavutil
import sys
import os
sys.path.append(os.path.abspath("../../"))
from mission_2030.common.mavlink_utils import arm_vehicle, wait_disarm, get_lidar_alt
from mission_2030.common.vision_utils import ArucoDetectorShim

def set_velocity_body(master, vx, vy, vz):
    master.mav.set_position_target_local_ned_send(
        0, master.target_system, master.target_component,
        mavutil.mavlink.MAV_FRAME_BODY_NED, 0b0000111111000111,
        0, 0, 0, vx, vy, vz, 0, 0, 0, 0, 0
    )

def main():
    print("--- Test 8: Center Tracking Hover (High) ---")
    if sl is None:
        print("ERROR: ZED SDK (pyzed) not installed. This test requires a Jetson with ZED SDK.")
        return

    cam = sl.Camera()
    params = sl.InitParameters(camera_resolution=sl.RESOLUTION.HD720)
    cam.open(params)
    zed_img = sl.Mat()
    point_cloud = sl.Mat()
    
    detector = ArucoDetectorShim(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50))

    master = mavutil.mavlink_connection("/dev/ttyACM0", baud=57600)
    master.wait_heartbeat()
    
    mapping = master.mode_mapping()
    print("Arming...")
    if not arm_vehicle(master):
        return

    print("Takeoff → 1.3m")
    master.mav.command_long_send(master.target_system, master.target_component, 
                                 mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, 1.3)
    
    # Wait for altitude
    t_takeoff = time.time()
    while time.time() - t_takeoff < 15:
        alt = get_lidar_alt(master)
        if alt and alt >= 1.3 * 0.9:
            print("Altitude reached ✓")
            break
        time.sleep(0.2)

    print("Hovering and centering over ArUco (15 seconds)...")
    start_t = time.time()
    kp = 0.5 # Proportional constraint

    while time.time() - start_t < 15:
        if cam.grab() == sl.ERROR_CODE.SUCCESS:
            cam.retrieve_image(zed_img, sl.VIEW.LEFT)
            cam.retrieve_measure(point_cloud, sl.MEASURE.XYZ)
            gray = cv2.cvtColor(cv2.cvtColor(zed_img.get_data(), cv2.COLOR_BGRA2BGR), cv2.COLOR_BGR2GRAY)
            
            corners, ids, _ = detector.detectMarkers(gray)
            if ids is not None and 0 in ids:
                idx = list(ids.flatten()).index(0)
                c = corners[idx][0]
                cx, cy = int((c[0][0] + c[2][0])/2), int((c[0][1] + c[2][1])/2)
                err, p3d = point_cloud.get_value(cx, cy)
                if err == sl.ERROR_CODE.SUCCESS and not math.isnan(p3d[0]):
                    x, y, z = float(p3d[0]), float(p3d[1]), float(p3d[2])
                    
                    # Convert X/Y drift to corrective velocity
                    v_right = x * kp
                    v_forward = -y * kp  # Image Y goes down, forward is up
                    
                    # clamp
                    v_right = max(-0.5, min(0.5, v_right))
                    v_forward = max(-0.5, min(0.5, v_forward))
                    
                    print(f"Centering -> Forward:{v_forward:.2f} Right:{v_right:.2f}")
                    set_velocity_body(master, v_forward, v_right, 0.0)
                    time.sleep(0.05)
                    continue
        
        # If no marker, hold position
        set_velocity_body(master, 0.0, 0.0, 0.0)
        time.sleep(0.05)

    print("Landing...")
    master.mav.set_mode_send(master.target_system, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, mapping["LAND"])
    if wait_disarm(master, 60):
        print("Test successful ✓")
    else:
        print("Disarm timeout.")

if __name__ == "__main__":
    main()
