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
from mission_2030.common.mavlink_utils import arm_vehicle, wait_disarm, get_lidar_alt, is_vehicle_disarmed

def main():
    print("--- Test 10: Precision Touchdown ---")
    if sl is None:
        print("ERROR: ZED SDK (pyzed) not installed. This test requires a Jetson with ZED SDK.")
        return

    cam = sl.Camera()
    params = sl.InitParameters(camera_resolution=sl.RESOLUTION.HD720)
    cam.open(params)
    zed_img = sl.Mat()
    point_cloud = sl.Mat()
    
    detector = cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50))

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

    print("Switching to LAND mode and streaming TARGET_ANGLES based on ZED physical depth...")
    master.mav.set_mode_send(master.target_system, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, mapping["LAND"])

    start_t = time.time()
    landed = False

    while not landed and time.time() - start_t < 60:
        # Check landing status instantly!
        if is_vehicle_disarmed(master):
            print("Landed and Disarmed successfully ✓")
            landed = True
            break

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
                    
                    # Compute pseudo-angles exactly
                    angle_x = math.atan2(x, z)
                    angle_y = math.atan2(y, z)
                    
                    print(f"Angle X: {math.degrees(angle_x):.1f} | Angle Y: {math.degrees(angle_y):.1f} | z: {z:.2f}")
                    master.mav.landing_target_send(
                        int(time.time() * 1000000), 0,
                        mavutil.mavlink.MAV_FRAME_BODY_NED,
                        angle_x, angle_y, z,
                        0, 0, 0, 0, 0, [1.0,0,0,0], 2, 1
                    )
        time.sleep(0.05)

if __name__ == "__main__":
    main()
