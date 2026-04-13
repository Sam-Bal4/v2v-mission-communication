import time
import math
import cv2
import pyzed.sl as sl
from pymavlink import mavutil
import sys
import os

sys.path.append(os.path.abspath("../../"))
from mission_2030.radio.v2v_bridge import V2VBridge
from mission_2030.radio.message_types import DestinationFound

def main():
    print("--- Test 4: UAV Localize ArUco -> Send to UGV ---")
    bridge = V2VBridge("/dev/ttyUSB0")
    bridge.connect()

    cam = sl.Camera()
    params = sl.InitParameters(camera_resolution=sl.RESOLUTION.HD720)
    cam.open(params)
    zed_img = sl.Mat()
    point_cloud = sl.Mat()
    
    detector = cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50))

    master = mavutil.mavlink_connection("/dev/ttyACM0", baud=57600)
    master.wait_heartbeat()
    
    master.mav.set_mode_send(master.target_system, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, master.mode_mapping()["GUIDED"])
    time.sleep(1)
    master.mav.command_long_send(master.target_system, master.target_component, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0,0,0,0,0,0)
    time.sleep(2)
    master.mav.command_long_send(master.target_system, master.target_component, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, 1.3)
    time.sleep(5)

    print("Scanning ArUco to send to UGV...")
    start_t = time.time()
    dest_msg = None

    while time.time() - start_t < 20:
        if cam.grab() == sl.ERROR_CODE.SUCCESS:
            cam.retrieve_image(zed_img, sl.VIEW.LEFT)
            cam.retrieve_measure(point_cloud, sl.MEASURE.XYZ)
            gray = cv2.cvtColor(cv2.cvtColor(zed_img.get_data(), cv2.COLOR_BGRA2BGR), cv2.COLOR_BGR2GRAY)
            
            corners, ids, _ = detector.detectMarkers(gray)
            if ids is not None:
                idx = 0
                c = corners[idx][0]
                cx, cy = int((c[0][0] + c[2][0])/2), int((c[0][1] + c[2][1])/2)
                err, p3d = point_cloud.get_value(cx, cy)
                if err == sl.ERROR_CODE.SUCCESS and not math.isnan(p3d[0]):
                    x, y, z = float(p3d[0]), float(p3d[1]), float(p3d[2])
                    print(f"Marker found at X:{x:.2f} Y:{y:.2f} Z:{z:.2f}")
                    dest_msg = DestinationFound(seq=1, timestamp_ms=int(time.time()*1000), marker_id=int(ids[idx][0]),
                                              x_m=x, y_m=y, z_m=z, yaw_rad=0.0, confidence=1.0)
                    break
        time.sleep(0.1)

    if dest_msg:
        print("Transmitting Destination vector to UGV...")
        for _ in range(5):
            bridge.send_destination(dest_msg)
            time.sleep(0.1)
    
    print("Pretending to wait for UGV to drive there (15 seconds)")
    time.sleep(15)

    print("Landing...")
    master.mav.set_mode_send(master.target_system, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, master.mode_mapping()["LAND"])
    while True:
        msg = master.recv_match(type='HEARTBEAT', blocking=False)
        if msg and msg.get_srcSystem() == master.target_system and not (msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
            break
        time.sleep(0.1)
        
    print("Done.")
    cam.close()
    bridge.stop()

if __name__ == "__main__":
    main()
