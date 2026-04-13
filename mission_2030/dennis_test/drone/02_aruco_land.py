import time
import cv2
from pymavlink import mavutil
import pyzed.sl as sl

DRONE_PORT = "/dev/ttyACM0"
BAUD_RATE = 57600
TARGET_ALT = 1.3

def main():
    print("--- Test 2: Camera + Flight ---")
    
    # Init ZED
    cam = sl.Camera()
    init_params = sl.InitParameters()
    init_params.camera_resolution = sl.RESOLUTION.HD720
    if cam.open(init_params) != sl.ERROR_CODE.SUCCESS:
        print("Camera failed to open.")
        return
    zed_image = sl.Mat()

    # ArUco Dict
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    detector = cv2.aruco.ArucoDetector(aruco_dict, cv2.aruco.DetectorParameters())

    master = mavutil.mavlink_connection(DRONE_PORT, baud=BAUD_RATE)
    master.wait_heartbeat()
    print("Heartbeat OK!")

    mapping = master.mode_mapping()
    master.mav.set_mode_send(master.target_system, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, mapping["GUIDED"])
    time.sleep(1)

    print("Arming & Takeoff...")
    master.mav.command_long_send(master.target_system, master.target_component, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0,0,0,0,0,0)
    time.sleep(2)
    master.mav.command_long_send(master.target_system, master.target_component, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, TARGET_ALT)
    time.sleep(5) # Mock takeoff timer
    print("Scanning ArUco...")

    found = False
    start_t = time.time()
    while time.time() - start_t < 15:
        if cam.grab() == sl.ERROR_CODE.SUCCESS:
            cam.retrieve_image(zed_image, sl.VIEW.LEFT)
            frame_bgra = zed_image.get_data()
            frame_bgr = cv2.cvtColor(frame_bgra, cv2.COLOR_BGRA2BGR)
            gray = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY)
            
            corners, ids, _ = detector.detectMarkers(gray)
            if ids is not None:
                print(f"Detected Marker IDs: {ids.flatten()}")
                found = True
                break
        time.sleep(0.1)

    print("Landing...")
    master.mav.set_mode_send(master.target_system, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, mapping["LAND"])

    landed = False
    while not landed:
        msg = master.recv_match(type='HEARTBEAT', blocking=False)
        if msg and msg.get_srcSystem() == master.target_system:
            if not (msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
                print("Landed and Disarmed successfully.")
                landed = True
                break
        time.sleep(0.1)

    cam.close()

if __name__ == "__main__":
    main()
