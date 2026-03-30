"""
UAV ArUco Precision Mission - Webots Simulation Edition
=======================================================
This is the simulation equivalent of your real `missionaruco.py`.
It runs in your terminal and talks to the Webots drone via TCP (port 5760),
and talks to the UGV ground station via UDP (port 9001).

Run this script AFTER you click 'Play' in Webots!
It will wait for the UGV to come online.
"""

import time
import cv2
import socket
import json
from aruco_tracker import ArucoTracker
from snake_search import SnakeSearch
import sim_v2v_bridge as v2v_bridge

# ============================================================
# CONFIG
# ============================================================
WEBOTS_PORT = 5760
ESP32_PORT = "/dev/ttyUSB0" 
ARUCO_ID = 2

TARGET_ALT = 1.3
FORWARD_FLIGHT_TIME = 2.0
CENTER_HOLD_TIME = 5.0
CONFIRM_NEEDED = 2

METER_DEADBAND_PX = 30
LAND_DEADBAND_PX = 20

CAMERA_INDEX = 0
LOG_FILE = "sim_missionaruco_log.txt"

def log(text):
    ts = time.strftime("%H:%M:%S")
    line = f"[{ts}] {text}"
    print(line)
    with open(LOG_FILE, "a") as f: f.write(line + "\n")


class WebotsDroneLink:
    """TCP link to the Webots SimPixhawk controller"""
    def __init__(self, port=5760):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.connect(('127.0.0.1', port))
        
        # We need to receive data too (lidar)
        self.sock.setblocking(False)

    def send_cmd(self, action, **kwargs):
        cmd = {"action": action}
        cmd.update(kwargs)
        self.sock.sendall((json.dumps(cmd) + "\n").encode('utf-8'))

    def arm(self):
        self.send_cmd("arm", value=True)

    def disarm(self):
        self.send_cmd("arm", value=False)

    def set_rc_override(self, roll=1500, pitch=1500, throttle=1500, yaw=1500):
        self.send_cmd("rc_override", roll=roll, pitch=pitch, throttle=throttle, yaw=yaw)

    def set_velocity(self, vx, vy, vz):
        self.send_cmd("set_velocity", vx=vx, vy=vy, vz=vz)

    def get_lidar_alt(self):
        """Requests the drone's altitude from the simulation's LidarLite sensor proxy"""
        self.send_cmd("get_lidar")
        self.sock.settimeout(1.0)
        try:
            data = self.sock.recv(1024).decode('utf-8').strip()
            if data:
                resp = json.loads(data)
                return resp.get("lidar", 0.0)
        except Exception:
            pass
        return 0.0

    def get_camera_frame(self):
        self.send_cmd("get_camera")
        self.sock.settimeout(0.5)
        try:
            data = b""
            while b"\n" not in data:
                chunk = self.sock.recv(16384)
                if not chunk: break
                data += chunk
            msg = json.loads(data.split(b"\n")[0].decode('utf-8'))
            if "camera_b64" in msg:
                import base64
                import numpy as np
                b64 = msg["camera_b64"]
                img_data = base64.b64decode(b64)
                np_arr = np.frombuffer(img_data, np.uint8)
                return cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        except Exception:
            pass
        return None

def main():
    log("--- ARUCO PRECISION MISSION (SIMULATION) ---")

    try:
        drone = WebotsDroneLink(WEBOTS_PORT)
    except Exception as e:
        log(f"[!] Could not connect to Webots Drone. Ensure Webots is PLAYING! {e}")
        return

    bridge = v2v_bridge.V2VBridge(ESP32_PORT, name="UAV-ArUco")
    
    tracker = ArucoTracker(dictionary=cv2.aruco.DICT_4X4_50, target_id=ARUCO_ID)

    # === TAKEOFF ===
    log("Phase 1: TAKEOFF")
    drone.arm()
    
    for _ in range(30):
        drone.set_rc_override(throttle=1650)
        time.sleep(0.1)

    log(f"Altitude reached.")

    # === FORWARD FLIGHT ===
    log(f"Phase 2: FORWARD FLIGHT ({FORWARD_FLIGHT_TIME}s)")
    bridge.send_message("UAV FLYING FORWARD")
    fwd_start = time.time()
    while (time.time() - fwd_start) < FORWARD_FLIGHT_TIME:
        drone.set_rc_override(pitch=1450, throttle=1500)
        time.sleep(0.1)

    drone.set_rc_override(pitch=1500, throttle=1500)
    log("Forward flight complete.")

    # === UGV SYNC + CIRCLE ===
    log("Syncing with UGV...")
    bridge.send_message("UAV IN POSITION: WAITING FOR UGV")
    
    while True:
        drone.set_rc_override(throttle=1500)
        if bridge.get_telemetry():
            break
        time.sleep(0.2)

    bridge.send_command(cmdSeq=400, cmd=v2v_bridge.CMD_CIRCLE, estop=0)
    log("UGV circling. Starting ArUco phase.")

    # === ARUCO TRACKING ===
    log(f"Phase 3: ACQUIRE + CENTRE ({CENTER_HOLD_TIME}s)")
    target_locked = False
    center_timer = None
    is_home = False
    searcher = SnakeSearch(row_length=8.0, row_spacing=2.5)

    while not is_home:
        frame = drone.get_camera_frame()
        if frame is None:
            drone.set_velocity(0.0, 0.0, 0.0)
            time.sleep(0.02)
            continue

        result = tracker.find_target(frame)

        if result:
            # P-Controller with Inverted Axis correction
            vx = -(result.dy / 240.0) * 2.5
            vy = -(result.dx / 320.0) * 2.5
            
            # Check deadband
            is_centered = abs(result.dx) < 40 and abs(result.dy) < 40
            
            if is_centered:
                if center_timer is None:
                    center_timer = time.time()
                time_held = time.time() - center_timer
                
                status = f"CENTRING ID:{result.marker_id} held:{time_held:.1f}s"
                # KEEP P-CONTROLLER ACTIVE for smooth settling (avoid abrupt stop)
                drone.set_velocity(vx, vy, 0.0) 
                
                if time_held >= CENTER_HOLD_TIME:
                    log("Centering complete! Initiating Land.")
                    drone.send_cmd("land")
                    is_home = True
                    break
            else:
                center_timer = None
                status = f"TRACKING ID:{result.marker_id}"
                drone.set_velocity(vx, vy, 0.0)

            # Overlay
            cv2.drawMarker(frame, (int(result.cx), int(result.cy)), (0, 255, 0), cv2.MARKER_CROSS, 20, 2)
            cv2.putText(frame, status, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        else:
            center_timer = None
            print("\r  SEARCHING (Snake Search)...", end="")
            svx, svy, _ = searcher.get_velocity()
            drone.set_velocity(svx, svy, 0.0)

        cv2.imshow("Webots Drone Camera", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'): break
        time.sleep(0.02)

    # === LAND ===
    log("LANDING SEQUENCE...")
    while True:
        frame = drone.get_camera_frame()
        if frame is not None:
            cv2.putText(frame, "LANDING: Touchdown Check...", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)
            cv2.imshow("Webots Drone Camera", frame)
            cv2.waitKey(1)
            
        alt = drone.get_lidar_alt()
        if alt < 0.1: break
        time.sleep(0.2)
    
    log("Touchdown confirmed! Motors stopped.")
    drone.disarm()
    bridge.stop()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
