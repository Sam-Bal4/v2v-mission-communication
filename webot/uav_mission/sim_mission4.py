"""
UAV Mission 4 - Webots Simulation Edition (Elite v3.0)
======================================================
Unified high-fidelity mission script for ArUco search and rescue.
Features: 640x480 Pro visuals, Sticky-Lock tracking, and Robust Touchdown.
"""

import time
import cv2
import socket
import json
import base64
import numpy as np
from aruco_tracker import ArucoTracker
import sim_v2v_bridge as v2v_bridge
from snake_search import SnakeSearch

def clamp(value, low, high):
    return max(low, min(high, value))

# ============================================================
# CONFIG
# ============================================================
WEBOTS_PORT = 5760
ESP32_PORT = "/dev/ttyUSB0"
ARUCO_ID = 2                    
TARGET_ALT = 1.3                
MISSION_TIMEOUT = 30.0
CENTER_HOLD_TIME = 5.0
LOG_FILE = "sim_mission4_log.txt"

def log_event(text):
    timestamp = time.strftime("%H:%M:%S")
    line = f"[{timestamp}] {text}"
    print(line)
    with open(LOG_FILE, "a") as f: f.write(line + "\n")

def perform_landing(drone, tracker):
    """Elite managed landing sequence with active centering and robust touchdown."""
    log_event("DESCENDING onto UGV...")
    drone.send_cmd("land") # Authoritative descent (0.5m/s)
    
    land_start = time.time()
    on_ground_frames = 0
    while time.time() - land_start < 40:
        alt = drone.get_lidar_alt()
        frame = drone.get_camera_frame()
        if frame is not None:
            res = tracker.find_target(frame)
            if res:
                # Active centering during descent (640x480 coords)
                vx = clamp(-(res.dy / 240.0) * 0.8, -0.4, 0.4)
                vy = clamp(-(res.dx / 320.0) * 0.8, -0.4, 0.4)
                drone.set_velocity(vx, vy, 0.0)
                
                h, w = frame.shape[:2]
                cv2.drawMarker(frame, (w//2, h//2), (255, 255, 255), cv2.MARKER_CROSS, 20, 1)
                cv2.circle(frame, (int(res.cx), int(res.cy)), 5, (0, 255, 0), -1)
                cv2.putText(frame, "LANDING: Active Centering", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            
            # Mirror to Webots HUD (Full Res 640x480)
            frame_hud = frame.copy()
            cv2.putText(frame_hud, f"ALT: {alt:.2f}m", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            _, buf = cv2.imencode('.jpg', frame_hud, [cv2.IMWRITE_JPEG_QUALITY, 75])
            b64 = base64.b64encode(buf).decode('utf-8')
            drone.send_cmd("hud", b64=b64)
            
            # Show Locally (Large)
            cv2.imshow("Elite Drone Preview (640x480)", frame)
            cv2.waitKey(1)
            print(f"\r  [ULTRA-LAND] DESCENDING: {alt:.2f}m... frames_ground:{on_ground_frames}/3", end="", flush=True)

        if 0.01 < alt < 0.12: # Confirmed touchdown range
            on_ground_frames += 1
            if on_ground_frames >= 3: # Need 3 solid frames to ignore glitches
                log_event("\nMISSION SUCCESSFUL - TOUCHDOWN.")
                break
        else:
            on_ground_frames = 0 # Reset on bounce/glitch

        time.sleep(0.02)

# ============================================================
# WEBOTS TCP DRONE LINK
# ============================================================
class WebotsDroneLink:
    def __init__(self, port=5760):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.settimeout(5.0)
        self.sock.connect(('127.0.0.1', port))
        print(f"[Link] Connected to Webots Drone on port {port}")

    def send_cmd(self, action, **kwargs):
        cmd = {"action": action}
        cmd.update(kwargs)
        try:
            self.sock.sendall((json.dumps(cmd) + "\n").encode('utf-8'))
            self.sock.settimeout(5.0)  # INCREASED TIMEOUT to 5.0s to prevent Simulation lag crashes
            data = b""
            while b"\n" not in data:
                chunk = self.sock.recv(65536) 
                if not chunk: break
                data += chunk
            return json.loads(data.split(b"\n")[0].decode('utf-8'))
        except Exception as e:
            return {"error": str(e)}

    def get_lidar_alt(self):
        resp = self.send_cmd("get_lidar")
        return resp.get("lidar", 0.0)

    def arm(self):
        self.send_cmd("arm", value=True)
        time.sleep(1)

    def disarm(self):
        self.send_cmd("arm", value=False)
        time.sleep(1)

    def set_rc_override(self, throttle=1500):
        self.send_cmd("rc_override", throttle=throttle)

    def set_velocity(self, vx=0.0, vy=0.0, vz=0.0):
        self.send_cmd("set_velocity", vx=vx, vy=vy, vz=vz)

    def get_camera_frame(self):
        resp = self.send_cmd("get_camera")
        if "image" in resp and resp["image"]:
            try:
                img_data = base64.b64decode(resp["image"])
                img_array = np.frombuffer(img_data, dtype=np.uint8)
                return cv2.imdecode(img_array, cv2.IMREAD_COLOR)
            except Exception: pass
        return None

# ============================================================
# MAIN MISSION
# ============================================================
def main():
    log_event("--- SIMULATION MISSION 4 STARTING ---")
    drone = WebotsDroneLink(WEBOTS_PORT)
    log_event("Simulation Hardware Linked. Drone is ready.")

    tracker = ArucoTracker(dictionary=cv2.aruco.DICT_6X6_1000, target_id=ARUCO_ID)
    log_event("Connecting to V2V Bridge...")
    bridge = v2v_bridge.V2VBridge(ESP32_PORT, name="UAV-Bridge")
    bridge.send_message("UAV SIMULATION MISSION 4 LIVE")

    # === LAUNCH ===
    log_event("Initiating Launch...")
    drone.arm()
    log_event(f"Climbing to {TARGET_ALT}m (Closed-Loop)...")
    drone.send_cmd("set_target_alt", value=TARGET_ALT)
    
    # Wait until altitude reached (UI polling synced to Simulation)
    climb_start = time.time()
    while time.time() - climb_start < 10:
        alt = drone.get_lidar_alt()
        print(f"\r  [Climb] Alt: {alt:.2f}m / {TARGET_ALT}m", end="", flush=True)
        if alt >= (TARGET_ALT - 0.1):
            log_event(f"\nTarget Altitude {TARGET_ALT}m Reached.")
            break
        # Polling continuously syncs the script to the drone's actual climb rate

    # Hover check
    time.sleep(1)

    # === TRACKING LOOP ===
    log_event("Tracking UGV ArUco Marker...")
    launch_t = time.time()
    center_timer = None
    ugv_stopped = False
    searcher = SnakeSearch(row_length=8.0, row_spacing=2.5)
    seq = 401
    
    while True:
        if (time.time() - launch_t) > MISSION_TIMEOUT:
            log_event(f"Timeout reached ({MISSION_TIMEOUT}s). Forcing landing.")
            perform_landing(drone, tracker)
            break

        frame = drone.get_camera_frame()
        result = tracker.find_target(frame) if frame is not None else None
        
        if result is not None:
            # Sticky Lock Logic
            searcher.reset()
            is_centered = abs(result.dx) < 35 and abs(result.dy) < 35
            
            if is_centered:
                if center_timer is None: center_timer = time.time()
                time_held = time.time() - center_timer
                status = f"STICKY-LOCK ID:{result.marker_id}  held:{time_held:.1f}s"
            else:
                center_timer = None
                status = f"TRACKING ID:{result.marker_id} (NOT CENTERED)"
            
            if not ugv_stopped:
                log_event("[V2V] Marker spotted! HARD BRAKING UGV.")
                drone.send_cmd("brake") 
                bridge.send_command(cmdSeq=seq, cmd=v2v_bridge.CMD_STOP, payload=0)
                ugv_stopped = True
                seq += 1
                time.sleep(1.0)
                continue

            vx = clamp(-(result.dy / 240.0) * 0.8, -0.4, 0.4)
            vy = clamp(-(result.dx / 320.0) * 0.8, -0.4, 0.4)
            drone.set_velocity(vx, vy, 0.0) 
            
            if center_timer and (time.time() - center_timer) >= CENTER_HOLD_TIME:
                log_event(f"\n[ArUco] {CENTER_HOLD_TIME}s Sticky Lock. LANDING.")
                perform_landing(drone, tracker)
                break
            
            # Draw HUD
            h, w = frame.shape[:2]
            cv2.drawMarker(frame, (w//2, h//2), (255, 255, 255), cv2.MARKER_CROSS, 20, 1)
            cv2.circle(frame, (int(result.cx), int(result.cy)), 5, (0, 255, 0), -1)
            cv2.rectangle(frame, (w//2-35, h//2-35), (w//2+35, h//2+35), (0, 255, 0) if is_centered else (0,0,255), 2)
            cv2.putText(frame, status, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            print(f"\r  {status}", end="", flush=True)

        else:
            # Search mode
            center_timer = None
            print("\r  SEARCHING (Snake Search)...", end="", flush=True)
            searcher.update()
            svx, svy, _ = searcher.get_velocity()
            drone.set_velocity(svx * 0.5, svy * 0.5, 0.0) 
            if frame is not None:
                cv2.putText(frame, "SEARCHING (Snake Search)", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

        # Mirror Preview to Webots HUD and Python GUI
        if frame is not None:
            cv2.imshow("Elite Drone Preview (640x480)", frame)
            cv2.waitKey(1)
            # Send HUD to Webots
            _, buf = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 55])
            b64 = base64.b64encode(buf).decode('utf-8')
            drone.send_cmd("hud", b64=b64)
            
        time.sleep(0.01)

    log_event("MISSION 4 COMPLETE.")
    drone.disarm()
    try: bridge.stop()
    except: pass
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
