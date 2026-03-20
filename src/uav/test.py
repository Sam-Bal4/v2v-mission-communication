"""
UAV Mission 5 - Vision Guided ArUco Follow + UGV Circles
============================================
Flow:
  Phase 1 — Arm + climb to TARGET_ALT
  Phase 2 — Fly forward ~5 m (timed pitch override)
  UGV     — Sync with rover, send circle command, rover circles for CIRCLE_TIME
  Phase 3 — Scan for ArUco marker, centre on it for CENTER_HOLD_TIME seconds
             Zone box turns GREEN when marker is inside deadband, RED otherwise
             Corrections use real tvec metres from pose estimation
             Cooldown gate prevents spamming the flight controller
  Phase 4 — Final re-centre tightened to LAND_DEADBAND, then LAND mode
  Land    — Wait for autopilot to confirm disarm

Fully self-contained — no external vision module required.
Camera: HD1080 @ 60 fps (highest ZED 2 quality + framerate per spec sheet).
MAVLink: confirmed STABILIZE + RC channel override pattern from Mission4.
"""

from pymavlink import mavutil
import time
import cv2
import cv2.aruco as aruco
import numpy as np
from dataclasses import dataclass
from typing import Optional, Tuple, List
import v2v_bridge   # UGV radio bridge


# =============================================================================
# TUNABLE MISSION CONFIG
# =============================================================================

# --- MAVLink ---
CONNECTION_STRING = "/dev/ttyACM0"
BAUD_RATE         = 57600

# --- UGV radio bridge ---
ESP32_PORT  = "/dev/ttyUSB0"   # ESP32 radio USB port
CIRCLE_TIME = 18.0             # seconds rover runs circles

# --- Calibration / vision ---
CALIBRATION_FILE = "calibration_chessboard.yaml"
MARKER_SIZE      = 0.1          # metres — must match your printed marker

# --- Camera ---
# HD1080 @ 60 fps is the highest quality + framerate available on ZED 2
# (per spec sheet: HD1080 = 3840x1080 side-by-side, supports 60/30/15 fps)
ZED_RESOLUTION  = "HD1080"      # highest quality mode
ZED_FPS         = 60            # highest framerate for HD1080
ZED_EXPOSURE    = 50            # 1-100; lower = sharper fast-moving markers
ZONE_BOX_WIDTH  = 200           # pixels — acceptance box width
ZONE_BOX_HEIGHT = 200           # pixels — acceptance box height

# --- Flight ---
TARGET_ALT     = 1.3            # hover height in metres
THROTTLE_CLIMB = 1650
THROTTLE_HOVER = 1500
ALT_BAND       = 0.1            # ±m band before altitude correction fires
ALT_BOOST      = 100            # PWM delta applied when outside ALT_BAND

# --- Forward flight ---
FORWARD_PITCH_PWM   = 1580      # slight forward pitch
FORWARD_FLIGHT_TIME = 7.0       # seconds (~5 m at 0.7 m/s — tune as needed)

# --- Marker acquisition ---
CONFIRM_NEEDED = 3              # consecutive detections before locking

# --- Centering PID (metre-based, uses tvec from pose estimation) ---
KP_ROLL        = 300            # gain: 0.3 m * 300 = 90 PWM nudge
KP_PITCH       = 300
MAX_NUDGE      = 150            # hard cap on PWM offset from RC_CENTER
METER_DEADBAND = 0.05           # 5 cm — box turns green inside this

# --- Cooldown between correction commands ---
CORRECTION_COOLDOWN = 0.8       # seconds

# --- Hold timer ---
CENTER_HOLD_TIME = 60.0         # seconds to stay centred before land phase
LAND_DEADBAND    = 0.075        # 7.5 cm — tighter gate used just before landing

# --- Loop rate ---
FOLLOW_HZ = 10

# --- Display ---
WINDOW_NAME = "UAV Mission 5 - Vision Guided"

# --- RC neutral ---
RC_CENTER = 1500

# --- Log file ---
LOG_FILE = "mission5.log"


# =============================================================================
# DATA TYPES
# =============================================================================

@dataclass
class MarkerPosition:
    marker_id: int
    x: float        # side offset in metres   (right = +)
    y: float        # forward offset in metres (fwd camera = +)
    z: float        # height in metres
    distance: float # Euclidean distance
    detected: bool = True


# =============================================================================
# CENTER ZONE BOX
# =============================================================================

class CenterZone:
    """
    Rectangular acceptance box drawn at the frame centre.
    GREEN when the marker is inside METER_DEADBAND (truly centred).
    RED  when the marker is visible but outside deadband, or not detected.
    """

    def __init__(self, box_width: int = ZONE_BOX_WIDTH,
                 box_height: int = ZONE_BOX_HEIGHT):
        self.box_width  = box_width
        self.box_height = box_height

    def bounds(self, frame_w: int, frame_h: int) -> Tuple[int, int, int, int]:
        cx, cy = frame_w / 2.0, frame_h / 2.0
        return (
            int(cx - self.box_width  / 2.0),
            int(cy - self.box_height / 2.0),
            int(cx + self.box_width  / 2.0),
            int(cy + self.box_height / 2.0),
        )

    def draw(self, frame: np.ndarray, in_zone: bool) -> np.ndarray:
        """
        Draws the zone box.
        in_zone=True  → GREEN  (marker is within METER_DEADBAND)
        in_zone=False → RED    (marker missing or outside deadband)
        """
        h, w   = frame.shape[:2]
        x0, y0, x1, y1 = self.bounds(w, h)
        color  = (0, 255, 0) if in_zone else (0, 0, 255)
        cv2.rectangle(frame, (x0, y0), (x1, y1), color, 2)
        label  = "CENTRED" if in_zone else "NOT CENTRED"
        cv2.putText(frame, label, (x0, y0 - 8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 2, cv2.LINE_AA)
        return frame


# =============================================================================
# CAMERA INTERFACE
# =============================================================================

class CameraInterface:
    def __init__(self, use_zed: bool = True):
        self.use_zed = use_zed
        self.zed     = None
        self._sl     = None
        self.cap     = None
        print(f"Initializing {'ZED' if use_zed else 'standard'} camera "
              f"({ZED_RESOLUTION} @ {ZED_FPS} fps)...")

        if use_zed:
            self._init_zed()
        else:
            self._init_standard()

        print("Camera initialized.")

    def _init_zed(self):
        import pyzed.sl as sl
        self._sl = sl

        self.zed  = sl.Camera()
        p         = sl.InitParameters()
        res_map   = {
            "HD2K":   sl.RESOLUTION.HD2K,
            "HD1080": sl.RESOLUTION.HD1080,   # ← highest quality
            "HD720":  sl.RESOLUTION.HD720,
            "SVGA":   sl.RESOLUTION.SVGA,
            "VGA":    sl.RESOLUTION.VGA,
        }
        p.camera_resolution = res_map.get(ZED_RESOLUTION, sl.RESOLUTION.HD1080)
        p.camera_fps        = ZED_FPS   # 60 fps at HD1080
        p.depth_mode        = sl.DEPTH_MODE.NONE

        status = self.zed.open(p)
        if status != sl.ERROR_CODE.SUCCESS:
            raise RuntimeError(f"ZED open failed: {status}")

        # exposure must be set AFTER open()
        self.zed.set_camera_settings(sl.VIDEO_SETTINGS.EXPOSURE, ZED_EXPOSURE)
        print(f"ZED ready — {ZED_RESOLUTION} @ {ZED_FPS} fps  exposure={ZED_EXPOSURE}")

    def _init_standard(self):
        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            raise RuntimeError("Standard camera failed to open.")
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH,  1280)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT,  720)

    def get_frame(self) -> Optional[np.ndarray]:
        return self._get_zed_frame() if self.use_zed else self._get_std_frame()

    def _get_zed_frame(self) -> Optional[np.ndarray]:
        sl = self._sl
        if self.zed.grab() != sl.ERROR_CODE.SUCCESS:
            return None
        img = sl.Mat()
        self.zed.retrieve_image(img, sl.VIEW.LEFT)
        return cv2.cvtColor(img.get_data(), cv2.COLOR_BGRA2BGR)

    def _get_std_frame(self) -> Optional[np.ndarray]:
        ret, frame = self.cap.read()
        return frame if ret else None

    def close(self):
        if self.use_zed and self.zed:
            self.zed.close()
        elif self.cap:
            self.cap.release()
        cv2.destroyAllWindows()


# =============================================================================
# ARUCO DETECTOR
# =============================================================================

class ArucoDetector:
    def __init__(self, calibration_file: str, marker_size: float = 0.1,
                 dictionary=aruco.DICT_6X6_1000):
        self.marker_size  = marker_size
        self.aruco_dict   = aruco.getPredefinedDictionary(dictionary)

        fs = cv2.FileStorage(calibration_file, cv2.FILE_STORAGE_READ)
        self.camera_matrix = fs.getNode("K").mat()
        self.dist_coeffs   = fs.getNode("D").mat()
        fs.release()

        if self.camera_matrix is None or self.dist_coeffs is None:
            raise ValueError(f"Invalid calibration file: {calibration_file}")

        print(f"Calibration loaded: {calibration_file}  "
              f"marker={marker_size}m  DICT_6X6_1000")

    def detect(self, frame: np.ndarray) -> List[MarkerPosition]:
        gray    = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = aruco.detectMarkers(gray, self.aruco_dict)
        results = []
        if ids is not None:
            for i, mid in enumerate(ids.flatten()):
                _, tvecs, _ = aruco.estimatePoseSingleMarkers(
                    corners[i], self.marker_size,
                    self.camera_matrix, self.dist_coeffs,
                )
                tv = tvecs[0].flatten()
                results.append(MarkerPosition(
                    marker_id=int(mid),
                    x=float(tv[0]), y=float(tv[1]), z=float(tv[2]),
                    distance=float(np.linalg.norm(tv)),
                ))
        return results

    def draw_detections(self, frame: np.ndarray,
                        positions: List[MarkerPosition]) -> np.ndarray:
        gray    = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = aruco.detectMarkers(gray, self.aruco_dict)
        if ids is not None:
            frame = aruco.drawDetectedMarkers(frame, corners, ids)
            for i, mid in enumerate(ids.flatten()):
                rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(
                    corners[i], self.marker_size,
                    self.camera_matrix, self.dist_coeffs,
                )
                cv2.drawFrameAxes(frame, self.camera_matrix, self.dist_coeffs,
                                  rvecs[0].flatten(), tvecs[0].flatten(),
                                  self.marker_size * 0.5)
                pos = next((p for p in positions if p.marker_id == mid), None)
                if pos:
                    center = np.mean(corners[i][0], axis=0).astype(int)
                    color  = (0, 255, 0) if mid == 0 else (0, 0, 255)
                    cv2.putText(frame,
                                f"ID:{mid} D:{pos.distance:.2f}m",
                                tuple(center),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, color,
                                2, cv2.LINE_AA)
        return frame


# =============================================================================
# UAV VISION  (self-contained)
# =============================================================================

class UAVVision:
    def __init__(self, calibration_file: str = CALIBRATION_FILE,
                 marker_size: float = MARKER_SIZE,
                 use_zed: bool = True,
                 zone_box_width: int  = ZONE_BOX_WIDTH,
                 zone_box_height: int = ZONE_BOX_HEIGHT):
        self.camera      = CameraInterface(use_zed=use_zed)
        self.detector    = ArucoDetector(calibration_file, marker_size)
        self.center_zone = CenterZone(zone_box_width, zone_box_height)

    def process_frame(self, display: bool = False):
        frame = self.camera.get_frame()
        if frame is None:
            return None, None, None, None
        positions = self.detector.detect(frame)
        annotated = self.detector.draw_detections(frame.copy(), positions)
        gray      = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = aruco.detectMarkers(gray, self.detector.aruco_dict)
        return positions, annotated, corners, ids

    def close(self):
        self.camera.close()


# =============================================================================
# LOGGING
# =============================================================================

def log(text: str):
    ts   = time.strftime("%H:%M:%S")
    line = f"[{ts}] {text}"
    print(line)
    with open(LOG_FILE, "a", encoding="utf-8") as f:
        f.write(line + "\n")


# =============================================================================
# MAVLINK HELPERS  (confirmed Mission4 pattern — unchanged)
# =============================================================================

def change_mode(master, mode: str):
    mapping = master.mode_mapping()
    if mode not in mapping:
        log(f"[FC] Unknown mode '{mode}'")
        return
    master.mav.set_mode_send(
        master.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mapping[mode],
    )
    log(f"[FC] Mode: {mode}")
    time.sleep(1)


def arm_drone(master):
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0, 1, 0, 0, 0, 0, 0, 0,
    )
    log("[FC] Arming motors...")
    time.sleep(2)


def set_rc_override(master, roll=RC_CENTER, pitch=RC_CENTER,
                    throttle=THROTTLE_HOVER, yaw=RC_CENTER):
    master.mav.rc_channels_override_send(
        master.target_system, master.target_component,
        roll, pitch, throttle, yaw, 0, 0, 0, 0,
    )


def get_lidar_alt(master, blocking: bool = False) -> float:
    msg = master.recv_match(type="DISTANCE_SENSOR",
                            blocking=blocking, timeout=0.05)
    if msg:
        return msg.current_distance / 100.0
    return 0.0


def throttle_hold(alt: float) -> int:
    if alt < TARGET_ALT - ALT_BAND:
        return THROTTLE_HOVER + ALT_BOOST
    if alt > TARGET_ALT + ALT_BAND:
        return THROTTLE_HOVER - ALT_BOOST
    return THROTTLE_HOVER


# =============================================================================
# CENTERING HELPERS
# =============================================================================

def compute_correction(pos: MarkerPosition) -> Tuple[int, int]:
    ex = 0.0 if abs(pos.x) < METER_DEADBAND else pos.x
    ey = 0.0 if abs(pos.y) < METER_DEADBAND else pos.y
    roll_pwm  = int(RC_CENTER + max(-MAX_NUDGE, min(MAX_NUDGE,  KP_ROLL  * ex)))
    pitch_pwm = int(RC_CENTER + max(-MAX_NUDGE, min(MAX_NUDGE, -KP_PITCH * ey)))
    return roll_pwm, pitch_pwm


def is_centered(pos: MarkerPosition, deadband: float = METER_DEADBAND) -> bool:
    return abs(pos.x) <= deadband and abs(pos.y) <= deadband


# =============================================================================
# DISPLAY HELPER
# =============================================================================

def show(frame, status: str, vision: UAVVision, in_zone: bool):
    """
    Draws the zone box (GREEN = centred, RED = not centred),
    a white crosshair at the frame centre, and a black status bar at the top.

    in_zone must reflect the METER-based deadband check so the colour
    matches the actual flight correction state, not just pixel position.
    """
    if frame is None:
        return
    out = frame.copy()

    # Zone box — colour driven by metre-based deadband (is_centered result)
    vision.center_zone.draw(out, in_zone)

    # White crosshair at exact frame centre
    h, w = out.shape[:2]
    cv2.drawMarker(out, (w // 2, h // 2), (255, 255, 255),
                   cv2.MARKER_CROSS, 20, 1, cv2.LINE_AA)

    # Status bar
    cv2.rectangle(out, (0, 0), (w, 36), (0, 0, 0), -1)
    cv2.putText(out, status, (8, 26),
                cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 255), 2, cv2.LINE_AA)

    cv2.imshow(WINDOW_NAME, out)
    cv2.waitKey(1)


# =============================================================================
# MAIN MISSION
# =============================================================================

def main():
    log("==========================================")
    log("   UAV MISSION 5 - VISION GUIDED + UGV")
    log("   Takeoff -> 5m Forward -> UGV Circles -> Centre ArUco (60s) -> Land")
    log("==========================================")

    # ── Flight controller ────────────────────────────────────────────────────
    log(f"[FC] Connecting: {CONNECTION_STRING}...")
    master = mavutil.mavlink_connection(CONNECTION_STRING, baud=BAUD_RATE)
    master.wait_heartbeat()
    log("[FC] Heartbeat OK.")

    # ── UGV radio bridge ─────────────────────────────────────────────────────
    bridge = v2v_bridge.V2VBridge(ESP32_PORT, name="UAV-Bridge")
    try:
        bridge.connect()
        bridge.send_message("MISSION 5: START")
        log("[Bridge] Radio bridge connected.")
    except Exception as e:
        log(f"[Bridge] Radio bridge failed: {e}")
        return

    # ── Vision ───────────────────────────────────────────────────────────────
    log(f"[Vision] Starting UAVVision — ZED {ZED_RESOLUTION} @ {ZED_FPS} fps...")
    try:
        vision = UAVVision(
            calibration_file=CALIBRATION_FILE,
            marker_size=MARKER_SIZE,
            use_zed=True,
            zone_box_width=ZONE_BOX_WIDTH,
            zone_box_height=ZONE_BOX_HEIGHT,
        )
    except Exception as e:
        log(f"[!] Vision init failed: {e}")
        bridge.stop()
        return

    cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(WINDOW_NAME, 1280, 720)

    loop_dt = 1.0 / FOLLOW_HZ

    try:
        ##################################################################
        # PHASE 1: ARM + CLIMB
        ##################################################################
        log("\n--- PHASE 1: TAKEOFF ---")
        change_mode(master, "STABILIZE")
        arm_drone(master)

        while True:
            alt = get_lidar_alt(master, blocking=True)
            print(f"\r  Alt: {alt:.2f}m / {TARGET_ALT}m", end="", flush=True)

            positions, annotated, _, _ = vision.process_frame(display=False)
            show(annotated, f"TAKEOFF  Alt:{alt:.2f}m  Target:{TARGET_ALT}m",
                 vision, False)   # box always red during takeoff (not centring yet)

            if alt >= TARGET_ALT:
                set_rc_override(master, throttle=THROTTLE_HOVER)
                log(f"\n[FC] Hover altitude reached: {alt:.2f}m")
                break

            set_rc_override(master, throttle=THROTTLE_CLIMB)
            time.sleep(0.1)

        ##################################################################
        # PHASE 2: FLY FORWARD ~5 METRES
        ##################################################################
        log(f"\n--- PHASE 2: FORWARD FLIGHT ({FORWARD_FLIGHT_TIME}s) ---")
        bridge.send_message("PHASE 2: FLYING FORWARD")

        fwd_start = time.time()
        while (time.time() - fwd_start) < FORWARD_FLIGHT_TIME:
            elapsed   = time.time() - fwd_start
            remaining = FORWARD_FLIGHT_TIME - elapsed

            alt = get_lidar_alt(master)
            set_rc_override(master, pitch=FORWARD_PITCH_PWM,
                            throttle=throttle_hold(alt))

            positions, annotated, _, _ = vision.process_frame(display=False)
            show(annotated,
                 f"FORWARD  {elapsed:.1f}s / {FORWARD_FLIGHT_TIME}s  "
                 f"remaining:{remaining:.1f}s  Alt:{alt:.2f}m",
                 vision, False)

            print(f"\r  Flying {elapsed:.1f}s  Alt:{alt:.2f}m",
                  end="", flush=True)
            time.sleep(0.1)

        alt = get_lidar_alt(master)
        set_rc_override(master, throttle=throttle_hold(alt))
        log("\n[FC] Forward flight complete. Hovering.")

        ##################################################################
        # UGV SYNC + CIRCLE COMMAND  (mission4 pattern, unchanged)
        # Drone is now in position — sync with rover then send circle command.
        # Throttle is kept alive every cycle so drone doesn't drop while waiting.
        ##################################################################
        log("\n--- UGV: WAITING FOR ROVER SYNC ---")
        bridge.send_message("UAV IN POSITION: WAITING FOR UGV")

        while True:
            set_rc_override(master, throttle=throttle_hold(get_lidar_alt(master)))
            data = bridge.get_telemetry()
            if data:
                log("[Bridge] UGV ready. Sending circle command.")
                break
            time.sleep(0.2)     # 5 Hz check — keeps RC heartbeat alive

        bridge.send_command(cmdSeq=400, cmd=v2v_bridge.CMD_CIRCLE, estop=0)
        log("[Bridge] Circle command sent. Rover is now circling.")
        bridge.send_message("UGV CIRCLING: STARTING ARUCO PHASE")

        ##################################################################
        # PHASE 3: ACQUIRE MARKER + CENTRE FOR 60 SECONDS
        # Zone box turns GREEN when is_centered() returns True (within 5 cm).
        # Zone box stays RED when searching, confirming, or outside deadband.
        ##################################################################
        log(f"\n--- PHASE 3: ACQUIRE + CENTRE ({CENTER_HOLD_TIME}s) ---")

        target_id            = None
        confirm_count        = 0
        center_timer         = None
        last_correction_time = 0.0

        while True:
            loop_start = time.time()

            alt = get_lidar_alt(master)
            thr = throttle_hold(alt)

            positions, annotated, corners, ids = vision.process_frame(display=False)

            if positions is None:
                set_rc_override(master, throttle=thr)
                time.sleep(0.05)
                continue

            # ── Acquisition ───────────────────────────────────────────────────
            if target_id is None:
                if positions:
                    confirm_count += 1
                    candidate = positions[0].marker_id
                    if confirm_count >= CONFIRM_NEEDED:
                        target_id    = candidate
                        center_timer = time.time()
                        log(f"\n[ArUco] Locked on ID:{target_id}. "
                            f"Centering for {CENTER_HOLD_TIME}s...")
                        bridge.send_message(f"MARKER LOCKED: ID {target_id}")
                    status = (f"CONFIRMING ID:{candidate}  "
                              f"{confirm_count}/{CONFIRM_NEEDED}  Alt:{alt:.2f}m")
                else:
                    confirm_count = 0
                    status = f"SEARCHING for marker...  Alt:{alt:.2f}m"

                set_rc_override(master, throttle=thr)
                show(annotated, status, vision, False)  # RED — not centring yet

            # ── Centring ──────────────────────────────────────────────────────
            else:
                pos = next((p for p in positions
                            if p.marker_id == target_id), None)

                if pos:
                    # is_centered() is the single source of truth for the green box
                    in_zone   = is_centered(pos, METER_DEADBAND)
                    time_held = time.time() - center_timer
                    time_left = CENTER_HOLD_TIME - time_held

                    now = time.time()
                    if not in_zone and (now - last_correction_time) >= CORRECTION_COOLDOWN:
                        roll_pwm, pitch_pwm = compute_correction(pos)
                        set_rc_override(master, roll=roll_pwm,
                                        pitch=pitch_pwm, throttle=thr)
                        log(f"[Centre] x:{pos.x:.3f}m y:{pos.y:.3f}m  "
                            f"roll:{roll_pwm} pitch:{pitch_pwm}  "
                            f"held:{time_held:.1f}s left:{time_left:.1f}s")
                        last_correction_time = now
                    else:
                        set_rc_override(master, throttle=thr)

                    status = (f"CENTRING ID:{target_id}  "
                              f"x:{pos.x:.2f}m y:{pos.y:.2f}m  "
                              f"held:{time_held:.1f}s  left:{time_left:.1f}s  "
                              f"Alt:{alt:.2f}m")

                    # pass in_zone directly — box is GREEN only when truly centred
                    show(annotated, status, vision, in_zone)
                    print(f"\r  {status}", end="", flush=True)

                    if time_held >= CENTER_HOLD_TIME:
                        log(f"\n[ArUco] {CENTER_HOLD_TIME}s centring complete.")
                        bridge.send_message("CENTERING COMPLETE: PREPARING LAND")
                        break

                else:
                    # Marker temporarily out of frame — hold, box stays RED
                    set_rc_override(master, throttle=thr)
                    show(annotated,
                         f"MARKER LOST — holding  ID:{target_id}  Alt:{alt:.2f}m",
                         vision, False)

            sleep_t = loop_dt - (time.time() - loop_start)
            if sleep_t > 0:
                time.sleep(sleep_t)

        ##################################################################
        # PHASE 4: FINAL RE-CENTRE BEFORE LAND
        # Tighter 7.5 cm deadband. Box stays GREEN until we break to land.
        ##################################################################
        log(f"\n--- PHASE 4: FINAL CENTRE (deadband={LAND_DEADBAND*100:.1f}cm) ---")
        bridge.send_message("PHASE 4: FINAL CENTERING")

        last_correction_time = 0.0

        while True:
            loop_start = time.time()

            alt = get_lidar_alt(master)
            thr = throttle_hold(alt)

            positions, annotated, _, _ = vision.process_frame(display=False)

            if positions is not None:
                pos = next((p for p in positions
                            if p.marker_id == target_id), None)
                if pos:
                    centered = is_centered(pos, LAND_DEADBAND)
                    if centered:
                        set_rc_override(master, throttle=thr)
                        show(annotated,
                             f"CENTRED  x:{pos.x:.3f}m y:{pos.y:.3f}m  → LANDING",
                             vision, True)   # GREEN — confirmed centred
                        log("[ArUco] Centred within land gate. Initiating land.")
                        bridge.send_message("CENTERED: LANDING NOW")
                        break
                    else:
                        now = time.time()
                        if (now - last_correction_time) >= CORRECTION_COOLDOWN:
                            roll_pwm, pitch_pwm = compute_correction(pos)
                            set_rc_override(master, roll=roll_pwm,
                                            pitch=pitch_pwm, throttle=thr)
                            log(f"[LandGate] x:{pos.x:.3f}m y:{pos.y:.3f}m  "
                                f"need<{LAND_DEADBAND:.3f}m  "
                                f"roll:{roll_pwm} pitch:{pitch_pwm}")
                            last_correction_time = now
                        else:
                            set_rc_override(master, throttle=thr)

                        show(annotated,
                             f"FINAL CENTRE  x:{pos.x:.3f}m y:{pos.y:.3f}m  "
                             f"need<{LAND_DEADBAND*100:.1f}cm  Alt:{alt:.2f}m",
                             vision, False)  # RED — still correcting
                else:
                    set_rc_override(master, throttle=thr)
                    show(annotated,
                         f"FINAL CENTRE — target lost  Alt:{alt:.2f}m",
                         vision, False)
            else:
                set_rc_override(master, throttle=thr)
                show(None, f"FINAL CENTRE — no frame  Alt:{alt:.2f}m",
                     vision, False)

            sleep_t = loop_dt - (time.time() - loop_start)
            if sleep_t > 0:
                time.sleep(sleep_t)

        ##################################################################
        # LAND  (confirmed mission4 pattern)
        ##################################################################
        log("\n[FC] Switching to LAND mode...")
        change_mode(master, "LAND")
        set_rc_override(master, roll=RC_CENTER, pitch=RC_CENTER,
                        throttle=0, yaw=RC_CENTER)  # release override

        log("[FC] Waiting for touchdown...")
        while True:
            alt = get_lidar_alt(master)
            print(f"\r  Land alt: {alt:.2f}m", end="", flush=True)

            positions, annotated, _, _ = vision.process_frame(display=False)
            show(annotated, f"LANDING  Alt:{alt:.2f}m", vision, False)

            msg = master.recv_match(type="HEARTBEAT", blocking=False)
            if msg and not (msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
                log("\n[FC] Touchdown confirmed. Motors stopped.")
                break
            time.sleep(0.5)

    except KeyboardInterrupt:
        log("\n[!] Emergency land triggered by user.")
        change_mode(master, "LAND")
        set_rc_override(master, roll=RC_CENTER, pitch=RC_CENTER,
                        throttle=0, yaw=RC_CENTER)
        time.sleep(1)

    finally:
        vision.close()
        bridge.stop()
        log("Mission 5 finalized.")


if __name__ == "__main__":
    main()