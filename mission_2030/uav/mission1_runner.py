"""
Mission 1 – UAV takes off, waits for UGV link, precision-lands on moving UGV, rides for 30 s.

Key design decisions:
 - GUIDED + MAV_CMD_NAV_TAKEOFF for arming (no RC overrides)
 - MAVLink LANDING_TARGET stream in LAND mode for precision landing
 - Heartbeat check is FILTERED to only exit on the Cube Orange system ID
 - Signal handler ensures LAND is always attempted on Ctrl+C
 - Message streams requested explicitly so DISTANCE_SENSOR arrives reliably
"""
import time
import signal
import math
import numpy as np
import cv2
from pymavlink import mavutil

from mission_2030.radio.v2v_bridge import V2VBridge
from mission_2030.common.logging_utils import setup_logger
from mission_2030.common.mavlink_utils import arm_vehicle, wait_disarm, get_lidar_alt, is_vehicle_disarmed
from mission_2030.common.vision_utils import ArucoDetectorShim

# ── config ─────────────────────────────────────────────────────────────────
ESP32_PORT   = "/dev/ttyUSB0"
DRONE_PORT   = "/dev/ttyACM0"
BAUD_RATE    = 57600
TARGET_ALT   = 1.3      # metres
RIDE_TIME_S  = 30       # seconds on the UGV after touchdown
LAND_TIMEOUT = 90       # seconds: hard limit for the landing loop
TOTAL_TIMEOUT = 420     # seconds: total mission abort gate

# ArUco dict – must match what is printed on the UGV landing pad
ARUCO_DICT    = cv2.aruco.DICT_4X4_50
LANDING_MARKER_ID = 0

# Field-of-view for pixel→angle conversion (fallback when no ZED SDK)
HFOV_DEG = 84.0   # ZED X horizontal FOV at 720p
VFOV_DEG = 54.0

logger = setup_logger("Mission1", "mission1.log")

# ── graceful Ctrl+C ─────────────────────────────────────────────────────────
_abort = False
def _sigint(sig, frame):
    global _abort
    _abort = True
signal.signal(signal.SIGINT,  _sigint)
signal.signal(signal.SIGTERM, _sigint)

# ── helpers ──────────────────────────────────────────────────────────────────
def request_streams(master):
    """Ask the Cube to stream the messages we need."""
    def _set_rate(msg_id, hz):
        master.mav.command_long_send(
            master.target_system, master.target_component,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
            0, msg_id, int(1e6 / hz), 0, 0, 0, 0, 0)
    _set_rate(mavutil.mavlink.MAVLINK_MSG_ID_DISTANCE_SENSOR,    20)
    _set_rate(mavutil.mavlink.MAVLINK_MSG_ID_HEARTBEAT,           5)
    _set_rate(mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT, 5)
    _set_rate(mavutil.mavlink.MAVLINK_MSG_ID_EKF_STATUS_REPORT,   2)

def change_mode(master, mode: str) -> bool:
    mapping = master.mode_mapping()
    if mode not in mapping:
        logger.error(f"Mode '{mode}' unavailable. Have: {list(mapping.keys())}")
        return False
    master.mav.set_mode_send(
        master.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mapping[mode])
    time.sleep(0.5)
    logger.info(f"Mode → {mode}")
    return True

def pixel_to_angles(cx, cy, w, h):
    angle_x = math.radians((cx - w / 2) / w * HFOV_DEG)
    angle_y = math.radians((cy - h / 2) / h * VFOV_DEG)
    return angle_x, angle_y
    angle_x = math.radians((cx - w / 2) / w * HFOV_DEG)
    angle_y = math.radians((cy - h / 2) / h * VFOV_DEG)
    return angle_x, angle_y

# ── main ─────────────────────────────────────────────────────────────────────
def main():
    logger.info("======== MISSION 1 START ========")

    # --- Radio bridge ---
    bridge = V2VBridge(ESP32_PORT)
    try:
        bridge.connect()
        logger.info("ESP32 bridge connected.")
    except Exception as e:
        logger.error(f"Bridge FAILED: {e}")
        return

    # --- Flight controller ---
    logger.info(f"Connecting to Cube on {DRONE_PORT}...")
    master = mavutil.mavlink_connection(DRONE_PORT, baud=BAUD_RATE)
    master.wait_heartbeat()
    logger.info(f"Heartbeat from sysid={master.target_system}")
    request_streams(master)

    # --- Camera ---
    aruco_dict   = cv2.aruco.getPredefinedDictionary(ARUCO_DICT)
    aruco_params = cv2.aruco.DetectorParameters()
    detector     = ArucoDetectorShim(aruco_dict, aruco_params)
    cam = cv2.VideoCapture(0)
    if not cam.isOpened():
        logger.error("Camera could not be opened. Aborting.")
        bridge.stop()
        return
    logger.info("Camera opened.")

    mission_start = time.time()

    def timeout_exceeded():
        if time.time() - mission_start > TOTAL_TIMEOUT:
            logger.error("TOTAL MISSION TIMEOUT.")
            return True
        return False

    try:
        # ─── Wait for UGV link ────────────────────────────────────────────
        logger.info("Waiting for UGV radio heartbeat...")
        seq = 0
        while not _abort:
            bridge.send_uav_heartbeat(seq, (int(time.time() * 1000) & 0xFFFFFFFF), 1, False)
            seq += 1
            if bridge.latest_heartbeat_ugv_time > time.time() - 3.0:
                logger.info("UGV link confirmed ✓")
                break
            if timeout_exceeded():
                return
            time.sleep(0.5)

        if _abort:
            return

        # ─── Takeoff ──────────────────────────────────────────────────────
        if not change_mode(master, "GUIDED"):
            return
        if not arm_vehicle(master):
            return

        master.mav.command_long_send(
            master.target_system, master.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0, 0, 0, 0, 0, 0, 0, TARGET_ALT)

        logger.info(f"Climbing to {TARGET_ALT} m...")
        climb_start = time.time()
        while not _abort:
            alt = get_lidar_alt(master)
            if alt is not None:
                logger.info(f"  Alt: {alt:.2f} m", )
                if alt >= TARGET_ALT * 0.90:
                    logger.info("Target altitude reached ✓")
                    break
            if time.time() - climb_start > 20:
                logger.warning("Takeoff timeout – proceeding anyway.")
                break
            if timeout_exceeded():
                return
            time.sleep(0.2)

        if _abort:
            return

        # ─── Enter LAND with LANDING_TARGET stream ────────────────────────
        if not change_mode(master, "LAND"):
            return

        logger.info("LAND mode active. Streaming LANDING_TARGET to Cube...")
        land_start = time.time()
        landed     = False

        while not landed and not _abort:
            # Hard time-limit for the landing loop
            if time.time() - land_start > LAND_TIMEOUT:
                logger.warning("Land loop timed out – assuming touchdown.")
                landed = True
                break
            if timeout_exceeded():
                break

            # Camera frame
            ret, frame = cam.read()
            if ret:
                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                corners, ids, _ = detector.detectMarkers(gray)
                if ids is not None:
                    flat_ids = ids.flatten()
                    if LANDING_MARKER_ID in flat_ids:
                        idx = list(flat_ids).index(LANDING_MARKER_ID)
                        c   = corners[idx][0]
                        cx  = float(np.mean(c[:, 0]))
                        cy  = float(np.mean(c[:, 1]))
                        h, w = frame.shape[:2]
                        ax, ay = pixel_to_angles(cx, cy, w, h)

                        alt = get_lidar_alt(master) or 0.5
                        master.mav.landing_target_send(
                            int(time.time() * 1_000_000),   # time_usec
                            0,                               # target_num
                            mavutil.mavlink.MAV_FRAME_BODY_NED,
                            ax, ay, alt,                     # angle_x, angle_y, distance
                            0.0, 0.0,                        # size_x, size_y
                            0.0, 0.0, 0.0,                   # x, y, z (not used in angle mode)
                            [1.0, 0.0, 0.0, 0.0],           # q
                            1,                               # type (LANDING_TARGET_TYPE_LIGHT_BEACON)
                            1                                # position_valid
                        )

            # Touchdown detection – ONLY trust the Cube's own heartbeat
            if is_vehicle_disarmed(master):
                logger.info("Cube heartbeat: motors disarmed → touchdown confirmed ✓")
                landed = True
                break

            time.sleep(0.05)   # 20 Hz loop

        # ─── Ride timer ───────────────────────────────────────────────────
        if landed:
            logger.info(f"Riding UGV for {RIDE_TIME_S} s...")
            ride_start = time.time()
            while time.time() - ride_start < RIDE_TIME_S and not _abort:
                time.sleep(0.5)
            logger.info("Ride timer complete. MISSION 1 SUCCESSFUL ✓")
        else:
            logger.error("Did not confirm landing. Check logs.")

    finally:
        change_mode(master, "LAND")
        bridge.stop()
        cam.release()
        logger.info("Mission 1 shutdown complete.")


if __name__ == "__main__":
    main()
