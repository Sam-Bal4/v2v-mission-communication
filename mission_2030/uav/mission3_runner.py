"""
Mission 3 – Same as Mission 2 plus UAV pauses descent when UGV is avoiding obstacle.

When bridge.latest_telemetry.mission_phase == UgvState.AVOID_OBSTACLE, the UAV
re-enters GUIDED and climbs slightly. Once the UGV clears, it switches back to LAND.
"""
import time
import signal
import math
import numpy as np
import cv2
from pymavlink import mavutil

from mission_2030.radio.v2v_bridge import V2VBridge
from mission_2030.radio.message_types import DestinationFound
from mission_2030.common.logging_utils import setup_logger
from mission_2030.ugv.state_machine import UgvState

# ── config ─────────────────────────────────────────────────────────────────
ESP32_PORT    = "/dev/ttyUSB0"
DRONE_PORT    = "/dev/ttyACM0"
BAUD_RATE     = 57600
TARGET_ALT    = 1.3
RIDE_TIME_S   = 30
LAND_TIMEOUT  = 120
TOTAL_TIMEOUT = 600
SCOUT_SPEED   = 0.15
SCAN_TIMEOUT  = 120
AVOID_CLIMB   = -0.3   # m/s upward (negative Z = up in NED)

ARUCO_DICT        = cv2.aruco.DICT_4X4_50
LANDING_MARKER_ID = 0
DEST_MARKER_IDS   = {1, 2, 3, 4}
HFOV_DEG = 84.0
VFOV_DEG = 54.0

logger = setup_logger("Mission3", "mission3.log")

_abort = False
def _sigint(sig, frame):
    global _abort
    _abort = True
signal.signal(signal.SIGINT,  _sigint)
signal.signal(signal.SIGTERM, _sigint)

# ── helpers (identical to M2) ─────────────────────────────────────────────────
def request_streams(master):
    def _r(msg_id, hz):
        master.mav.command_long_send(
            master.target_system, master.target_component,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
            0, msg_id, int(1e6 / hz), 0, 0, 0, 0, 0)
    _r(mavutil.mavlink.MAVLINK_MSG_ID_DISTANCE_SENSOR,    20)
    _r(mavutil.mavlink.MAVLINK_MSG_ID_HEARTBEAT,           5)

def change_mode(master, mode: str) -> bool:
    mapping = master.mode_mapping()
    if mode not in mapping:
        logger.error(f"Mode '{mode}' not found.")
        return False
    master.mav.set_mode_send(
        master.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mapping[mode])
    time.sleep(0.5)
    logger.info(f"Mode → {mode}")
    return True

def arm(master) -> bool:
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0, 1, 0, 0, 0, 0, 0, 0)
    deadline = time.time() + 10
    while time.time() < deadline:
        hb = master.recv_match(type='HEARTBEAT', blocking=True, timeout=1.0)
        if hb and hb.get_srcSystem() == master.target_system:
            if hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED:
                logger.info("Armed ✓")
                return True
    return False

def get_lidar_alt(master) -> float | None:
    alt = None
    while True:
        msg = master.recv_match(type='DISTANCE_SENSOR', blocking=False)
        if msg is None:
            break
        if msg.current_distance > 0:
            alt = msg.current_distance / 100.0
    return alt

def is_disarmed(master) -> bool:
    msg = master.recv_match(type='HEARTBEAT', blocking=False)
    if msg and msg.get_srcSystem() == master.target_system:
        return not bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
    return False

def send_velocity_ned(master, vx, vy, vz):
    master.mav.set_position_target_local_ned_send(
        0, master.target_system, master.target_component,
        mavutil.mavlink.MAV_FRAME_BODY_NED, 0b0000111111000111,
        0, 0, 0, vx, vy, vz, 0, 0, 0, 0, 0)

def pixel_to_angles(cx, cy, w, h):
    ax = math.radians((cx - w / 2) / w * HFOV_DEG)
    ay = math.radians((cy - h / 2) / h * VFOV_DEG)
    return ax, ay

def ugv_is_avoiding(bridge: V2VBridge) -> bool:
    """Returns True if latest telemetry says UGV is dodging an obstacle."""
    telem = bridge.latest_telemetry
    if telem is None:
        return False
    return telem.mission_phase == UgvState.AVOID_OBSTACLE.value

# ── main ─────────────────────────────────────────────────────────────────────
def main():
    logger.info("======== MISSION 3 START ========")

    bridge = V2VBridge(ESP32_PORT)
    try:
        bridge.connect()
        logger.info("ESP32 bridge connected.")
    except Exception as e:
        logger.error(f"Bridge FAILED: {e}")
        return

    master = mavutil.mavlink_connection(DRONE_PORT, baud=BAUD_RATE)
    master.wait_heartbeat()
    logger.info(f"Heartbeat sysid={master.target_system}")
    request_streams(master)

    aruco_dict = cv2.aruco.getPredefinedDictionary(ARUCO_DICT)
    detector   = cv2.aruco.ArucoDetector(aruco_dict, cv2.aruco.DetectorParameters())
    cam = cv2.VideoCapture(0)
    if not cam.isOpened():
        logger.error("Camera unavailable.")
        bridge.stop()
        return

    zed = None; zed_img = None; point_cloud_mat = None; sl = None
    try:
        import pyzed.sl as sl_module
        sl = sl_module
        zed = sl.Camera()
        ip  = sl.InitParameters()
        ip.camera_resolution = sl.RESOLUTION.HD720
        ip.depth_mode        = sl.DEPTH_MODE.PERFORMANCE
        ip.coordinate_units  = sl.UNIT.METER
        if zed.open(ip) == sl.ERROR_CODE.SUCCESS:
            zed_img         = sl.Mat()
            point_cloud_mat = sl.Mat()
            cam.release(); cam = None
            logger.info("ZED SDK active ✓")
        else:
            zed = None
    except ImportError:
        logger.warning("pyzed not installed – OpenCV fallback.")

    mission_start = time.time()

    def grab_frame():
        if zed and sl:
            if zed.grab() != sl.ERROR_CODE.SUCCESS:
                return None, None
            zed.retrieve_image(zed_img, sl.VIEW.LEFT)
            zed.retrieve_measure(point_cloud_mat, sl.MEASURE.XYZ)
            return cv2.cvtColor(zed_img.get_data(), cv2.COLOR_BGRA2BGR), point_cloud_mat
        else:
            ret, f = cam.read() if cam else (False, None)
            return (f, None) if ret else (None, None)

    def get_angles(cx, cy, w, h, pc):
        if zed and sl and pc is not None:
            err, p3d = pc.get_value(cx, cy)
            x, y, z = float(p3d[0]), float(p3d[1]), float(p3d[2])
            if err == sl.ERROR_CODE.SUCCESS and not any(math.isnan(v) for v in [x, y, z]):
                return math.atan2(x, z), math.atan2(y, z), z
        alt = get_lidar_alt(master) or 0.5
        return pixel_to_angles(cx, cy, w, h) + (alt,)

    try:
        # ─── Takeoff ──────────────────────────────────────────
        if not change_mode(master, "GUIDED") or not arm(master):
            return

        master.mav.command_long_send(
            master.target_system, master.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0, 0, 0, 0, 0, 0, 0, TARGET_ALT)
        logger.info(f"Climbing to {TARGET_ALT} m...")
        t0 = time.time()
        while not _abort and time.time() - t0 < 20:
            alt = get_lidar_alt(master)
            if alt and alt >= TARGET_ALT * 0.90:
                break
            time.sleep(0.2)

        # ─── Destination scan ─────────────────────────────────
        logger.info("Scanning for destination ArUco marker...")
        found = False
        dest_id = -1
        dx = dy = dz = 0.0
        scan_start = time.time()

        while not found and not _abort:
            if time.time() - scan_start > SCAN_TIMEOUT:
                logger.error("Scan timeout.")
                return
            frame, pc = grab_frame()
            if frame is None:
                time.sleep(0.05)
                continue
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            corners, ids, _ = detector.detectMarkers(gray)
            if ids is not None:
                for i, mid in enumerate(ids.flatten()):
                    if int(mid) in DEST_MARKER_IDS:
                        c  = corners[i][0]
                        cx = int((c[0][0] + c[2][0]) / 2)
                        cy = int((c[0][1] + c[2][1]) / 2)
                        ax, ay, dist = get_angles(cx, cy, *frame.shape[:2][::-1], pc)
                        dest_id = int(mid)
                        if zed and pc is not None:
                            dx, dy, dz = dist * math.tan(ax), dist * math.tan(ay), dist
                        found = True
                        logger.info(f"Marker {dest_id} found.")
                        break
            if not found:
                send_velocity_ned(master, SCOUT_SPEED, 0.0, 0.0)
                time.sleep(0.1)

        # ─── Send destination ─────────────────────────────────
        payload = DestinationFound(
            seq=1, timestamp_ms=int(time.time() * 1000), marker_id=dest_id,
            x_m=dx, y_m=dy, z_m=dz, yaw_rad=0.0, confidence=0.9)
        for _ in range(5):
            bridge.send_destination(payload)
            time.sleep(0.1)
        logger.info("Destination sent to UGV.")

        # ─── Land with obstacle-avoidance relay ───────────────
        change_mode(master, "LAND")
        in_land_mode = True
        land_start   = time.time()
        landed       = False

        while not landed and not _abort:
            if time.time() - land_start > LAND_TIMEOUT:
                logger.warning("Landing timeout.")
                landed = True
                break

            # Check UGV avoidance state
            avoiding = ugv_is_avoiding(bridge)
            if avoiding and in_land_mode:
                logger.info("UGV avoiding obstacle – pausing descent.")
                change_mode(master, "GUIDED")
                in_land_mode = False
            elif not avoiding and not in_land_mode:
                logger.info("UGV clear – resuming LAND.")
                change_mode(master, "LAND")
                in_land_mode = True

            # Hover correction while waiting for UGV
            if not in_land_mode:
                send_velocity_ned(master, 0.0, 0.0, AVOID_CLIMB)

            # LANDING_TARGET stream
            frame, pc = grab_frame()
            if frame is not None:
                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                corners, ids, _ = detector.detectMarkers(gray)
                if ids is not None and LANDING_MARKER_ID in ids.flatten():
                    i  = list(ids.flatten()).index(LANDING_MARKER_ID)
                    c  = corners[i][0]
                    cx = int((c[0][0] + c[2][0]) / 2)
                    cy = int((c[0][1] + c[2][1]) / 2)
                    h, w = frame.shape[:2]
                    ax, ay, dist = get_angles(cx, cy, w, h, pc)
                    master.mav.landing_target_send(
                        int(time.time() * 1_000_000), 0,
                        mavutil.mavlink.MAV_FRAME_BODY_NED,
                        ax, ay, dist,
                        0.0, 0.0, 0.0, 0.0, 0.0,
                        [1.0, 0.0, 0.0, 0.0], 1, 1)

            if is_disarmed(master):
                logger.info("Touchdown confirmed ✓")
                landed = True
                break

            time.sleep(0.05)

        # ─── Ride timer ───────────────────────────────────────
        if landed:
            logger.info(f"Riding UGV {RIDE_TIME_S} s...")
            t0 = time.time()
            while time.time() - t0 < RIDE_TIME_S and not _abort:
                time.sleep(0.5)
            logger.info("MISSION 3 COMPLETE ✓")

    finally:
        change_mode(master, "LAND")
        bridge.stop()
        if cam:
            cam.release()
        if zed:
            zed.close()
        logger.info("Mission 3 shutdown.")


if __name__ == "__main__":
    main()
