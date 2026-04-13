"""
Mission 2 – UAV scans field for destination ArUco (IDs 1-4), sends GPS-denied
coordinates to UGV via V2V, then precision-lands on UGV landing pad (ID 0).

Uses ZED SDK point-cloud for real 3-D position. Falls back to pixel angles if
ZED is unavailable (e.g. bench test).
"""
import time
import signal
import math
import cv2
from pymavlink import mavutil

from mission_2030.radio.v2v_bridge import V2VBridge
from mission_2030.radio.message_types import DestinationFound
from mission_2030.common.logging_utils import setup_logger
from mission_2030.common.mavlink_utils import arm_vehicle, wait_disarm, get_lidar_alt, is_vehicle_disarmed

# ── config ─────────────────────────────────────────────────────────────────
ESP32_PORT   = "/dev/ttyUSB0"
DRONE_PORT   = "/dev/ttyACM0"
BAUD_RATE    = 57600
TARGET_ALT   = 1.3
RIDE_TIME_S  = 30
LAND_TIMEOUT = 90
TOTAL_TIMEOUT = 600
SCOUT_SPEED   = 0.15   # m/s forward scout velocity
SCAN_TIMEOUT  = 120    # seconds to scan before abort

ARUCO_DICT        = cv2.aruco.DICT_4X4_50
LANDING_MARKER_ID = 0
DEST_MARKER_IDS   = {1, 2, 3, 4}

HFOV_DEG = 84.0
VFOV_DEG = 54.0

logger = setup_logger("Mission2", "mission2.log")

_abort = False
def _sigint(sig, frame):
    global _abort
    _abort = True
signal.signal(signal.SIGINT,  _sigint)
signal.signal(signal.SIGTERM, _sigint)

# ── helpers ──────────────────────────────────────────────────────────────────
def request_streams(master):
    def _set_rate(msg_id, hz):
        master.mav.command_long_send(
            master.target_system, master.target_component,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
            0, msg_id, int(1e6 / hz), 0, 0, 0, 0, 0)
    _set_rate(mavutil.mavlink.MAVLINK_MSG_ID_DISTANCE_SENSOR,    20)
    _set_rate(mavutil.mavlink.MAVLINK_MSG_ID_HEARTBEAT,           5)

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

def send_velocity_ned(master, vx, vy, vz):
    """Send body-NED velocity in GUIDED mode (vz positive = down)."""
    master.mav.set_position_target_local_ned_send(
        0, master.target_system, master.target_component,
        mavutil.mavlink.MAV_FRAME_BODY_NED,
        0b0000111111000111,   # only velocity bits
        0, 0, 0,
        vx, vy, vz,
        0, 0, 0, 0, 0)

def pixel_to_angles(cx, cy, w, h):
    ax = math.radians((cx - w / 2) / w * HFOV_DEG)
    ay = math.radians((cy - h / 2) / h * VFOV_DEG)
    return ax, ay

# ── main ─────────────────────────────────────────────────────────────────────
def main():
    logger.info("======== MISSION 2 START ========")

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

    # Camera / detector
    aruco_dict  = cv2.aruco.getPredefinedDictionary(ARUCO_DICT)
    detector    = cv2.aruco.ArucoDetector(aruco_dict, cv2.aruco.DetectorParameters())
    cam = cv2.VideoCapture(0)
    if not cam.isOpened():
        logger.error("Camera unavailable. Aborting.")
        bridge.stop()
        return

    # Try ZED SDK for depth
    zed = None
    sl  = None          # will be set if pyzed import succeeds
    zed_img = None
    point_cloud_mat = None
    try:
        import pyzed.sl as sl_module  # type: ignore
        sl  = sl_module
        zed = sl.Camera()
        ip  = sl.InitParameters()
        ip.camera_resolution = sl.RESOLUTION.HD720
        ip.depth_mode        = sl.DEPTH_MODE.PERFORMANCE
        ip.coordinate_units  = sl.UNIT.METER
        if zed.open(ip) == sl.ERROR_CODE.SUCCESS:
            zed_img      = sl.Mat()
            point_cloud_mat = sl.Mat()
            cam.release()
            cam = None
            logger.info("ZED SDK active ✓")
        else:
            zed = None
    except ImportError:
        logger.warning("pyzed not installed – using OpenCV camera fallback.")

    mission_start = time.time()

    def elapsed():
        return time.time() - mission_start

    try:
        # ─── Takeoff ──────────────────────────────────────────────────────
        if not change_mode(master, "GUIDED") or not arm_vehicle(master):
            return

        master.mav.command_long_send(
            master.target_system, master.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0, 0, 0, 0, 0, 0, 0, TARGET_ALT)
        logger.info(f"Climbing to {TARGET_ALT} m...")

        climb_start = time.time()
        while not _abort and time.time() - climb_start < 20:
            alt = get_lidar_alt(master)
            if alt and alt >= TARGET_ALT * 0.90:
                break
            time.sleep(0.2)

        # ─── Scout scan for destination marker ───────────────────────────
        logger.info("Scanning for destination ArUco marker (IDs 1-4)...")
        found_dest      = False
        dest_marker_id  = -1
        dest_x_m = dest_y_m = dest_z_m = 0.0
        scan_start = time.time()

        while not found_dest and not _abort:
            if time.time() - scan_start > SCAN_TIMEOUT:
                logger.error("Destination scan timed out.")
                return
            if elapsed() > TOTAL_TIMEOUT:
                logger.error("Total timeout.")
                return

            # Grab frame
            if zed and sl:
                if zed.grab() != sl.ERROR_CODE.SUCCESS:
                    continue
                zed.retrieve_image(zed_img, sl.VIEW.LEFT)
                zed.retrieve_measure(point_cloud_mat, sl.MEASURE.XYZ)
                frame_bgr = cv2.cvtColor(zed_img.get_data(), cv2.COLOR_BGRA2BGR)
            else:
                ret, frame_bgr = cam.read() if cam else (False, None)
                if not ret:
                    continue

            gray = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY)
            corners, ids, _ = detector.detectMarkers(gray)

            if ids is not None:
                for i, mid in enumerate(ids.flatten()):
                    if int(mid) in DEST_MARKER_IDS:
                        dest_marker_id = int(mid)
                        c  = corners[i][0]
                        cx = int((c[0][0] + c[2][0]) / 2)
                        cy = int((c[0][1] + c[2][1]) / 2)

                        if zed and sl and point_cloud_mat is not None:
                            err, p3d = point_cloud_mat.get_value(cx, cy)
                            x, y, z  = float(p3d[0]), float(p3d[1]), float(p3d[2])
                            if err == sl.ERROR_CODE.SUCCESS and not any(math.isnan(v) for v in [x, y, z]):
                                dest_x_m, dest_y_m, dest_z_m = x, y, z
                                found_dest = True
                                logger.info(f"Dest Marker {dest_marker_id} at X:{x:.2f} Y:{y:.2f} Z:{z:.2f}")
                                break
                        else:
                            # No ZED – coords will be 0; UGV will use them as bearing
                            found_dest = True
                            logger.info(f"Dest Marker {dest_marker_id} detected (no depth).")
                            break

            if not found_dest:
                send_velocity_ned(master, SCOUT_SPEED, 0.0, 0.0)
                time.sleep(0.1)

        if not found_dest:
            return

        # ─── Send destination to UGV (spray 5x for reliability) ──────────
        payload = DestinationFound(
            seq=1, timestamp_ms=int(time.time() * 1000),
            marker_id=dest_marker_id,
            x_m=dest_x_m, y_m=dest_y_m, z_m=dest_z_m,
            yaw_rad=0.0, confidence=0.9)
        for _ in range(5):
            bridge.send_destination(payload)
            bridge.send_uav_heartbeat(1, int(time.time() * 1000), 8, False)
            time.sleep(0.1)
        logger.info("Destination transmitted to UGV.")

        # ─── Precision landing on UGV (marker 0) ─────────────────────────
        if not change_mode(master, "LAND"):
            return
        logger.info("LAND mode active. Streaming LANDING_TARGET...")
        land_start = time.time()
        landed     = False

        while not landed and not _abort:
            if time.time() - land_start > LAND_TIMEOUT:
                logger.warning("Landing timeout – assuming touchdown.")
                landed = True
                break

            # Grab frame
            if zed and sl:
                if zed.grab() != sl.ERROR_CODE.SUCCESS:
                    time.sleep(0.05)
                    continue
                zed.retrieve_image(zed_img, sl.VIEW.LEFT)
                zed.retrieve_measure(point_cloud_mat, sl.MEASURE.XYZ)
                frame_bgr = cv2.cvtColor(zed_img.get_data(), cv2.COLOR_BGRA2BGR)
            else:
                ret, frame_bgr = cam.read() if cam else (False, None)
                if not ret:
                    time.sleep(0.02)
                    continue

            gray = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY)
            corners, ids, _ = detector.detectMarkers(gray)

            if ids is not None and LANDING_MARKER_ID in ids.flatten():
                i  = list(ids.flatten()).index(LANDING_MARKER_ID)
                c  = corners[i][0]
                cx = int((c[0][0] + c[2][0]) / 2)
                cy = int((c[0][1] + c[2][1]) / 2)
                h, w = frame_bgr.shape[:2]

                if zed and sl and point_cloud_mat is not None:
                    err, p3d = point_cloud_mat.get_value(cx, cy)
                    px, py, pz = float(p3d[0]), float(p3d[1]), float(p3d[2])
                    if err == sl.ERROR_CODE.SUCCESS and not any(math.isnan(v) for v in [px, py, pz]):
                        ax = math.atan2(px, pz)
                        ay = math.atan2(py, pz)
                        dist = pz
                    else:
                        ax, ay = pixel_to_angles(cx, cy, w, h)
                        dist = get_lidar_alt(master) or 0.5
                else:
                    ax, ay = pixel_to_angles(cx, cy, w, h)
                    dist = get_lidar_alt(master) or 0.5

                master.mav.landing_target_send(
                    int(time.time() * 1_000_000), 0,
                    mavutil.mavlink.MAV_FRAME_BODY_NED,
                    ax, ay, dist,
                    0.0, 0.0, 0.0, 0.0, 0.0,
                    [1.0, 0.0, 0.0, 0.0], 1, 1)

            if is_vehicle_disarmed(master):
                logger.info("Touchdown confirmed by Cube heartbeat ✓")
                landed = True
                break

            time.sleep(0.05)

        # ─── Ride timer ───────────────────────────────────────────────────
        if landed:
            logger.info(f"Riding UGV {RIDE_TIME_S} s...")
            t0 = time.time()
            while time.time() - t0 < RIDE_TIME_S and not _abort:
                time.sleep(0.5)
            logger.info("MISSION 2 COMPLETE ✓")

    finally:
        change_mode(master, "LAND")
        bridge.stop()
        if cam:
            cam.release()
        if zed:
            zed.close()
        logger.info("Mission 2 shutdown.")


if __name__ == "__main__":
    main()
