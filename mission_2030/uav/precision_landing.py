import time
from pymavlink import mavutil
from mission_2030.common.logging_utils import setup_logger

logger = setup_logger("PrecisionLanding")

class PrecisionLanding:
    def __init__(self, master):
        self.master = master
        self._target_sysid = master.target_system

    def send_target_angles(self, offset_x_rad: float, offset_y_rad: float, distance_m: float):
        """
        Feeds the MAVLink LANDING_TARGET precision landing stream to ArduPilot.
        Expects angles in radians in MAV_FRAME_BODY_NED.
        offset_x_rad: positive = target is to the right
        offset_y_rad: positive = target is below center (forward on ground)
        distance_m:   true range from ZED depth or lidar
        """
        self.master.mav.landing_target_send(
            int(time.time() * 1_000_000),   # time_usec
            0,                               # target_num
            mavutil.mavlink.MAV_FRAME_BODY_NED,
            offset_x_rad,
            offset_y_rad,
            distance_m,
            0.0, 0.0,                        # size_x, size_y (unused)
            0.0, 0.0, 0.0,                   # x, y, z position (unused in angle mode)
            [1.0, 0.0, 0.0, 0.0],           # q quaternion (unused)
            1,                               # type: LANDING_TARGET_TYPE_LIGHT_BEACON
            1                                # position_valid
        )

    def is_landed(self) -> bool:
        """
        Returns True only when the Cube Orange itself confirms motors are disarmed via heartbeat.
        Filters by target_system to avoid false triggers from companion computer heartbeats.
        """
        msg = self.master.recv_match(type='HEARTBEAT', blocking=False)
        if msg and msg.get_srcSystem() == self._target_sysid:
            return not bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
        return False
