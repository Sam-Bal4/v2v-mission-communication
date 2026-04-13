import time
from pymavlink import mavutil

class LandingTargetEncoder:
    """Sends MAVLink LANDING_TARGET messages to ArduPilot for precision landing."""

    def __init__(self, master):
        self.master = master

    def send(self, angle_x: float, angle_y: float, distance_m: float):
        """
        angle_x: lateral angle offset in radians (positive = target right of centre)
        angle_y: longitudinal angle offset in radians (positive = target below centre)
        distance_m: ZED depth (Z axis) or lidar range in metres
        """
        self.master.mav.landing_target_send(
            int(time.time() * 1_000_000),   # time_usec
            0,                               # target_num
            mavutil.mavlink.MAV_FRAME_BODY_NED,
            angle_x,                         # angle_x (rad)
            angle_y,                         # angle_y (rad)
            distance_m,                      # distance (m)
            0.0, 0.0,                        # size_x, size_y (unused)
            0.0, 0.0, 0.0,                   # x, y, z position (unused in angle mode)
            [1.0, 0.0, 0.0, 0.0],           # q quaternion (unused)
            1,                               # type: LANDING_TARGET_TYPE_LIGHT_BEACON
            1                                # position_valid
        )
