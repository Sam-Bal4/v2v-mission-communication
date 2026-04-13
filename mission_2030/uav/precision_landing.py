import time
import math
from pymavlink import mavutil
from mission_2030.common.logging_utils import setup_logger

logger = setup_logger("PrecisionLanding")

class PrecisionLanding:
    def __init__(self, master):
        self.master = master

    def send_target_angles(self, offset_x_rad: float, offset_y_rad: float, distance_m: float):
        """
        Feeds the MAVLink LANDING_TARGET precision landing stream.
        Expects angle offsets in body frame NED.
        """
        self.master.mav.landing_target_send(
            int(time.time() * 1000000), 
            0, # default target num
            mavutil.mavlink.MAV_FRAME_BODY_NED,
            offset_x_rad, 
            offset_y_rad, 
            distance_m, 
            0, 0, 0, 0, 0, 
            [1.0,0,0,0], 2, 1
        )

    def is_landed(self) -> bool:
        """
        Checks if the payload has safely touched down based on the armed status logic.
        """
        msg = self.master.recv_match(type='HEARTBEAT', blocking=False)
        if msg and not (msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
            return True
        return False
