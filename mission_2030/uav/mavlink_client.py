from pymavlink import mavutil
import time

class MavlinkClient:
    def __init__(self, port: str, baud: int):
        self.port = port
        self.baud = baud
        self.master = None

    def connect(self):
        self.master = mavutil.mavlink_connection(self.port, baud=self.baud)
        self.master.wait_heartbeat()
        
    def set_mode(self, mode: str):
        mapping = self.master.mode_mapping()
        if mode not in mapping: return False
        mode_id = mapping[mode]
        self.master.mav.set_mode_send(
            self.master.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mode_id)
        return True

    def arm(self):
        self.master.mav.command_long_send(
            self.master.target_system, self.master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0, 1, 0, 0, 0, 0, 0, 0)
            
    def takeoff(self, altitude: float):
        self.master.mav.command_long_send(
            self.master.target_system, self.master.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0, 0, 0, 0, 0, 0, 0, altitude)

    def send_landing_target(self, timestamp_ms: int, x_rad: float, y_rad: float, distance_m: float, x_m: float, y_m: float, z_m: float):
        # MAVLink LANDING_TARGET precision landing stream
        # MAV_FRAME_BODY_NED implies x forward, y right, z down
        self.master.mav.landing_target_send(
            timestamp_ms * 1000, # usec
            0, # target num
            mavutil.mavlink.MAV_FRAME_BODY_NED,
            x_rad, y_rad,
            distance_m,
            0, 0, # target size x y
            x_m, y_m, z_m,
            [1.0,0,0,0], # q (not used usually)
            2, # POSITION_VALID flag
            1 # valid
        )
