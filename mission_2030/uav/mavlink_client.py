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

    def send_landing_target(self, angle_x_rad: float, angle_y_rad: float, distance_m: float):
        """
        Stream MAVLink LANDING_TARGET for ArduPilot precision landing.
        Uses angle mode (MAV_FRAME_BODY_NED) — position fields are zeroed.
        """
        self.master.mav.landing_target_send(
            int(time.time() * 1_000_000),   # time_usec
            0,                               # target_num
            mavutil.mavlink.MAV_FRAME_BODY_NED,
            angle_x_rad,                     # angle_x
            angle_y_rad,                     # angle_y
            distance_m,                      # distance
            0.0, 0.0,                        # size_x, size_y
            0.0, 0.0, 0.0,                   # x, y, z (unused in angle mode)
            [1.0, 0.0, 0.0, 0.0],           # q quaternion
            1,                               # type: LANDING_TARGET_TYPE_LIGHT_BEACON
            1                                # position_valid
        )
