from pymavlink import mavutil
from mission_2030.common.logging_utils import setup_logger

logger = setup_logger("UgvMavlink")

class MavlinkGroundVehicle:
    def __init__(self, port: str):
        self.port = port
        self.master = mavutil.mavlink_connection(port)
        self.master.wait_heartbeat()
        logger.info("UGV Base Connection fully established.")

    def set_mode(self, mode: str):
        mapping = self.master.mode_mapping()
        if mode in mapping:
            self.master.mav.set_mode_send(
                self.master.target_system,
                mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                mapping[mode]
            )

    def set_waypoint(self, lat, lon, alt):
         self.master.mav.mission_item_int_send(
            self.master.target_system, self.master.target_component,
            0, mavutil.mavlink.MAV_FRAME_GLOBAL_INT,
            mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
            2, 0, 0, 0, 0, 0,
            int(lat * 1e7), int(lon * 1e7), alt
        )
