import time
from pymavlink import mavutil
from mission_2030.common.logging_utils import setup_logger
from mission_2030.uav.ardupilot_control import ArdupilotControl

logger = setup_logger("Takeoff")

class TakeoffManager:
    def __init__(self, master, control: ArdupilotControl):
        self.master = master
        self.control = control

    def request_streams(self):
        """Ask the Cube to stream DISTANCE_SENSOR at 20 Hz."""
        self.master.mav.command_long_send(
            self.master.target_system, self.master.target_component,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
            0,
            mavutil.mavlink.MAVLINK_MSG_ID_DISTANCE_SENSOR,
            int(1e6 / 20),   # 20 Hz = 50 000 us interval
            0, 0, 0, 0, 0)

    def request_takeoff(self, target_altitude_m: float, timeout_s: float = 20.0) -> bool:
        """
        Issues GUIDED + ARM + NAV_TAKEOFF and blocks until LidarLite reads ≥90% of target.
        Uses non-blocking DISTANCE_SENSOR drain to avoid freezing the loop.
        """
        logger.info(f"Issuing Takeoff to {target_altitude_m:.1f} m")
        self.request_streams()
        self.master.mav.command_long_send(
            self.master.target_system, self.master.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0, 0, 0, 0, 0, 0, 0, target_altitude_m)

        last_alt = 0.0
        start_t  = time.time()

        while time.time() - start_t < timeout_s:
            # Drain non-blocking — take the freshest reading
            while True:
                msg = self.master.recv_match(type='DISTANCE_SENSOR', blocking=False)
                if msg is None:
                    break
                if msg.current_distance > 0:
                    last_alt = msg.current_distance / 100.0

            logger.info(f"  Alt: {last_alt:.2f} m / {target_altitude_m:.1f} m")
            if last_alt >= target_altitude_m * 0.90:
                logger.info("Takeoff altitude reached ✓")
                return True
            time.sleep(0.2)

        logger.warning("Takeoff timeout – proceeding anyway.")
        return False
