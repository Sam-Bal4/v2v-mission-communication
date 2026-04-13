import time
from pymavlink import mavutil
from mission_2030.common.logging_utils import setup_logger
from mission_2030.uav.ardupilot_control import ArdupilotControl

logger = setup_logger("Takeoff")

class TakeoffManager:
    def __init__(self, master, control: ArdupilotControl):
        self.master = master
        self.control = control

    def request_takeoff(self, target_altitude_m: float, timeout_s: float = 15.0) -> bool:
        """
        Executes native MAV_CMD_NAV_TAKEOFF and blocks until the altitude is reached or it times out.
        """
        logger.info(f"Issuing Takeoff command to {target_altitude_m}m")
        self.master.mav.command_long_send(
            self.master.target_system, self.master.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0, 0, 0, 0, 0, 0, 0, target_altitude_m
        )

        start_t = time.time()
        while time.time() - start_t < timeout_s:
            msg = self.master.recv_match(type='DISTANCE_SENSOR', blocking=True, timeout=1.0)
            if msg:
                current_alt = msg.current_distance / 100.0
                if current_alt >= (target_altitude_m * 0.9):
                    logger.info("Takeoff altitude reached.")
                    return True
            time.sleep(0.1)

        logger.warning("Takeoff timeline exceeded threshold. Verifying hold.")
        return False
