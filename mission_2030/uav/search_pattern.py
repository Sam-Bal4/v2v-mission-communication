import time
from mission_2030.common.logging_utils import setup_logger
from mission_2030.uav.ardupilot_control import ArdupilotControl

logger = setup_logger("SearchPattern")

class SearchPattern:
    def __init__(self, control: ArdupilotControl):
        self.control = control
        self.scan_speed = 0.1 # m/s body forward vector
        self.last_shift_t = time.time()
        self.turn_state = 0

    def step_scout_drift(self):
        """
        Commands a slow drift to discover ArUco markers based on standard lawnmower.
        """
        # Very simple drift forward pattern
        self.control.set_velocity_body(self.scan_speed, 0.0, 0.0)
