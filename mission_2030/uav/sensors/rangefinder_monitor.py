import time
from mission_2030.common.logging_utils import setup_logger

logger = setup_logger("RangefinderMonitor")

class RangefinderMonitor:
    def __init__(self, master):
        self.master = master
        self.last_alt = 0.0

    def get_altitude(self) -> float:
        msg = self.master.recv_match(type='DISTANCE_SENSOR', blocking=False)
        if msg:
            self.last_alt = msg.current_distance / 100.0
        return self.last_alt
