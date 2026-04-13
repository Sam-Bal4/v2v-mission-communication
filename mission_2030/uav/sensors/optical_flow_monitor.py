import time
from mission_2030.common.logging_utils import setup_logger

logger = setup_logger("OpticalFlowMonitor")

class OpticalFlowMonitor:
    def __init__(self, master):
        self.master = master
        self.healthy = False
        self.last_msg_t = 0.0

    def check_health(self) -> bool:
        msg = self.master.recv_match(type='OPTICAL_FLOW', blocking=False)
        if msg:
            self.last_msg_t = time.time()
            if msg.quality > 10:
                self.healthy = True
            else:
                logger.warning("Low optical flow quality detected.")
                self.healthy = False
                
        if time.time() - self.last_msg_t > 2.0:
            self.healthy = False
        return self.healthy
