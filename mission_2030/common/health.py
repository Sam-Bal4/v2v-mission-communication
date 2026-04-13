import time

class HealthMonitor:
    """Generic timeout/heartbeat checker"""
    def __init__(self, timeout_s: float):
        self.timeout_s = timeout_s
        self.last_update_t = 0.0

    def feed(self):
        self.last_update_t = time.time()

    def is_healthy(self) -> bool:
        if self.last_update_t == 0.0:
            return False
        return (time.time() - self.last_update_t) < self.timeout_s
