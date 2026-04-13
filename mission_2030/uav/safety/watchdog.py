import time

class Watchdog:
    def __init__(self, timeout: float):
        self.timeout = timeout
        self.last_pet = time.time()

    def pet(self):
        self.last_pet = time.time()

    def is_triggered(self) -> bool:
        return (time.time() - self.last_pet) > self.timeout
