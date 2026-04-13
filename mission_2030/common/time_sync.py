import time

def current_time_ms() -> int:
    return (int(time.time() * 1000) & 0xFFFFFFFF)
