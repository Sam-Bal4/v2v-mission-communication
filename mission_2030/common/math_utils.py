import math

def constrain(val: float, min_val: float, max_val: float) -> float:
    return max(min_val, min(max_val, val))

def wrap_pi(angle_rad: float) -> float:
    return (angle_rad + math.pi) % (2 * math.pi) - math.pi
