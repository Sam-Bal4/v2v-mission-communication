import time
class LandingTargetEncoder:
    def __init__(self, master):
        self.master = master

    def send(self, angle_x: float, angle_y: float, distance_m: float):
        from pymavlink import mavutil
        self.master.mav.landing_target_send(
            int(time.time() * 1000000), 
            0, mavutil.mavlink.MAV_FRAME_BODY_NED,
            angle_x, angle_y, distance_m,
            0, 0, 0, 0, 0, [1.0,0,0,0], 2, 1
        )
