class MotionController:
    def __init__(self, vehicle):
        self.vehicle = vehicle

    def drive_to_coordinate(self, lat, lon):
        """ Abstract pathing interface """
        self.vehicle.set_mode("GUIDED")
        self.vehicle.set_waypoint(lat, lon, 0)
        
    def stop(self):
        """ Immediate brake """
        self.vehicle.set_mode("HOLD")
