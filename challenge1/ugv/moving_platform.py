"""
Challenge 1 - UGV Moving Platform
=================================
Hardware: Cube Orange+, TF-Nova Lidar, Dual nested ArUco markers on top.
Logic:
  - Takes off or moves UGV in GUIDED mode at a slow constant speed
  - Runs continuous Lidar obstacle detection to stop if path is blocked
  - Just serves as the moving landing pad for the UAV
"""

import time
import math
import signal
from dronekit import connect, VehicleMode
import serial

# ─── CONFIGURATION ─────────────────────────────────────────────────────────────
UGV_PORT    = "/dev/ttyACM0"
BAUD_RATE   = 115200
LIDAR_PORT  = "/dev/ttyAMA0"
LIDAR_BAUD  = 115200

# Speed config
SPEED_MPS   = 0.45    # ~1.0 mph
OBSTACLE_THRESHOLD_M = 1.0

# ─── SIGNAL HANDLING ──────────────────────────────────────────────────────────
_abort = False
def _sigint(sig, frame):
    global _abort
    _abort = True
signal.signal(signal.SIGINT,  _sigint)
signal.signal(signal.SIGTERM, _sigint)

# ─── LIDAR READER ─────────────────────────────────────────────────────────────
class LidarReader:
    FRAME_HEADER = 0x59
    def __init__(self, port, baud):
        self.ser = serial.Serial(
            port=port, baudrate=baud,
            bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE, timeout=0.1
        )
        self.recent = []

    def read_distance(self):
        self.ser.reset_input_buffer()
        for _ in range(20):
            b1 = self.ser.read(1)
            if not b1 or b1[0] != self.FRAME_HEADER: continue
            b2 = self.ser.read(1)
            if not b2 or b2[0] != self.FRAME_HEADER: continue
            payload = self.ser.read(7)
            if len(payload) < 7: break
            dist_l, dist_h, _, _, _, conf, _ = payload
            if conf < 10: continue
            return ((dist_h << 8) | dist_l) / 100.0
        return 9999.0

    def get_filtered_distance(self):
        raw = self.read_distance()
        if raw < 9999.0:
            self.recent.append(raw)
            if len(self.recent) > 3:
                self.recent.pop(0)
        return min(self.recent) if self.recent else 9999.0

    def close(self):
        self.ser.close()

# ─── UGV CONTROL ──────────────────────────────────────────────────────────────
def build_drive_msg(vehicle, throttle, yaw_rate_deg_s=0.0):
    return vehicle.message_factory.set_attitude_target_encode(
        0, 0, 0, 0xA3, [1.0, 0.0, 0.0, 0.0], 0.0, 0.0, math.radians(yaw_rate_deg_s), throttle
    )

def stop_vehicle(vehicle):
    stop_msg = build_drive_msg(vehicle, 0.0)
    for _ in range(5):
        vehicle.send_mavlink(stop_msg)
        time.sleep(0.1)

def main():
    print("========================================")
    print("   CHALLENGE 1 - MOVING UGV PLATFORM    ")
    print("========================================")

    lidar = LidarReader(LIDAR_PORT, LIDAR_BAUD)
    
    print("Connecting to UGV...")
    vehicle = connect(UGV_PORT, wait_ready=True, baud=BAUD_RATE)
    
    print("Arming UGV...")
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True
    while not vehicle.armed and not _abort:
        time.sleep(0.1)
    
    print(f"Driving forward at {SPEED_MPS} m/s constantly...")
    
    # We set WP_SPEED just in case, but rely on attitude target throttle
    # Actually, for rovers, throttle is 0-1 fraction.
    # 0.5 throttle assumes it moves forward. 
    # MAV_CMD_DO_CHANGE_SPEED can be used, or we just send 0.5 throttle
    # We will use throttle for forward moment.
    throttle_frac = 0.5 

    try:
        while not _abort:
            dist = lidar.get_filtered_distance()
            
            if dist < OBSTACLE_THRESHOLD_M:
                print(f"OBSTACLE DETECTED at {dist:.2f}m. Pausing.")
                stop_vehicle(vehicle)
                time.sleep(1.0) # Wait until it's clear
            else:
                vehicle.send_mavlink(build_drive_msg(vehicle, throttle_frac))
                time.sleep(0.1)
                
    finally:
        print("Stopping mission.")
        stop_vehicle(vehicle)
        vehicle.armed = False
        lidar.close()
        vehicle.close()

if __name__ == "__main__":
    main()
