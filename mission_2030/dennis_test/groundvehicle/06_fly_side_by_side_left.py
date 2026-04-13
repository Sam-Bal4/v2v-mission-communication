import time
# Compatibility patch for dronekit
import collections
if not hasattr(collections, 'MutableMapping'):
    import collections.abc
    collections.MutableMapping = collections.abc.MutableMapping

from dronekit import connect, VehicleMode
import sys
import os
sys.path.append(os.path.abspath("../../"))
from mission_2030.common.mavlink_utils import arm_vehicle_dronekit

def build_attitude_msg(vehicle, throttle, yaw_rate=0.0):
    return vehicle.message_factory.set_attitude_target_encode(
        0, 0, 0, 0xA3, [1.0, 0.0, 0.0, 0.0], 0.0, 0.0, yaw_rate, throttle
    )

def main():
    print("--- Test 6: UGV Pacing LEFT ---")
    vehicle = connect("/dev/ttyACM0", wait_ready=True, baud=115200)
    if not arm_vehicle_dronekit(vehicle, mode_name="GUIDED"):
        return
    time.sleep(5) 

    print("Turning Left for pacing test...")
    start_t = time.time()
    while time.time() - start_t < 10:
        msg = build_attitude_msg(vehicle, 1.0, yaw_rate=-0.5)
        vehicle.send_mavlink(msg)
        time.sleep(0.1)
        
    print("Stop.")
    for _ in range(5): vehicle.send_mavlink(build_attitude_msg(vehicle, 0.0))
    vehicle.armed = False
    vehicle.close()

if __name__ == "__main__":
    main()
