import time
import sys
import os
# Compatibility patch for dronekit
import collections
if not hasattr(collections, 'MutableMapping'):
    import collections.abc
    collections.MutableMapping = collections.abc.MutableMapping

from dronekit import connect, VehicleMode

sys.path.append(os.path.abspath("../../"))
from mission_2030.radio.v2v_bridge import V2VBridge
from mission_2030.common.mavlink_utils import arm_vehicle_dronekit

def main():
    print("--- Test 4: UGV Receives Coord and Drives ---")
    bridge = V2VBridge("/dev/ttyUSB0")
    bridge.connect()

    vehicle = connect("/dev/ttyACM0", wait_ready=True, baud=115200)
    if not arm_vehicle_dronekit(vehicle, mode_name="GUIDED"):
        return

    print("Waiting for UAV to send destination payload...")
    dest = None
    while not dest:
        if bridge.latest_destination is not None:
             dest = bridge.latest_destination
             break
        time.sleep(0.5)

    print(f"Goal received! X:{dest.x_m:.2f} Y:{dest.y_m:.2f}")
    # Pseudo-drive simulating moving towards X/Y over Dronekit
    # Real dronekit would use vehicle.simple_goto(LocationGlobalRelative)
    print("Simulating driving path duration...")
    time.sleep(abs(dest.x_m) + abs(dest.y_m)) # Hacky timeout based on distance
    
    print("Destination Reached.")
    vehicle.armed = False
    vehicle.close()
    bridge.stop()

if __name__ == "__main__":
    main()
