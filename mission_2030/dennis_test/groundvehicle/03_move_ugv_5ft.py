import time
import sys
import os
from dronekit import connect, VehicleMode

sys.path.append(os.path.abspath("../../"))
from mission_2030.radio.v2v_bridge import V2VBridge

UGV_PORT = "/dev/ttyACM0"
ESP32_PORT = "/dev/ttyUSB0"

def build_attitude_msg(vehicle, throttle, yaw_rate=0.0):
    return vehicle.message_factory.set_attitude_target_encode(
        0, 0, 0, 0xA3, [1.0, 0.0, 0.0, 0.0], 0.0, 0.0, yaw_rate, throttle
    )

def main():
    print("--- Test 3: UGV Move via UAV Command ---")
    bridge = V2VBridge(ESP32_PORT)
    bridge.connect()

    vehicle = connect(UGV_PORT, wait_ready=True, baud=115200)
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True
    time.sleep(2)

    print("Waiting for UAV Command logic (Phase 5)...")
    start_drive = False
    while not start_drive:
        if bridge.latest_uav_heartbeat and bridge.latest_uav_heartbeat.mission_phase == 5:
            start_drive = True
            break
        time.sleep(0.5)

    print("Received Command! Driving 5 ft (1.5 meters)...")
    speed_mps = 0.5
    duration = 1.5 / speed_mps
    start_t = time.time()
    
    while time.time() - start_t < duration:
        msg = build_attitude_msg(vehicle, 1.0)
        vehicle.send_mavlink(msg)
        time.sleep(0.1)
        
    print("5 ft reached. Stopping.")
    msg = build_attitude_msg(vehicle, 0.0)
    for _ in range(5): vehicle.send_mavlink(msg)

    vehicle.armed = False
    bridge.stop()
    vehicle.close()

if __name__ == "__main__":
    main()
