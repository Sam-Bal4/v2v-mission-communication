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
from mission_2030.ugv.obstacle_avoidance import ObstacleAvoidance

UGV_PORT = "/dev/ttyACM0"
ESP32_PORT = "/dev/ttyUSB0"

def build_drive_msg(vehicle, throttle, yaw_rate=0.0):
    """
    throttle: roughly -1.0 to 1.0 mapped to ESC PWM
    yaw_rate: rad/s for differential steering
    """
    return vehicle.message_factory.set_attitude_target_encode(
        0, 0, 0, 0xA3, [1.0, 0.0, 0.0, 0.0], 0.0, 0.0, yaw_rate, throttle
    )

def send_stop(vehicle):
    msg = build_drive_msg(vehicle, 0.0, 0.0)
    for _ in range(5):
        vehicle.send_mavlink(msg)
        time.sleep(0.01)

def main():
    print("--- Test 11: UGV Arm & Avoid via UAV Command ---")
    bridge = V2VBridge(ESP32_PORT)
    bridge.connect()

    print("Initializing TF-Nova Lidar...")
    # Matches competition UGV baud & port
    lidar = ObstacleAvoidance(port="/dev/ttyAMA0", baud=115200)

    vehicle = connect(UGV_PORT, wait_ready=True, baud=115200)

    print("Waiting for UAV Command (Phase 11)...")
    start_drive = False
    while not start_drive:
        if bridge.latest_uav_heartbeat and bridge.latest_uav_heartbeat.mission_phase == 11:
            start_drive = True
            break
        time.sleep(0.5)

    print("Command Received! Arming UGV...")
    if not arm_vehicle_dronekit(vehicle, mode_name="GUIDED"):
        return

    print("Starting Drive with Obstacle Avoidance...")
    
    DRIVE_SPEED = 0.5
    TURN_RATE_DEG_S = 45.0 * (3.14159 / 180.0) # ~0.78 rad/s
    OBSTACLE_THRESHOLD_M = 1.6

    active = True
    while active:
        # Check if UAV turned off Phase 11
        if bridge.latest_uav_heartbeat and bridge.latest_uav_heartbeat.mission_phase != 11:
            print("UAV commanded stop. Finishing test.")
            active = False
            break

        dist = lidar.read_lidar()
        if dist < OBSTACLE_THRESHOLD_M:
            print(f"Obstacle detected at {dist:.2f}m! Executing maneuver...")
            send_stop(vehicle)
            lidar.led_obstacle()
            time.sleep(0.5)

            print("  -> Turn Left")
            t0 = time.time()
            turn_dur = 90.0 / 45.0 # 90 deg at 45 deg/s = 2 sec
            while time.time() - t0 < turn_dur:
                vehicle.send_mavlink(build_drive_msg(vehicle, 0.0, -TURN_RATE_DEG_S))
                time.sleep(0.05)
            send_stop(vehicle)
            time.sleep(0.5)

            print("  -> Drive Past")
            t0 = time.time()
            while time.time() - t0 < 3.0:
                vehicle.send_mavlink(build_drive_msg(vehicle, DRIVE_SPEED, 0.0))
                time.sleep(0.05)
            send_stop(vehicle)
            time.sleep(0.5)

            print("  -> Turn Right")
            t0 = time.time()
            while time.time() - t0 < turn_dur:
                vehicle.send_mavlink(build_drive_msg(vehicle, 0.0, TURN_RATE_DEG_S))
                time.sleep(0.05)
            send_stop(vehicle)
            time.sleep(0.5)

            lidar.led_clear()
            print("Resuming straight forward...")

        else:
            # Clear path, keep driving forward
            vehicle.send_mavlink(build_drive_msg(vehicle, DRIVE_SPEED, 0.0))
            time.sleep(0.05)

    print("Stopping and Disarming UGV.")
    send_stop(vehicle)
    vehicle.armed = False
    
    bridge.stop()
    vehicle.close()
    print("Test 11 UGV complete ✓")

if __name__ == "__main__":
    main()
