"""
Test 11 – UGV Arm & Obstacle Avoidance (UGV side)
===================================================
1. Boots ESP32 radio bridge and waits for UAV to broadcast phase=11.
2. Once commanded, arms the UGV via DroneKit and starts driving forward.
3. TF-Nova Lidar monitors for obstacles. On detection:
     - Stop
     - Turn Left 90°
     - Drive past (~3 s)
     - Turn Right 90°
     - Resume straight
4. Stops and disarms cleanly when UAV broadcasts phase=0, or Ctrl+C.

Run from repo root on RPi:
  export PYTHONPATH=$(pwd)
  python3 mission_2030/dennis_test/groundvehicle/11_arm_ugv_avoid.py
"""
import time
import signal
import sys
import os

# Python 3.10+ DroneKit compatibility patch
import collections
if not hasattr(collections, 'MutableMapping'):
    import collections.abc
    collections.MutableMapping = collections.abc.MutableMapping

from dronekit import connect, VehicleMode

sys.path.append(os.path.abspath("../../"))
from mission_2030.radio.v2v_bridge import V2VBridge
from mission_2030.common.mavlink_utils import arm_vehicle_dronekit
from mission_2030.ugv.obstacle_avoidance import ObstacleAvoidance

UGV_PORT    = "/dev/ttyACM0"
ESP32_PORT  = "/dev/ttyUSB0"
BAUD        = 115200

DRIVE_SPEED           = 0.5     # throttle 0-1
TURN_RATE_DEG_S       = 45.0   # degrees per second
TURN_90_S             = 90.0 / TURN_RATE_DEG_S   # 2 seconds
TURN_RATE_RAD_S       = TURN_RATE_DEG_S * (3.14159 / 180.0)
OBSTACLE_THRESHOLD_M  = 1.6    # metres – TF-Nova trigger distance
PAST_DRIVE_S          = 3.0    # seconds to drive past obstacle

_abort = False
def _sigint(sig, frame):
    global _abort
    _abort = True
signal.signal(signal.SIGINT,  _sigint)
signal.signal(signal.SIGTERM, _sigint)

# ── MAVLink helpers ──────────────────────────────────────────────────────────

def build_drive_msg(vehicle, throttle, yaw_rate=0.0):
    """Send an attitude-target command: throttle -1..1, yaw_rate rad/s."""
    return vehicle.message_factory.set_attitude_target_encode(
        0, 0, 0, 0xA3,           # time_boot, target_sys, target_comp, type_mask
        [1.0, 0.0, 0.0, 0.0],   # quaternion (identity)
        0.0, 0.0, yaw_rate,      # roll / pitch / yaw rates
        throttle
    )

def send_stop(vehicle):
    msg = build_drive_msg(vehicle, 0.0, 0.0)
    for _ in range(8):
        vehicle.send_mavlink(msg)
        time.sleep(0.02)

def drive_forward(vehicle):
    vehicle.send_mavlink(build_drive_msg(vehicle, DRIVE_SPEED, 0.0))

def turn(vehicle, direction_sign, duration_s):
    """direction_sign: -1 = left, +1 = right"""
    t0 = time.time()
    while time.time() - t0 < duration_s and not _abort:
        vehicle.send_mavlink(build_drive_msg(vehicle, 0.0, direction_sign * TURN_RATE_RAD_S))
        time.sleep(0.05)
    send_stop(vehicle)
    time.sleep(0.3)

def avoid_obstacle(vehicle, lidar):
    """Full left-bypass maneuver."""
    print("\n  [Avoidance] Obstacle detected - stopping")
    send_stop(vehicle)
    lidar.led_obstacle()
    time.sleep(0.4)

    print("  [Avoidance] Turn Left 90°")
    turn(vehicle, -1, TURN_90_S)

    print("  [Avoidance] Drive past obstacle")
    t0 = time.time()
    while time.time() - t0 < PAST_DRIVE_S and not _abort:
        drive_forward(vehicle)
        time.sleep(0.05)
    send_stop(vehicle)
    time.sleep(0.3)

    print("  [Avoidance] Turn Right 90°")
    turn(vehicle, +1, TURN_90_S)

    lidar.led_clear()
    print("  [Avoidance] Resuming straight\n")

# ── Main ─────────────────────────────────────────────────────────────────────

def main():
    print("=" * 52)
    print("  TEST 11 - UGV ARM & AVOID (UGV side)")
    print("  Ctrl+C -> stops cleanly and disarms")
    print("=" * 52)

    # ── 1. Init radio bridge ──────────────────────────────
    bridge = V2VBridge(ESP32_PORT)
    bridge.connect()
    print("ESP32 bridge connected [OK]")

    # ── 2. Init Lidar ─────────────────────────────────────
    lidar = ObstacleAvoidance(port="/dev/ttyAMA0", baud=115200)
    print("TF-Nova Lidar initialised [OK]")

    # ── 3. Wait for UAV start signal ──────────────────────
    print("Waiting for UAV Phase 11 command over radio...")
    while not _abort:
        hb = bridge.latest_uav_heartbeat
        if hb and hb.mission_phase == 11:
            print("Phase 11 received from UAV [OK]")
            break
        time.sleep(0.3)

    if _abort:
        bridge.stop()
        print("Aborted before arming.")
        return

    # ── 4. Connect DroneKit & arm ─────────────────────────
    print(f"Connecting to UGV on {UGV_PORT}...")
    vehicle = connect(UGV_PORT, wait_ready=True, baud=BAUD)

    if not arm_vehicle_dronekit(vehicle, mode_name="GUIDED"):
        print("ERROR: Failed to arm UGV. Aborting.")
        bridge.stop()
        vehicle.close()
        return

    print("UGV Armed and driving! (Ctrl+C to stop)")

    # ── 5. Drive loop ─────────────────────────────────────
    try:
        while not _abort:
            # Check for UAV stop signal
            hb = bridge.latest_uav_heartbeat
            if hb and hb.mission_phase != 11:
                print(f"\nUAV sent phase={hb.mission_phase} - stopping.")
                break

            dist = lidar.read_lidar()
            if dist < OBSTACLE_THRESHOLD_M:
                avoid_obstacle(vehicle, lidar)
            else:
                drive_forward(vehicle)
                time.sleep(0.05)

    finally:
        # ── 6. Stop & disarm ─────────────────────────────
        print("Stopping and disarming UGV...")
        send_stop(vehicle)
        vehicle.armed = False
        t0 = time.time()
        while vehicle.armed and time.time() - t0 < 10:
            time.sleep(0.2)
        bridge.stop()
        vehicle.close()
        print("Test 11 UGV complete [OK]")

if __name__ == "__main__":
    main()
