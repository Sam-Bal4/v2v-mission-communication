"""
Test 01 – Basic Flight
======================
Arm → Takeoff to 1.3 m → Hover 8 s → LAND mode → Wait for disarm heartbeat.
Validates: motors, ESCs, barometer, EKF3, optical flow + lidar altitude hold.

Run from repo root:
  export PYTHONPATH=$(pwd)
  python3 mission_2030/dennis_test/drone/01_basic_flight.py
"""
import time
import signal
from pymavlink import mavutil

DRONE_PORT      = "/dev/ttyACM0"
BAUD_RATE       = 57600
TARGET_ALT      = 1.3          # metres
HOVER_TIME_S    = 8.0          # seconds to hold altitude
LAND_TIMEOUT_S  = 90.0         # hard limit for landing confirmation

_abort = False
def _sigint(sig, frame):
    global _abort
    _abort = True
signal.signal(signal.SIGINT,  _sigint)
signal.signal(signal.SIGTERM, _sigint)

def request_streams(master):
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
        mavutil.mavlink.MAVLINK_MSG_ID_DISTANCE_SENSOR, int(1e6 / 20),
        0, 0, 0, 0, 0)
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
        mavutil.mavlink.MAVLINK_MSG_ID_HEARTBEAT, int(1e6 / 5),
        0, 0, 0, 0, 0)

def set_mode(master, mode: str):
    mapping = master.mode_mapping()
    master.mav.set_mode_send(
        master.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mapping[mode])
    print(f"Mode → {mode}")
    time.sleep(0.5)

def arm(master) -> bool:
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0, 1, 0, 0, 0, 0, 0, 0)
    deadline = time.time() + 10
    while time.time() < deadline:
        hb = master.recv_match(type='HEARTBEAT', blocking=True, timeout=1.0)
        if hb and hb.get_srcSystem() == master.target_system:
            if hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED:
                print("Armed ✓")
                return True
    print("ERROR: arm timed out.")
    return False

def get_alt(master) -> float:
    alt = 0.0
    while True:
        msg = master.recv_match(type='DISTANCE_SENSOR', blocking=False)
        if msg is None:
            break
        if msg.current_distance > 0:
            alt = msg.current_distance / 100.0
    return alt

def wait_disarm(master, timeout_s: float):
    print(f"Waiting for Cube disarm confirm (max {timeout_s:.0f} s)...")
    deadline = time.time() + timeout_s
    while time.time() < deadline and not _abort:
        hb = master.recv_match(type='HEARTBEAT', blocking=True, timeout=2.0)
        if hb and hb.get_srcSystem() == master.target_system:
            if not (hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
                print("Cube heartbeat: motors DISARMED → touchdown confirmed ✓")
                return True
        alt = get_alt(master)
        if alt > 0:
            print(f"  Land alt: {alt:.2f} m", end="\r", flush=True)
    print("\nWarning: disarm timeout reached.")
    return False

def main():
    print("=" * 50)
    print("  TEST 01 – BASIC FLIGHT")
    print("  Ctrl+C → LAND mode (graceful)")
    print("=" * 50)

    master = mavutil.mavlink_connection(DRONE_PORT, baud=BAUD_RATE)
    master.wait_heartbeat()
    print(f"Heartbeat OK – sysid={master.target_system}")
    request_streams(master)

    try:
        set_mode(master, "GUIDED")
        if not arm(master):
            return

        print(f"Takeoff → {TARGET_ALT} m")
        master.mav.command_long_send(
            master.target_system, master.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0, 0, 0, 0, 0, 0, 0, TARGET_ALT)

        t0 = time.time()
        while time.time() - t0 < 20 and not _abort:
            if get_alt(master) >= TARGET_ALT * 0.90:
                print(f"\nTarget altitude reached ✓")
                break
            time.sleep(0.2)

        if not _abort:
            print(f"Hovering {HOVER_TIME_S:.0f} s...")
            time.sleep(HOVER_TIME_S)

    finally:
        print("Switching to LAND...")
        set_mode(master, "LAND")
        wait_disarm(master, LAND_TIMEOUT_S)
        print("Test 01 complete.")

if __name__ == "__main__":
    main()
