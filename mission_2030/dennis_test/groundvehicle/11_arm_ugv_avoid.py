"""
Test 11 - UGV Arm & Obstacle Avoidance (UGV side)
===================================================
Mirrors the production Challenge3v2.py logic, triggered by a UAV phase=11
radio signal instead of running standalone.

Obstacle avoidance uses:
  - TF-Nova lidar (UART /dev/ttyAMA0) with rolling 3-sample filter
  - Compass-guided 90-degree turns (vehicle.heading, slow-update method)
  - GPIO LEDs: green = clear, red = obstacle/avoidance active
  - Full U-shape bypass: Left -> Sidestep -> Right -> Forward -> Right -> Sidestep -> Left

Run on Raspberry Pi from repo root:
  export PYTHONPATH=$(pwd)
  python3 mission_2030/dennis_test/groundvehicle/11_arm_ugv_avoid.py
"""
import time
import math
import signal
import sys
import os

# Python 3.10+ DroneKit compatibility patch
import collections
if not hasattr(collections, 'MutableMapping'):
    import collections.abc
    collections.MutableMapping = collections.abc.MutableMapping

from dronekit import connect, VehicleMode

import serial

sys.path.append(os.path.abspath("../../"))
from mission_2030.radio.v2v_bridge import V2VBridge

# ── Ports ────────────────────────────────────────────────────────────────────
UGV_PORT        = "/dev/ttyACM0"
UGV_BAUD        = 115200
ESP32_PORT      = "/dev/ttyUSB0"
LIDAR_PORT      = "/dev/ttyAMA0"
LIDAR_BAUD      = 115200

# ── GPIO LEDs ─────────────────────────────────────────────────────────────────
GREEN_LED_PIN   = 16
RED_LED_PIN     = 19

# ── Motion parameters (from Challenge3v2 tuning) ─────────────────────────────
SPEED_MPS                = 0.8 * 0.44704   # 0.8 mph in m/s
OBSTACLE_THRESHOLD_M     = 2.0 * 0.3048    # 2 ft in metres
AVOIDANCE_SIDESTEP_M     = 2.0 * 0.3048    # side-step width
BYPASS_FORWARD_M         = 3.0 * 0.3048    # forward leg while bypassing
AVOIDANCE_PROGRESS_M     = 4.0 * 0.3048    # estimated net forward gain per bypass
DRIVE_DISTANCE_M         = 10.0 * 0.3048   # how far to drive before stopping

# ── Compass-based turn tuning (from Challenge3v2) ─────────────────────────────
TURN_RATE_DEG_S          = 10.0
HEADING_CHECK_INTERVAL_S = 0.20
STOP_EARLY_DEG           = 8.0
STABLE_COUNT_REQUIRED    = 2
TURN_TOLERANCE_DEG       = 5.0

# ── TF-Nova constants ─────────────────────────────────────────────────────────
FRAME_HEADER             = 0x59
LIDAR_MIN_CONFIDENCE     = 10
LIDAR_NO_TARGET_M        = 9999.0
LED_FLASH_INTERVAL_S     = 0.3

# ── Signal handler ────────────────────────────────────────────────────────────
_abort = False
def _sigint(sig, frame):
    global _abort
    _abort = True
signal.signal(signal.SIGINT,  _sigint)
signal.signal(signal.SIGTERM, _sigint)


# ── LED helpers ───────────────────────────────────────────────────────────────
def open_leds():
    try:
        from gpiozero import LED
        try:
            from gpiozero.pins.lgpio import LGPIOFactory
            factory = LGPIOFactory()
        except ImportError:
            factory = None
        kwargs = {"pin_factory": factory} if factory else {}
        green = LED(GREEN_LED_PIN, **kwargs)
        red   = LED(RED_LED_PIN, **kwargs)
        green.off()
        red.off()
        return green, red
    except Exception as e:
        print(f"[LED] GPIO unavailable: {e}  (running without LEDs)")
        return None, None

def led_clear(green, red):
    if red:   red.off()
    if green: green.on()

def led_obstacle(green, red):
    if green: green.off()
    if red:   red.on()

def flash_red_tick(red, last_t):
    now = time.time()
    if red and now - last_t >= LED_FLASH_INTERVAL_S:
        red.toggle()
        return now
    return last_t


# ── TF-Nova lidar ─────────────────────────────────────────────────────────────
def open_lidar():
    ser = serial.Serial(
        port=LIDAR_PORT, baudrate=LIDAR_BAUD,
        bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE, timeout=0.1
    )
    ser.reset_input_buffer()
    print(f"TF-Nova lidar opened on {LIDAR_PORT}")
    return ser

def read_lidar_m(ser):
    ser.reset_input_buffer()
    for _ in range(18):
        b1 = ser.read(1)
        if not b1 or b1[0] != FRAME_HEADER: continue
        b2 = ser.read(1)
        if not b2 or b2[0] != FRAME_HEADER: continue
        payload = ser.read(7)
        if len(payload) < 7: return LIDAR_NO_TARGET_M
        dist_l, dist_h, _, _, _, confidence, checksum = payload
        raw = [FRAME_HEADER, FRAME_HEADER, dist_l, dist_h,
               payload[2], payload[3], payload[4], confidence]
        if (sum(raw) & 0xFF) != checksum: continue
        distance_cm = (dist_h << 8) | dist_l
        if confidence < LIDAR_MIN_CONFIDENCE or distance_cm == 0:
            return LIDAR_NO_TARGET_M
        return distance_cm / 100.0
    return LIDAR_NO_TARGET_M

def get_filtered_lidar(ser, recent, window=3):
    raw = read_lidar_m(ser)
    if raw < LIDAR_NO_TARGET_M:
        recent.append(raw)
        if len(recent) > window:
            recent.pop(0)
    return raw, (min(recent) if recent else LIDAR_NO_TARGET_M)


# ── Vehicle helpers ───────────────────────────────────────────────────────────
def build_drive_msg(vehicle, throttle, yaw_rate_deg_s=0.0):
    return vehicle.message_factory.set_attitude_target_encode(
        0, 0, 0, 0xA3,
        [1.0, 0.0, 0.0, 0.0],
        0.0, 0.0, math.radians(yaw_rate_deg_s),
        throttle
    )

def send_stop(vehicle, repeats=5):
    msg = build_drive_msg(vehicle, 0.0)
    for _ in range(repeats):
        vehicle.send_mavlink(msg)
        time.sleep(0.1)

def get_heading(vehicle):
    h = vehicle.heading
    return float(h) if h is not None else 0.0

def angle_diff_deg(current, start):
    return ((current - start + 540) % 360) - 180

def arm_ugv(vehicle):
    print("Setting ARMING_CHECK=0 (bypass GPS requirement)...")
    vehicle.parameters["ARMING_CHECK"] = 0
    time.sleep(0.5)

    if vehicle.mode.name == "HOLD":
        vehicle.mode = VehicleMode("MANUAL")
        t0 = time.time()
        while vehicle.mode.name != "MANUAL" and time.time() - t0 < 5:
            time.sleep(0.1)

    print("Arming...")
    vehicle.armed = True
    t0 = time.time()
    while not vehicle.armed and time.time() - t0 < 10:
        time.sleep(0.1)
    if not vehicle.armed:
        raise RuntimeError("Failed to arm UGV.")
    print("Armed [OK]")

    print("Switching to GUIDED...")
    vehicle.mode = VehicleMode("GUIDED")
    t0 = time.time()
    while vehicle.mode.name != "GUIDED" and time.time() - t0 < 5:
        time.sleep(0.1)
    print(f"Mode: {vehicle.mode.name}  Armed: {vehicle.armed}")


# ── Compass-accurate turns ────────────────────────────────────────────────────
def turn_left(vehicle, angle_deg, green, red, flashing=False):
    start = get_heading(vehicle)
    stop_at = abs(angle_deg) - STOP_EARLY_DEG
    msg = build_drive_msg(vehicle, 0.0, -abs(TURN_RATE_DEG_S))
    stable = 0
    last_flash = time.time()
    print(f"Turn LEFT {angle_deg:.0f}  from heading={start:.1f}")
    while True:
        vehicle.send_mavlink(msg)
        if flashing: last_flash = flash_red_tick(red, last_flash)
        time.sleep(HEADING_CHECK_INTERVAL_S)
        delta = angle_diff_deg(get_heading(vehicle), start)
        if delta <= -(stop_at - TURN_TOLERANCE_DEG):
            stable += 1
        else:
            stable = 0
        if stable >= STABLE_COUNT_REQUIRED:
            break
    send_stop(vehicle)
    time.sleep(0.6)
    print(f"  -> heading now {get_heading(vehicle):.1f}")

def turn_right(vehicle, angle_deg, green, red, flashing=False):
    start = get_heading(vehicle)
    stop_at = abs(angle_deg) - STOP_EARLY_DEG
    msg = build_drive_msg(vehicle, 0.0, abs(TURN_RATE_DEG_S))
    stable = 0
    last_flash = time.time()
    print(f"Turn RIGHT {angle_deg:.0f}  from heading={start:.1f}")
    while True:
        vehicle.send_mavlink(msg)
        if flashing: last_flash = flash_red_tick(red, last_flash)
        time.sleep(HEADING_CHECK_INTERVAL_S)
        delta = angle_diff_deg(get_heading(vehicle), start)
        if delta >= (stop_at - TURN_TOLERANCE_DEG):
            stable += 1
        else:
            stable = 0
        if stable >= STABLE_COUNT_REQUIRED:
            break
    send_stop(vehicle)
    time.sleep(0.6)
    print(f"  -> heading now {get_heading(vehicle):.1f}")


# ── Forward drive with lidar obstacle check ───────────────────────────────────
def drive_forward(vehicle, lidar_ser, distance_m, green, red,
                  check_obstacle=True, flashing=False, label="DRIVE"):
    if distance_m <= 0:
        return {"obstacle_hit": False, "distance_completed_m": 0.0}

    duration_s = distance_m / SPEED_MPS
    msg = build_drive_msg(vehicle, 1.0)
    recent = []
    start_t = time.time()
    last_flash = time.time()
    last_print = 0.0
    print(f"{label}: driving {distance_m:.2f} m @ {SPEED_MPS:.3f} m/s")

    while (time.time() - start_t) < duration_s and not _abort:
        vehicle.send_mavlink(msg)
        elapsed = time.time() - start_t
        if flashing: last_flash = flash_red_tick(red, last_flash)

        raw, filtered = get_filtered_lidar(lidar_ser, recent)

        if elapsed - last_print >= 0.5:
            lidar_str = f"{filtered:.2f} m" if filtered < LIDAR_NO_TARGET_M else "no target"
            print(f"  t={elapsed:.1f}s  lidar={lidar_str}  heading={get_heading(vehicle):.0f}")
            last_print = elapsed

        if check_obstacle and filtered < OBSTACLE_THRESHOLD_M:
            done = min(elapsed * SPEED_MPS, distance_m)
            print(f"  *** Obstacle at {filtered:.2f} m - STOP! (completed {done:.2f} m)")
            send_stop(vehicle)
            return {"obstacle_hit": True, "distance_completed_m": done}

        time.sleep(0.05)

    send_stop(vehicle)
    return {"obstacle_hit": False, "distance_completed_m": distance_m}


# ── U-shape bypass maneuver ───────────────────────────────────────────────────
def avoid_obstacle(vehicle, lidar_ser, green, red):
    print("\n[Avoidance] Starting U-shape bypass (red LED on)...")
    led_obstacle(green, red)
    time.sleep(0.5)

    # Left -> sidestep -> right -> forward -> right -> sidestep back -> left
    turn_left(vehicle, 90.0, green, red, flashing=True); time.sleep(4.0)
    drive_forward(vehicle, lidar_ser, AVOIDANCE_SIDESTEP_M, green, red,
                  check_obstacle=False, flashing=True, label="SIDESTEP"); time.sleep(4.0)
    turn_right(vehicle, 90.0, green, red, flashing=True); time.sleep(4.0)
    drive_forward(vehicle, lidar_ser, BYPASS_FORWARD_M, green, red,
                  check_obstacle=False, flashing=True, label="BYPASS FWD"); time.sleep(4.0)
    turn_right(vehicle, 90.0, green, red, flashing=True); time.sleep(4.0)
    drive_forward(vehicle, lidar_ser, AVOIDANCE_SIDESTEP_M, green, red,
                  check_obstacle=False, flashing=True, label="RETURN"); time.sleep(4.0)
    turn_left(vehicle, 90.0, green, red, flashing=True); time.sleep(4.0)

    print("[Avoidance] Bypass complete - resuming mission direction")
    led_clear(green, red)
    return AVOIDANCE_PROGRESS_M


# ── Main leg executor ─────────────────────────────────────────────────────────
def execute_leg(vehicle, lidar_ser, green, red, total_m, label="DRIVE"):
    remaining = total_m
    print(f"\n--- {label}: total {remaining:.2f} m ---")
    while remaining > 0.0 and not _abort:
        result = drive_forward(vehicle, lidar_ser, remaining, green, red,
                               check_obstacle=True, label=label)
        remaining -= result["distance_completed_m"]
        remaining = max(0.0, remaining)
        if not result["obstacle_hit"]:
            print(f"{label}: leg complete.")
            break
        progress = avoid_obstacle(vehicle, lidar_ser, green, red)
        remaining -= progress
        remaining = max(0.0, remaining)
        print(f"{label}: remaining after avoidance = {remaining:.2f} m")


# ── Entry point ───────────────────────────────────────────────────────────────
def main():
    print("=" * 52)
    print("  TEST 11 - UGV ARM & AVOID (UGV side)")
    print("  Ctrl+C -> stops cleanly and disarms")
    print("=" * 52)

    # Init radio bridge
    bridge = V2VBridge(ESP32_PORT)
    bridge.connect()
    print("ESP32 bridge connected [OK]")

    # Init LEDs
    green, red = open_leds()

    # Init Lidar
    lidar_ser = open_lidar()
    time.sleep(1.0)
    sample = read_lidar_m(lidar_ser)
    print(f"Lidar warm-up reading: {sample:.2f} m" if sample < LIDAR_NO_TARGET_M else "Lidar warm-up: no target")

    # Wait for UAV Phase 11
    print("Waiting for UAV Phase 11 command over radio...")
    while not _abort:
        hb = bridge.latest_uav_heartbeat
        if hb and hb.mission_phase == 11:
            print("Phase 11 received from UAV [OK]")
            break
        time.sleep(0.3)

    if _abort:
        lidar_ser.close()
        bridge.stop()
        return

    # Connect DroneKit & arm
    print(f"Connecting to UGV on {UGV_PORT}...")
    vehicle = connect(UGV_PORT, wait_ready=True, baud=UGV_BAUD)

    @vehicle.on_message('STATUSTEXT')
    def on_fc_msg(self, name, message):
        print(f"[FC] {message.text}")

    try:
        led_clear(green, red)
        arm_ugv(vehicle)

        # Drive forward (with full obstacle avoidance) until UAV stops Phase 11
        print("Driving forward with obstacle avoidance...")
        execute_leg(vehicle, lidar_ser, green, red, DRIVE_DISTANCE_M, label="LEG 1")

        print("Drive complete. Checking UAV signal for more...")
        # If UAV is still broadcasting phase=11, keep looping
        while not _abort:
            hb = bridge.latest_uav_heartbeat
            if hb and hb.mission_phase != 11:
                print(f"UAV sent phase={hb.mission_phase} - stopping.")
                break
            time.sleep(0.5)

    finally:
        print("Stopping and disarming UGV...")
        send_stop(vehicle)
        led_obstacle(green, red)
        vehicle.armed = False
        t0 = time.time()
        while vehicle.armed and time.time() - t0 < 10:
            time.sleep(0.2)
        if green: green.off()
        if red:   red.off()
        lidar_ser.close()
        bridge.stop()
        vehicle.close()
        print("Test 11 UGV complete [OK]")

if __name__ == "__main__":
    main()
