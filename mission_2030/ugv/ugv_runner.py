"""
UGV Runner – receives V2V destination from UAV, drives toward it,
avoids obstacles with TF-Nova Lidar, broadcasts heartbeats so UAV
can track phase changes (AVOID_OBSTACLE, NAVIGATE, etc.)

Real DriveKit motion via DroneKit + SET_ATTITUDE_TARGET.
"""
import time
import signal
import math
from dronekit import connect, VehicleMode
from mission_2030.radio.v2v_bridge import V2VBridge
from mission_2030.common.logging_utils import setup_logger
from mission_2030.ugv.state_machine import UgvState
from mission_2030.ugv.obstacle_avoidance import ObstacleAvoidance

logger = setup_logger("UGV_Runner", "ugv_runner.log")

# ── config ───────────────────────────────────────────────────────────────────
ESP32_PORT  = "/dev/ttyUSB0"
UGV_PORT    = "/dev/ttyACM0"   # direct serial to Pixhawk (NOT tcp)
BAUD_RATE   = 115200

DRIVE_SPEED          = 0.4     # m/s forward speed
OBSTACLE_THRESHOLD_M = 1.5     # metres – trigger avoidance
AVOIDANCE_TURN_DEG   = 90.0    # degrees for each avoidance turn
TURN_RATE_DEG_S      = 45.0    # deg/s yaw rate
HEARTBEAT_HZ         = 2.0     # send UGV heartbeat to UAV at this rate

_stop = False
def _sigint(sig, frame):
    global _stop
    _stop = True
signal.signal(signal.SIGINT,  _sigint)
signal.signal(signal.SIGTERM, _sigint)

# ── helpers ──────────────────────────────────────────────────────────────────
def build_drive_msg(vehicle, throttle: float, yaw_rate_dps: float = 0.0):
    return vehicle.message_factory.set_attitude_target_encode(
        0, 0, 0,
        0xA3,               # ignore roll + pitch rate
        [1.0, 0.0, 0.0, 0.0],
        0.0, 0.0,
        math.radians(yaw_rate_dps),
        throttle)

def arm_ugv(vehicle):
    vehicle.parameters["ARMING_CHECK"] = 0
    time.sleep(0.5)
    if vehicle.mode.name == "HOLD":
        vehicle.mode = VehicleMode("MANUAL")
        t0 = time.time()
        while vehicle.mode.name != "MANUAL" and time.time() - t0 < 5:
            time.sleep(0.1)
    vehicle.armed = True
    t0 = time.time()
    while not vehicle.armed and time.time() - t0 < 5:
        time.sleep(0.1)
    if not vehicle.armed:
        raise RuntimeError("UGV failed to arm.")
    logger.info("UGV armed ✓")
    vehicle.mode = VehicleMode("GUIDED")
    t0 = time.time()
    while vehicle.mode.name != "GUIDED" and time.time() - t0 < 5:
        time.sleep(0.1)
    logger.info(f"UGV mode: {vehicle.mode.name}")

def send_stop(vehicle, repeats=5):
    msg = build_drive_msg(vehicle, 0.0)
    for _ in range(repeats):
        vehicle.send_mavlink(msg)
        time.sleep(0.05)

# ── main ─────────────────────────────────────────────────────────────────────
def main():
    logger.info("======== UGV RUNNER START ========")

    bridge = V2VBridge(ESP32_PORT)
    try:
        bridge.connect()
        logger.info("ESP32 bridge connected ✓")
    except Exception as e:
        logger.error(f"Bridge failed: {e}")
        return

    lidar = ObstacleAvoidance(port="/dev/ttyAMA0", baud=115200,
                               green_pin=16, red_pin=19)

    logger.info(f"Connecting to UGV on {UGV_PORT}...")
    vehicle = connect(UGV_PORT, wait_ready=True, baud=BAUD_RATE)
    logger.info("UGV DriveKit connected ✓")

    @vehicle.on_message('STATUSTEXT')
    def on_status(self, name, msg):
        logger.info(f"[FC] {msg.text}")

    state = UgvState.WAIT_FOR_DESTINATION
    seq   = 0

    def broadcast(phase: UgvState, estop=False):
        nonlocal seq
        bridge.send_ugv_heartbeat(seq, int(time.time() * 1000), phase.value, estop)
        seq += 1

    last_hb_t = 0.0
    
    target_drive_time = 0.0
    active_drive_time = 0.0
    last_drive_t = 0.0
    touchdown_start_t = 0.0

    try:
        arm_ugv(vehicle)

        while not _stop and state != UgvState.MISSION_COMPLETE:
            now = time.time()

            # Periodic heartbeat to UAV
            if now - last_hb_t > 1.0 / HEARTBEAT_HZ:
                broadcast(state)
                last_hb_t = now

            # ── WAIT ─────────────────────────────────────────────────
            if state == UgvState.WAIT_FOR_DESTINATION:
                if bridge.latest_destination is not None:
                    dest = bridge.latest_destination
                    logger.info(f"Destination received: Marker {dest.marker_id} "
                                f"X:{dest.x_m:.2f} Y:{dest.y_m:.2f}")
                    state = UgvState.START_MOTION
                time.sleep(0.2)
                continue

            # ── START MOVING ─────────────────────────────────────────
            elif state == UgvState.START_MOTION:
                target_dist = math.hypot(dest.x_m, dest.z_m)
                target_drive_time = target_dist / DRIVE_SPEED if DRIVE_SPEED > 0 else 0
                logger.info(f"Calculated distance: {target_dist:.2f} m. "
                            f"Target drive time: {target_drive_time:.1f} s")
                logger.info("Starting drive toward destination...")
                state = UgvState.NAVIGATE_TO_DESTINATION
                last_drive_t = time.time()

            # ── NAVIGATE ─────────────────────────────────────────────
            elif state == UgvState.NAVIGATE_TO_DESTINATION:
                dt = now - last_drive_t
                last_drive_t = now
                active_drive_time += dt
                
                if active_drive_time >= target_drive_time:
                    logger.info("Destination reached! Stopping.")
                    send_stop(vehicle)
                    state = UgvState.EXPECT_UAV_TOUCHDOWN
                    touchdown_start_t = time.time()
                    continue

                # Check lidar
                dist = lidar.read_lidar()
                if dist < OBSTACLE_THRESHOLD_M:
                    logger.warning(f"Obstacle at {dist:.2f} m – triggering avoidance.")
                    send_stop(vehicle)
                    lidar.led_obstacle()
                    state = UgvState.AVOID_OBSTACLE
                    broadcast(UgvState.AVOID_OBSTACLE)
                    continue

                # Drive forward
                msg = build_drive_msg(vehicle, DRIVE_SPEED)
                vehicle.send_mavlink(msg)
                time.sleep(0.05)

            # ── AVOID ──────────────────────────────────────────────
            elif state == UgvState.AVOID_OBSTACLE:
                broadcast(UgvState.AVOID_OBSTACLE)
                logger.info("Avoidance: turn left...")

                turn_dur = AVOIDANCE_TURN_DEG / TURN_RATE_DEG_S
                t0 = time.time()
                while time.time() - t0 < turn_dur:
                    vehicle.send_mavlink(build_drive_msg(vehicle, 0.0, -TURN_RATE_DEG_S))
                    time.sleep(0.05)
                send_stop(vehicle)
                time.sleep(0.5)

                logger.info("Avoidance: drive past...")
                t0 = time.time()
                while time.time() - t0 < 3.0:
                    vehicle.send_mavlink(build_drive_msg(vehicle, DRIVE_SPEED))
                    time.sleep(0.05)
                send_stop(vehicle)
                time.sleep(0.5)

                logger.info("Avoidance: turn right...")
                t0 = time.time()
                while time.time() - t0 < turn_dur:
                    vehicle.send_mavlink(build_drive_msg(vehicle, 0.0, TURN_RATE_DEG_S))
                    time.sleep(0.05)
                send_stop(vehicle)

                lidar.led_clear()
                logger.info("Obstacle cleared. Resuming navigation.")
                state = UgvState.NAVIGATE_TO_DESTINATION
                last_drive_t = time.time()

            # ── EXPECT TOUCHDOWN ────────────────────────────────────
            elif state == UgvState.EXPECT_UAV_TOUCHDOWN:
                broadcast(UgvState.EXPECT_UAV_TOUCHDOWN)
                if time.time() - touchdown_start_t > 30.0:
                    logger.info("Ride time complete. Mission ending.")
                    state = UgvState.MISSION_COMPLETE
                else:
                    logger.info("Holding at destination. Waiting for UAV touchdown... (Timeout: {:.1f}s)".format(30 - (time.time() - touchdown_start_t)))
                    time.sleep(0.5)

            # ── COMPLETE ─────────────────────────────────────────────
            elif state == UgvState.MISSION_COMPLETE:
                break

            time.sleep(0.02)

    except Exception as e:
        logger.error(f"UGV Runner exception: {e}")
    finally:
        broadcast(UgvState.ESTOP, estop=True)
        send_stop(vehicle)
        vehicle.armed = False
        time.sleep(1)
        vehicle.close()
        bridge.stop()
        logger.info("UGV shutdown complete.")


if __name__ == "__main__":
    main()
