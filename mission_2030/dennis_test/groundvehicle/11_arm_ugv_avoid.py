"""
Test 11 - UGV Triple-Sensor Arm & Avoid (UGV side)
===================================================
Orchestrates three sensors for maximum obstacle reliability:
1. TF-Nova Lidar (UART) - High accuracy long range.
2. Oak-D Lite + BrainChip (Camera) - AI-based object detection (cones, boxes).
3. Ultrasonic (GPIO) - Low-speed short-range fail-safe.

Uses compass-guided turns and a production U-shape bypass maneuver.
Triggered by UAV phase=11 over ESP32 V2V radio.

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

# Sensor-specific imports
try:
    import depthai as dai
except ImportError:
    dai = None

try:
    from akida import Model, devices
except ImportError:
    Model, devices = None, None

from gpiozero import DistanceSensor
try:
    from gpiozero.pins.lgpio import LGPIOFactory
    _pin_factory = LGPIOFactory()
except ImportError:
    _pin_factory = None

sys.path.append(os.path.abspath("../../"))
from mission_2030.radio.v2v_bridge import V2VBridge

# ── Configuration ─────────────────────────────────────────────────────────────
UGV_PORT           = "/dev/ttyACM0"
UGV_BAUD           = 115200
ESP32_PORT         = "/dev/ttyUSB0"
LIDAR_PORT         = "/dev/ttyAMA0"
LIDAR_BAUD         = 115200

# GPIO Pins
GREEN_LED_PIN      = 16
RED_LED_PIN        = 19
ULTRASONIC_ECHO    = 10
ULTRASONIC_TRIG    = 9

# Detection Constants
MODEL_PATH         = "/home/raspberry/Desktop/UGVCode/akida_model.fbz"
CONF_THRESH        = 0.60
OBSTACLE_THRESHOLD_M = 1.5 * 0.3048   # ~0.45m
AVOIDANCE_SIDESTEP_M = 2.0 * 0.3048
BYPASS_FORWARD_M     = 3.0 * 0.3048
DRIVE_DISTANCE_M     = 10.0 * 0.3048
SPEED_MPS          = 0.8 * 0.44704    # 0.8 mph

# Compass/Turn Constants
TURN_RATE_DEG_S    = 10.0
STOP_EARLY_DEG     = 8.0
STABLE_COUNT_REQ   = 2
HEADING_INTERVAL_S = 0.2

# ── Signal handler ────────────────────────────────────────────────────────────
_abort = False
def _sigint(sig, frame):
    global _abort
    _abort = True
signal.signal(signal.SIGINT,  _sigint)
signal.signal(signal.SIGTERM, _sigint)

# ── Helper Classes ────────────────────────────────────────────────────────────

class MultiSensorDetector:
    FRAME_HEADER = 0x59

    def __init__(self, lidar_port, model_path):
        self.lidar_ser = serial.Serial(
            port=lidar_port, baudrate=LIDAR_BAUD,
            bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE, timeout=0.1
        )
        self.lidar_recent = []

        # Ultrasonic
        try:
            self.us_sensor = DistanceSensor(
                echo=ULTRASONIC_ECHO, trigger=ULTRASONIC_TRIG,
                max_distance=5.0, pin_factory=_pin_factory
            )
        except Exception as e:
            print(f"[WARN] Ultrasonic init failed: {e}")
            self.us_sensor = None

        # Camera + Akida
        self.cam_pipeline = None
        self.akida_model = None
        self.cam_q = None
        if dai and Model and os.path.exists(model_path):
            try:
                self.akida_model = Model(model_path)
                devs = devices()
                if devs:
                    self.akida_model.map(devs[0])
                    print(f"Akida BrainChip mapped [OK]")
                    self._setup_camera()
            except Exception as e:
                print(f"[WARN] Akida/Camera init failed: {e}")
        else:
            reason = ""
            if not dai: reason += "depthai missing, "
            if not Model: reason += "akida missing, "
            if not os.path.exists(model_path): reason += f"model not found at {model_path}"
            print(f"[WARN] Camera AI disabled: {reason}")

    def _setup_camera(self):
        self.cam_pipeline = dai.Pipeline()
        cam = self.cam_pipeline.create(dai.node.ColorCamera)
        cam.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
        cam.setInterleaved(False)
        cam.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)
        cam.setFps(15)

        xout = self.cam_pipeline.create(dai.node.XLinkOut)
        xout.setStreamName("rgb")
        cam.video.link(xout.input)

        device = dai.Device(self.cam_pipeline)
        self.cam_q = device.getOutputQueue(name="rgb", maxSize=4, blocking=False)
        print("Oak-D Lite Camera ready [OK]")

    def read_lidar(self):
        self.lidar_ser.reset_input_buffer()
        for _ in range(18):
            b1 = self.lidar_ser.read(1)
            if not b1 or b1[0] != self.FRAME_HEADER: continue
            b2 = self.lidar_ser.read(1)
            if not b2 or b2[0] != self.FRAME_HEADER: continue
            payload = self.lidar_ser.read(7)
            if len(payload) < 7: break
            dist_l, dist_h, _, _, _, conf, _ = payload
            if conf < 10: continue
            dist = (dist_h << 8) | dist_l
            return dist / 100.0
        return 9999.0

    def check_all(self):
        """Returns (triggered_bool, reason_string, distance_val)"""
        # 1. Lidar check (Filtered)
        l_raw = self.read_lidar()
        if l_raw < 9999.0:
            self.lidar_recent.append(l_raw)
            if len(self.lidar_recent) > 3: self.lidar_recent.pop(0)
        l_dist = min(self.lidar_recent) if self.lidar_recent else 9999.0
        if l_dist < OBSTACLE_THRESHOLD_M:
            return True, "LIDAR", l_dist

        # 2. Ultrasonic check
        if self.us_sensor:
            us_dist = self.us_sensor.distance
            if us_dist < OBSTACLE_THRESHOLD_M:
                return True, "ULTRASONIC", us_dist

        # 3. Camera check (Akida inference on middle third)
        if self.akida_model and self.cam_q and self.cam_q.has():
            import cv2
            frame = self.cam_q.get().getCvFrame()
            # Mini-preprocess for Akida (224x224)
            img = cv2.resize(frame, (224, 224))
            img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
            out = self.akida_model.predict(img.reshape(1, 224, 224, 3))[0]
            # Check center columns of grid (assuming grid 28x28 or similar)
            # This is a simplified FOMO check
            grid_h, grid_w, classes = out.shape
            mid_start = int(grid_w * 0.33)
            mid_end = int(grid_w * 0.66)
            for gy in range(grid_h):
                for gx in range(mid_start, mid_end):
                    cls = int(np.argmax(out[gy, gx]))
                    if cls != 0 and out[gy, gx, cls] > CONF_THRESH:
                        return True, f"CAMERA ({cls})", 0.5 # Estimated distance

        return False, "CLEAR", 9999.0

    def close(self):
        self.lidar_ser.close()
        if self.us_sensor: self.us_sensor.close()

# ── Main ─────────────────────────────────────────────────────────────────────

def build_drive_msg(vehicle, throttle, yaw_rate_deg_s=0.0):
    return vehicle.message_factory.set_attitude_target_encode(
        0, 0, 0, 0xA3, [1.0, 0.0, 0.0, 0.0], 0.0, 0.0, math.radians(yaw_rate_deg_s), throttle
    )

def turn(vehicle, direction_sign, angle_deg):
    """direction_sign: -1 = left, 1 = right"""
    start = vehicle.heading
    stop_at = abs(angle_deg) - STOP_EARLY_DEG
    msg = build_drive_msg(vehicle, 0.0, direction_sign * TURN_RATE_DEG_S)
    stable = 0
    t_start = time.time()
    while not _abort:
        vehicle.send_mavlink(msg)
        time.sleep(HEADING_INTERVAL_S)
        delta = ((vehicle.heading - start + 540) % 360) - 180
        if direction_sign == 1: # Right
            if delta >= (stop_at - 5): stable += 1
            else: stable = 0
        else: # Left
            if delta <= -(stop_at - 5): stable += 1
            else: stable = 0
        if stable >= STABLE_COUNT_REQ or time.time() - t_start > 10: break
    msg_stop = build_drive_msg(vehicle, 0.0)
    for _ in range(5): vehicle.send_mavlink(msg_stop); time.sleep(0.1)

def main():
    print("=" * 52)
    print("  TEST 11 - UGV TRIPLE-SENSOR ARM & AVOID")
    print("  (Lidar + Ultrasonic + Oak-D Camera)")
    print("=" * 52)

    bridge = V2VBridge(ESP32_PORT); bridge.connect()
    
    # Init LEDs
    from gpiozero import LED
    try:
        green = LED(GREEN_LED_PIN, pin_factory=_pin_factory)
        red = LED(RED_LED_PIN, pin_factory=_pin_factory)
        green.off(); red.off()
    except: green = red = None

    # Init multi-sensor suite
    import numpy as np # Needed for Akida processing
    detector = MultiSensorDetector(LIDAR_PORT, MODEL_PATH)

    # Wait for UAV trigger
    print("Waiting for UAV Phase 11...")
    while not _abort:
        if bridge.latest_uav_heartbeat and bridge.latest_uav_heartbeat.mission_phase == 11:
            print("Command Received [OK]")
            break
        time.sleep(0.5)

    if _abort: bridge.stop(); return

    # Connect & Arm
    vehicle = connect(UGV_PORT, wait_ready=True, baud=UGV_BAUD)
    print("Arming UGV...")
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True
    while not vehicle.armed and not _abort: time.sleep(0.1)
    
    if green: green.on()
    print("UGV Driving with Triple-Sensor Avoidance [OK]")

    # Main drive loop
    start_t = time.time()
    try:
        while not _abort and (time.time() - start_t) < 60:
            # Check UAV stop
            if bridge.latest_uav_heartbeat and bridge.latest_uav_heartbeat.mission_phase != 11:
                print("UAV commanded stop."); break

            # Sensor fusion check
            triggered, reason, dist = detector.check_all()
            if triggered:
                print(f"!!! OBSTACLE detected by {reason} at {dist:.2f}m !!!")
                if green: green.off()
                if red: red.on()
                
                # U-Shape Bypass Maneuver
                print(" -> Executing Bypass...")
                turn(vehicle, -1, 90);  time.sleep(1) # Left
                # Move sideways
                sidestep_msg = build_drive_msg(vehicle, 0.5)
                ts = time.time()
                while time.time() - ts < 3: vehicle.send_mavlink(sidestep_msg); time.sleep(0.1)
                
                turn(vehicle, 1, 90); time.sleep(1) # Right
                # Move forward past obstacle
                bypass_msg = build_drive_msg(vehicle, 0.5)
                ts = time.time()
                while time.time() - ts < 5: vehicle.send_mavlink(bypass_msg); time.sleep(0.1)
                
                turn(vehicle, 1, 90); time.sleep(1) # Right
                # Move back to original line
                while time.time() - ts < 3: vehicle.send_mavlink(sidestep_msg); time.sleep(0.1)
                
                turn(vehicle, -1, 90); time.sleep(1) # Left
                
                if red: red.off()
                if green: green.on()
                print(" -> Path cleared, resuming.")

            else:
                # Drive forward
                vehicle.send_mavlink(build_drive_msg(vehicle, 0.5))
                time.sleep(0.1)
    finally:
        print("Finalizing...")
        vehicle.armed = False
        if green: green.off()
        if red: red.off()
        detector.close()
        bridge.stop()
        vehicle.close()
        print("Test 11 Complete [OK]")

if __name__ == "__main__":
    main()
