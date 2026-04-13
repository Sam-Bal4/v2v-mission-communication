import time
import serial
from mission_2030.common.logging_utils import setup_logger

try:
    from gpiozero import LED
    from gpiozero.pins.lgpio import LGPIOFactory
    _pin_factory = LGPIOFactory()
except ImportError:
    LED = None
    _pin_factory = None

logger = setup_logger("ObstacleAvoidance")

class ObstacleAvoidance:
    FRAME_HEADER = 0x59
    LIDAR_MIN_CONFIDENCE = 10
    LIDAR_NO_TARGET_M = 9999.0

    def __init__(self, port="/dev/ttyAMA0", baud=115200, green_pin=16, red_pin=19):
        self.avoiding = False
        self.obstacle_clear_time = 0.0
        
        # Initialize LEDs
        if LED:
            kwargs = {"pin_factory": _pin_factory} if _pin_factory else {}
            self.green = LED(green_pin, **kwargs)
            self.red = LED(red_pin, **kwargs)
            self.green.on()
            self.red.off()
        else:
            self.green = None
            self.red = None
            
        # Initialize TF-Nova serial
        try:
            self.ser = serial.Serial(
                port=port, baudrate=baud, bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE,
                timeout=0.1
            )
            self.ser.reset_input_buffer()
            logger.info(f"TF-Nova lidar opened on {port}")
        except Exception as e:
            logger.error(f"Failed to open Lidar UART: {e}")
            self.ser = None

    def read_lidar(self) -> float:
        """ Returns distance in meters directly sourced from the TF-Nova buffer """
        if not self.ser: return self.LIDAR_NO_TARGET_M
        
        self.ser.reset_input_buffer()
        for _ in range(18):
            b1 = self.ser.read(1)
            if not b1 or b1[0] != self.FRAME_HEADER: continue

            b2 = self.ser.read(1)
            if not b2 or b2[0] != self.FRAME_HEADER: continue

            payload = self.ser.read(7)
            if len(payload) < 7: return self.LIDAR_NO_TARGET_M

            dist_l, dist_h, peak_l, peak_h, temp, confidence, checksum = payload
            raw = [self.FRAME_HEADER, self.FRAME_HEADER, dist_l, dist_h, peak_l, peak_h, temp, confidence]
            
            if (sum(raw) & 0xFF) != checksum: continue

            distance_cm = (dist_h << 8) | dist_l
            if confidence < self.LIDAR_MIN_CONFIDENCE or distance_cm == 0:
                return self.LIDAR_NO_TARGET_M

            return distance_cm / 100.0
            
        return self.LIDAR_NO_TARGET_M

    def led_obstacle(self):
        if self.green: self.green.off()
        if self.red: self.red.on()
        
    def led_clear(self):
        if self.red: self.red.off()
        if self.green: self.green.on()

    def check_trigger(self, threshold_m=1.5, manual_duration_s=3.0) -> bool:
        dist = self.read_lidar()
        if dist < threshold_m:
            self.avoiding = True
            self.obstacle_clear_time = time.time() + manual_duration_s
            logger.warning(f"Obstacle detected at {dist:.2f}m. Avoidance maneuver triggered.")
            self.led_obstacle()
            return True
        return False

    def is_clear(self) -> bool:
        if self.avoiding and time.time() > self.obstacle_clear_time:
            self.avoiding = False
            logger.info("Obstacle bypassed. LEDs cleared.")
            self.led_clear()
        return not self.avoiding
