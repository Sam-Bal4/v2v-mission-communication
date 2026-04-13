import time
from mission_2030.radio.v2v_bridge import V2VBridge

class TelemetrySender:
    def __init__(self, bridge: V2VBridge):
        self.bridge = bridge
        self.seq = 0

    def push_state(self, phase_val: int, is_estop: bool = False):
        """Broadcast UGV phase to the UAV via the ESP32 radio bridge."""
        self.seq += 1
        self.bridge.send_ugv_heartbeat(
            self.seq,
            (int(time.time() * 1000) & 0xFFFFFFFF),
            phase_val,
            is_estop
        )

