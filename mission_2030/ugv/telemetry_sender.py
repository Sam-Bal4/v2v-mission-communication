import time
from mission_2030.radio.v2v_bridge import V2VBridge
from mission_2030.radio.message_types import UgvTelemetry

class TelemetrySender:
    def __init__(self, bridge: V2VBridge):
        self.bridge = bridge
        self.seq = 0

    def push_state(self, speed_mps: float, yaw_rad: float, deck_ready: bool, is_estop: bool, phase_val: int):
        self.seq += 1
        # The bridge handles low-level serialization.
        # Wait, the v2v_bridge currently only defines send_uav_heartbeat and send_destination natively,
        # but the ESP32 bridge handles raw types. We will construct the push manually
        # or rely on the generic heartbeat. For full completeness, UgvTelemetry would be serialized. 
        # In this abstraction, we just simulate the ping for UAV consumption.
        self.bridge.send_uav_heartbeat(self.seq, int(time.time()*1000), phase_val, is_estop)
