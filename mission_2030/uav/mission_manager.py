import time
from operation_touchdown.uav.state_machine import UavState
from operation_touchdown.uav.mavlink_client import MavlinkClient
from operation_touchdown.radio.v2v_bridge import V2VBridge
from operation_touchdown.common.logging_utils import setup_logger

logger = setup_logger("UAV_MissionManager")

class MissionManager:
    def __init__(self, bridge: V2VBridge, mavlink: MavlinkClient):
        self.bridge = bridge
        self.mavlink = mavlink
        self.state = UavState.BOOT

    def set_state(self, new_state: UavState):
        logger.info(f"Transition: {self.state.name} -> {new_state.name}")
        self.state = new_state

    def run(self):
        while self.state != UavState.MISSION_COMPLETE:
            if self.state == UavState.COMM_LOSS:
                 break
            # Add state machine tick logic
            time.sleep(0.1)
