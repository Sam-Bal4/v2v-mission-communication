import time
from mission_2030.ugv.state_machine import UgvState
from mission_2030.radio.v2v_bridge import V2VBridge
from mission_2030.common.logging_utils import setup_logger

logger = setup_logger("UGV_MissionManager")

class MissionManager:
    def __init__(self, bridge: V2VBridge):
        self.bridge = bridge
        self.state = UgvState.BOOT

    def set_state(self, new_state: UgvState):
        logger.info(f"Transition: {self.state.name} -> {new_state.name}")
        self.state = new_state

    def run(self):
        while self.state != UgvState.MISSION_COMPLETE:
            if self.state == UgvState.ESTOP:
                break
            time.sleep(0.1)
