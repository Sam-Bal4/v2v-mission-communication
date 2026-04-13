from mission_2030.common.logging_utils import setup_logger

logger = setup_logger("Failsafes")

class FailsafeController:
    def __init__(self, control):
        self.control = control

    def trigger_comm_loss(self):
        logger.error("COMM_LOSS TRIGGERED: Holding position.")
        # E.g. trigger LOITER or LAND based on rules
        self.control.change_mode("LOITER")

    def trigger_target_loss(self):
        logger.error("TARGET_LOSS TRIGGERED: Climbing to search alt.")
        self.control.change_mode("GUIDED")
        self.control.set_velocity_body(0, 0, -0.2) # Climb
