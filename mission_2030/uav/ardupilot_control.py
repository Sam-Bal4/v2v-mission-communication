import time
from pymavlink import mavutil
from mission_2030.common.logging_utils import setup_logger

logger = setup_logger("ArdupilotControl")

class ArdupilotControl:
    def __init__(self, master):
        self.master = master

    def change_mode(self, mode: str) -> bool:
        mapping = self.master.mode_mapping()
        if mode not in mapping:
            logger.error(f"Mode {mode} not available in ArduCopter.")
            return False
        
        mode_id = mapping[mode]
        self.master.mav.set_mode_send(
            self.master.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mode_id
        )
        logger.info(f"Requested mode switch to {mode}")
        return True

    def arm_vehicle(self) -> bool:
        logger.info("Attempting to Arm...")
        self.master.mav.command_long_send(
            self.master.target_system, self.master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0, 1, 0, 0, 0, 0, 0, 0
        )
        # Wait for ARM confirm
        start_t = time.time()
        while time.time() - start_t < 10:
            msg = self.master.recv_match(type='HEARTBEAT', blocking=True, timeout=1.0)
            if msg and (msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
                logger.info("Vehicle Armed successfully.")
                return True
        logger.error("Failed to arm within timeout.")
        return False

    def disarm_vehicle(self):
        logger.info("Disarming...")
        self.master.mav.command_long_send(
            self.master.target_system, self.master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0, 0, 0, 0, 0, 0, 0, 0
        )

    def set_velocity_body(self, vx: float, vy: float, vz: float):
        self.master.mav.set_position_target_local_ned_send(
            0, self.master.target_system, self.master.target_component,
            mavutil.mavlink.MAV_FRAME_BODY_NED, 
            0b0000111111000111, # Use velocity ONLY
            0, 0, 0, 
            vx, vy, vz, 
            0, 0, 0, 0, 0
        )
