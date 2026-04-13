import time
import collections

# Compatibility patch for dronekit on Python 3.10+
if not hasattr(collections, 'MutableMapping'):
    import collections.abc
    collections.MutableMapping = collections.abc.MutableMapping

from pymavlink import mavutil
from dronekit import VehicleMode
from mission_2030.common.logging_utils import setup_logger

logger = setup_logger("MavlinkUtils")

def arm_vehicle(master, timeout=15) -> bool:
    """
    Robust arming function with COMMAND_ACK and heartbeat filtering.
    """
    # Drain buffer first
    while master.recv_match(blocking=False):
        pass

    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0, 1, 0, 0, 0, 0, 0, 0
    )
    
    start_t = time.time()
    while time.time() - start_t < timeout:
        msg = master.recv_match(type=['HEARTBEAT', 'COMMAND_ACK', 'RC_CHANNELS'], blocking=True, timeout=1.0)
        if not msg:
            continue
        
        if msg.get_srcSystem() != master.target_system:
            continue

        if msg.get_type() == 'COMMAND_ACK':
            if msg.command == mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM:
                if msg.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
                    logger.info("Arm command accepted ✓")
                elif msg.result == mavutil.mavlink.MAV_RESULT_TEMPORARILY_REJECTED:
                    # Specific check for RC7 switch state based on user parameters
                    rc_msg = master.recv_match(type='RC_CHANNELS', blocking=True, timeout=0.5)
                    if rc_msg and hasattr(rc_msg, 'chan7_raw') and rc_msg.chan7_raw < 1200:
                         logger.error("Arm REJECTED: Check physical RC7 Safety Switch!")
                    else:
                         logger.error("Arm REJECTED: Check Pre-arm (check GCS/Lidar/Flow)")
                    return False
        
        if msg.get_type() == 'HEARTBEAT':
            if msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED:
                logger.info("Vehicle Armed successfully ✓")
                return True
                
    logger.error("Failed to arm within timeout.")
    return False

def wait_disarm(master, timeout_s=90) -> bool:
    """
    Waits for the flight controller to confirm disarmed state.
    """
    logger.info(f"Waiting for disarm confirmation (max {timeout_s}s)...")
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        hb = master.recv_match(type='HEARTBEAT', blocking=True, timeout=2.0)
        if hb and hb.get_srcSystem() == master.target_system:
            if not (hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
                logger.info("Disarm confirmed by FC ✓")
                return True
    logger.warning("Disarm confirmation timeout.")
    return False

def is_vehicle_disarmed(master) -> bool:
    """
    Non-blocking check for disarmed state. Drains the buffer and looks at the freshest heartbeat.
    """
    hb = None
    while True:
        m = master.recv_match(type='HEARTBEAT', blocking=False)
        if m is None: break
        if m.get_srcSystem() == master.target_system:
            hb = m
    if hb:
        return not bool(hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
    return False

def arm_vehicle_dronekit(vehicle, mode_name="GUIDED", timeout=15):
    """
    Robust arming for DroneKit vehicles.
    """
    logger.info(f"Arming {vehicle} into {mode_name}...")
    
    # Switch mode if needed
    if vehicle.mode.name != mode_name:
        vehicle.mode = VehicleMode(mode_name)
        t0 = time.time()
        while vehicle.mode.name != mode_name and time.time() - t0 < 5:
            time.sleep(0.1)
    
    # Command arm
    vehicle.armed = True
    t0 = time.time()
    while not vehicle.armed and time.time() - t0 < timeout:
        time.sleep(0.1)
    
    if not vehicle.armed:
        logger.error("Failed to arm vehicle (DroneKit).")
        return False
        
    logger.info("Vehicle Armed (DroneKit) ✓")
    return True

def get_lidar_alt(master) -> float | None:
    """Drains DISTANCE_SENSOR messages and returns the latest reading in meters."""
    alt = None
    while True:
        msg = master.recv_match(type='DISTANCE_SENSOR', blocking=False)
        if msg is None: break
        if msg.current_distance > 0:
            alt = msg.current_distance / 100.0
    return alt
