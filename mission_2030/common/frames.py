import numpy as np

def camera_to_body_ned(x_cam: float, y_cam: float, z_cam: float) -> tuple[float, float, float]:
    """
    Transforms ZED downward camera frame (OpenCV standard: Z out, Y down, X right) 
    to ArduPilot Body NED (X forward, Y right, Z down).
    Assumes camera is mounted perfectly downward facing.
    """
    # X_body = -Y_cam (assuming camera top is pointing forward)
    # Y_body = X_cam
    # Z_body = Z_cam
    # TODO: Verify extrinsics against physical mount
    return (-y_cam, x_cam, z_cam)
