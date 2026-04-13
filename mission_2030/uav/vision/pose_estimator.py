import cv2
import numpy as np
from mission_2030.common.frames import camera_to_body_ned

class PoseEstimator:
    def __init__(self, camera_matrix, dist_coeffs):
        self.camera_matrix = camera_matrix
        self.dist_coeffs = dist_coeffs

    def estimate_pose_single_markers(self, corners, marker_size):
        marker_points = np.array([
            [-marker_size / 2, marker_size / 2, 0],
            [marker_size / 2, marker_size / 2, 0],
            [marker_size / 2, -marker_size / 2, 0],
            [-marker_size / 2, -marker_size / 2, 0]
        ], dtype=np.float32)

        rvecs = []
        tvecs = []
        for c in corners:
            success, rvec, tvec = cv2.solvePnP(
                marker_points, c, self.camera_matrix, self.dist_coeffs, flags=cv2.SOLVEPNP_IPPE_SQUARE)
            if success:
                rvecs.append(rvec)
                tvecs.append(tvec)
        return rvecs, tvecs

    def estimate_angles(self, tvec_body) -> tuple:
        """
        Given a body-NED tvec (x_fwd, y_right, z_down in metres),
        return (angle_x_rad, angle_y_rad, dist_m) for LANDING_TARGET.
        """
        import math
        x, y, z = tvec_body
        dist = math.sqrt(x**2 + y**2 + z**2)
        ax   = math.atan2(y, z)   # lateral angle (right positive)
        ay   = math.atan2(-x, z)  # longitudinal angle (forward negative)
        return ax, ay, dist
