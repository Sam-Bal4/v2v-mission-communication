import cv2
import numpy as np
from operation_touchdown.common.frames import camera_to_body_ned

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
