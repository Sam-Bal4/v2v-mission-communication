import math
import cv2
import pyzed.sl as sl
from mission_2030.common.logging_utils import setup_logger

logger = setup_logger("UgvTracking")

class UgvTracking:
    def __init__(self, detector, marker_id):
        self.detector = detector
        self.marker_id = marker_id

    def extract_target_angles(self, frame_bgr, point_cloud: sl.Mat):
        """ Returns (angle_x, angle_y, true_dist_z) for MAVLink landing_target_send. """
        gray = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = self.detector.detect(gray)
        
        if ids is not None and self.marker_id in ids:
            idx = list(ids.flatten()).index(self.marker_id)
            c = corners[idx][0]
            cx = int((c[0][0] + c[2][0]) / 2.0)
            cy = int((c[0][1] + c[2][1]) / 2.0)
            
            err, point3D = point_cloud.get_value(cx, cy)
            if err == sl.ERROR_CODE.SUCCESS:
                x, y, z = point3D[0], point3D[1], point3D[2]
                
                if math.isnan(x) or math.isnan(y) or math.isnan(z):
                    logger.warning(f"Marker {self.marker_id} detected but ZED depth returned NaN.")
                    return None
                
                logger.info(f"Marker {self.marker_id} localized at X:{x:.2f}m, Y:{y:.2f}m, Depth Z:{z:.2f}m")
                
                # Convert physical coordinates back to precise camera-frame angles for MAVLink
                angle_x = math.atan2(x, z)
                angle_y = math.atan2(y, z)
                
                return angle_x, angle_y, z
                
        return None
