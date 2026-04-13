import numpy as np
from mission_2030.common.logging_utils import setup_logger

logger = setup_logger("DestinationDetection")

class DestinationDetection:
    def __init__(self, detector):
        self.detector = detector

    def process_frame(self, frame):
        """ Returns (marker_id, cx, cy) if found """
        corners, ids, _ = self.detector.detect(frame)
        if ids is not None:
            for idx, marker_id in enumerate(ids.flatten()):
                if 0 <= marker_id <= 4:
                    c = corners[idx][0]
                    return marker_id, np.mean(c[:, 0]), np.mean(c[:, 1])
        return None
