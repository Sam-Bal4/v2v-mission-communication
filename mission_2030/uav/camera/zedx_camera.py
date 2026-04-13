import cv2
import pyzed.sl as sl
from mission_2030.common.logging_utils import setup_logger

logger = setup_logger("ZedCamera")

class ZedXCamera:
    def __init__(self):
        self.cam = sl.Camera()
        init_params = sl.InitParameters()
        init_params.camera_resolution = sl.RESOLUTION.HD720  
        init_params.camera_fps = 30 
        init_params.depth_mode = sl.DEPTH_MODE.PERFORMANCE 
        init_params.coordinate_units = sl.UNIT.METER 

        status = self.cam.open(init_params)
        if status != sl.ERROR_CODE.SUCCESS:
            logger.error(f"ZED Camera open failed: {status}")
            raise Exception("Cannot open ZED camera.")
            
        self.zed_image = sl.Mat()
        self.point_cloud = sl.Mat()

    def get_frame(self):
        """ Returns (bgr_frame, point_cloud_mat) if successful, else (None, None) """
        if self.cam.grab() == sl.ERROR_CODE.SUCCESS:
            self.cam.retrieve_image(self.zed_image, sl.VIEW.LEFT)
            self.cam.retrieve_measure(self.point_cloud, sl.MEASURE.XYZ)
            frame_bgra = self.zed_image.get_data()
            frame_bgr = cv2.cvtColor(frame_bgra, cv2.COLOR_BGRA2BGR)
            return frame_bgr, self.point_cloud
        return None, None

    def release(self):
        self.cam.close()
        logger.info("ZED camera closed.")
