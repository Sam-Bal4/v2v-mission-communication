import cv2

class ArucoDetectorShim:
    """
    A backward-compatible shim for OpenCV's ArUco detection.
    OpenCV 4.7+ uses cv2.aruco.ArucoDetector.
    OpenCV 4.6- (often standard on Jetson/Ubuntu apt) uses cv2.aruco.detectMarkers.
    """
    def __init__(self, dictionary, parameters=None):
        self.dictionary = dictionary
        self.parameters = parameters if parameters is not None else cv2.aruco.DetectorParameters()
        
        self.new_api = hasattr(cv2.aruco, 'ArucoDetector')
        if self.new_api:
            self.detector = cv2.aruco.ArucoDetector(self.dictionary, self.parameters)

    def detectMarkers(self, image):
        if self.new_api:
            return self.detector.detectMarkers(image)
        else:
            if hasattr(cv2.aruco, "detectMarkers"):
                # Handle old API style
                corners, ids, rejected = cv2.aruco.detectMarkers(
                    image, 
                    self.dictionary, 
                    parameters=self.parameters
                )
                return corners, ids, rejected
            else:
                raise RuntimeError("Could not find any ArUco detection method in this cv2 build.")
