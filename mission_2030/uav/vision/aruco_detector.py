import cv2

class ArucoDetector:
    def __init__(self, dict_type=cv2.aruco.DICT_4X4_50):
        self.dictionary = cv2.aruco.getPredefinedDictionary(dict_type)
        self.parameters = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(self.dictionary, self.parameters)

    def detect(self, image):
        # returns corners, ids, rejected
        return self.detector.detectMarkers(image)
