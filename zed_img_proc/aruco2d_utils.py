import cv2.aruco as aruco
import cv2
import numpy as np
import os

  
class ArucoDetector:
    def __init__(self) -> None:
        base_path = os.path.dirname(os.path.abspath(__file__))
        aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_1000)
        arucoParameters = aruco.DetectorParameters()
        self.detector = aruco.ArucoDetector(aruco_dict, arucoParameters)

        self.corners = None
        self.ids = None
        self.idCornerMap = {}

    def detect_bgr(self, bgr_img):
        gray_img = cv2.cvtColor(bgr_img, cv2.COLOR_BGR2GRAY)
        self.detect_gray(gray_img)

    def detect_gray(self, gray_img):
        # corners, ids, rejectedImgPoints
        self.corners, self.ids, _ = self.detector.detectMarkers(gray_img)
        if self.ids is None or len(self.ids) == 0:
            self.idCornerMap = {}
            return
        # make a map of ids to corners
        for i, marker_idx in enumerate(self.ids):
            mid = marker_idx[0]
            corners = np.reshape(self.corners[i], (4, 2))
            self.idCornerMap[mid] = corners

    def drawMarkers(self, img):
        if (self.corners is None) or (self.ids is None):
            return img
        img = aruco.drawDetectedMarkers(img, self.corners)
        return img
    
