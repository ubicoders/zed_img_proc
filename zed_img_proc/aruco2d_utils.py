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
        self.idCenterMap = {}
        self.idXYZMap = {}

    def detect_bgr(self, bgr_img):
        gray_img = cv2.cvtColor(bgr_img, cv2.COLOR_BGR2GRAY)
        self.detect_gray(gray_img)

    def detect_gray(self, gray_img):
        self.idCornerMap = {}
        # corners, ids, rejectedImgPoints
        self.corners, self.ids, _ = self.detector.detectMarkers(gray_img)
        if self.ids is None or len(self.ids) == 0:
            return
        # make a map of ids to corners
        for i, marker_idx in enumerate(self.ids):
            mid = marker_idx[0]
            if mid not in [0, 1, 2, 3, 4, 5]:
                continue
            corners = np.reshape(self.corners[i], (4, 2))
            self.idCornerMap[mid] = corners

    def drawMarkers(self, img):
        if (self.corners is None) or (self.ids is None):
            return img
        img = aruco.drawDetectedMarkers(img, self.corners)
        return img
    
    def update_idXYZ(self, mid, xyz):
        self.idXYZMap[mid] = xyz
    
    def get_ids(self):
        return self.idCenterMap.keys()
    
    def update_center(self):
        all_ids = self.idCornerMap.keys()
        for mid in all_ids:
            center = self.get_center(mid)
            self.idCenterMap[mid] = center
       
    
    def get_center(self, mid):
        if mid in self.idCornerMap:
            corners = self.idCornerMap[mid]
            center = np.mean(corners, axis=0)
            return center
        return None