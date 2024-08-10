import cv2.aruco as aruco
import cv2

class ArucoDetector:
    def __init__(self) -> None:
        aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
        arucoParameters = aruco.DetectorParameters()
        self.detector = aruco.ArucoDetector(aruco_dict, arucoParameters)

    def detect_bgr(self, bgr_img):
        gray_img = cv2.cvtColor(bgr_img, cv2.COLOR_BGR2GRAY)
        return self.detect_gray(gray_img)

    def detect_gray(self, gray_img):
        # return corners, ids, rejectedImgPoints
        return self.detector.detectMarkers(gray_img)
        
    def drawMarkers(self, img, corners):
        return aruco.drawDetectedMarkers(img, corners)