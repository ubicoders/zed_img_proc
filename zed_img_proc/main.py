import cv2
import pyzed.sl as sl
import numpy as np
import PIL.Image as Image
import cv2.aruco as aruco
import time
from .zed_utils import ZEDCam

aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
arucoParameters = aruco.DetectorParameters()
detector = aruco.ArucoDetector(aruco_dict, arucoParameters)

def main():
    # Create a Camera object
    cam = ZEDCam()
    cam.open_cam()
    
    key = ''
    prev = time.time()
    while key != 27:  # Press 'Esc' to exit
        if not cam.update_cam():
            print("camera error")
            break

        left, right = cam.get_bgr_images()
        
        gray_left = cv2.cvtColor(left, cv2.COLOR_BGRA2GRAY)

        # aruco detection
        corners, ids, rejectedImgPoints = detector.detectMarkers(gray_left)
        gray_left = aruco.drawDetectedMarkers(left, corners)        

        # Display the images
        cv2.imshow("Left Image", left)
        # cv2.imshow("Right Image", right_image_ocv)

        now = time.time()
        elapse = now - prev
        prev = now
        print(f"dt = elapsed {elapse}, freq = {1.0/elapse}")
        

        # Wait for a key press
        key = cv2.waitKey(1)

    # Close the camera
    cam.close_cam()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()