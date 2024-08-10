import cv2
import pyzed.sl as sl
import numpy as np
import PIL.Image as Image
import cv2.aruco as aruco
import time
from .zed_utils import ZEDCam
from .aruco_utils import ArucoDetector

def main():
    # Create a Camera object
    cam = ZEDCam()
    cam.open_cam()

    # Create an ArucoDetector object
    aruco_detector = ArucoDetector()
    
    key = ''
    prev = time.time()
    while key != 27:  # Press 'Esc' to exit
        if not cam.update_cam():
            print("camera error")
            break

        left, right = cam.get_bgr_images()
        
        corners, ids, rejectedImgPoints = aruco_detector.detect_bgr(left)
        left = aruco_detector.drawMarkers(left, corners)

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