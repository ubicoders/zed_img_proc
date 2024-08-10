import cv2
import time
from .zed_utils import ZEDCam
from .aruco_utils import ArucoDetector
import numpy as np

class TimerTicTok:
    def __init__(self):
        self.prev = time.time()
    
    def update(self):
        now = time.time()
        self.elapse = now - self.prev
        self.prev = now
    
    def pprint(self):
        # print up to 2 decimal places
        print(f"dt = {self.elapse:.2f} sec, freq = {1.0/self.elapse:.2f}")


def calculate_XYZ_zed(marker_xy, point_cloud):
    N = len(marker_xy)
    result = []
    for i in range(0, N):
        marker_id, mx, my = marker_xy[i]
        _, xyz_mm = point_cloud.get_value(mx, my)
        zed_xyz_mm = np.array([xyz_mm[0], xyz_mm[1], xyz_mm[2]])
        result.append(
            {
                "m_id": marker_id,
                "xy": [mx, my],
                "zed_xyz_mm": zed_xyz_mm
            }
        )
    return result

def calculate_depth_zed(marker_xy, depth_map):
    N = len(marker_xy)
    result = []
    for i in range(0, N):
        marker_id, mx, my = marker_xy[i]
        zed_depth_mm = depth_map.get_value(mx, my)
        result.append(
            {
                "m_id": marker_id,
                "xy": [mx, my],
                "zed_depth_mm": zed_depth_mm
            }
        )
    return result

def main():
    # Create a Camera object
    cam = ZEDCam(has_depth=False)
    cam.open_cam()

    # Create an ArucoDetector object
    aruco_left = ArucoDetector()
    aruco_right = ArucoDetector(is_left=False)
    
    key = ''
    timer = TimerTicTok()
    while key != 27 and key != ord('q'):  # Press 'Esc' to exit
        if not cam.update_cam(left=True, right=True, depth=False, point_cloud=False):
            print("camera error")
            break

        left, right = cam.get_bgr_images()
        
        # Detect ArUco markers in the left image
        aruco_left.detect_bgr(left, estimate_pose=False)
        ids, marker_xy_list, tvecs = aruco_left.get_xy_XYZ_info()
        left = aruco_left.drawMarkers(left)
        if (tvecs is not None) :
            print(f"{tvecs}")

        # Detect ArUco markers in the right image
        aruco_right.detect_bgr(right, estimate_pose=False)
        ids, marker_xy_list, tvecs = aruco_right.get_xy_XYZ_info()
        right = aruco_right.drawMarkers(right)
        if (tvecs is not None) :
            print(f"{tvecs}")


        # # depth
        # depth_map = cam.get_depth_map()
        # if (ids is not None) :
        #     m_info = calculate_depth_zed(marker_xy_list, depth_map)
        #     print(f"{m_info}")
        # point_cloud = cam.get_point_cloud()
        # if (ids is not None) :
        #     m_info = calculate_XYZ_zed(marker_xy_list, point_cloud)
        #     print(f"{m_info}")

        # Display the images  
        cv2.imshow("Left Image", left)
        cv2.imshow("Right Image", right)

        timer.update()
        timer.pprint()        

        # Wait for a key press
        key = cv2.waitKey(1)

    # Close the camera
    cam.close_cam()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()