import cv2
import rclpy
from rclpy.node import Node
import numpy as np
from .zed_utils import ZEDCam
from .aruco_utils import ArucoDetector
from .perf_utils import TimerTicTok
from aruco_interface.msg import ImageMarkers
from .aruco_msgpack import pack_aruco
from .body_msgpack import pack_body
from skeleton_interface.msg import BodyKeyPoints
from .zed_params import left_params, right_params

ZED_SERIAL_NUMBER = 17437

class ZedArucoNode(Node):
    def __init__(self):
        super().__init__('zed_aruco_with_depth')

        self.stereo_on = True

        # Create a Camera object
        self.cam = ZEDCam(body_track=False, serial_number=ZED_SERIAL_NUMBER)
        self.cam.open_cam()

        # Create ArucoDetector objects
        self.aruco_left = ArucoDetector(pose_on=True)
        self.aruco_left.camera_matrix = left_params.get_camera_matrix()
        self.aruco_left.dist_coeffs = left_params.get_dist_coeffs()
        
        if self.stereo_on:
            self.aruco_right = ArucoDetector(pose_on=True)
            self.aruco_right.camera_matrix = right_params.get_camera_matrix()
            self.aruco_right.dist_coeffs = right_params.get_dist_coeffs()

        # tic-tok timer
        self.tictok = TimerTicTok()

        # ROS 2 Publisher for 3D vector arrays
        self.pub_aruco = self.create_publisher(ImageMarkers, '/cam/aruco_depth', 10)

        # Timer for periodic callback
        self.timer = self.create_timer(0.01, self.timer_callback)

    def timer_callback(self):
        if not self.cam.update_cam():
            self.get_logger().error("Camera error")
            return

        self.tictok.update()
        self.tictok.pprint()

        # Get the point cloud
        point_cloud = self.cam.get_point_cloud()


        # process left image
        left = self.cam.get_bgr_left()
        self.aruco_left.detect_bgr(left)
        self.aruco_left.update_center()
        num_markers = len(self.aruco_left.get_all_ids())
        # Get the 3D coordinates of the markers
        for mid in self.aruco_left.get_all_ids():
            center = self.aruco_left.aruco_info[mid]["center"]
            _, xyz = point_cloud.get_value(int(center[0]), int(center[1]))
            xyz = self.cam.clean_xyz(xyz)
            self.aruco_left.update_idXYZ(mid, xyz)
            #print(f"Marker {mid} all: {self.aruco_left.aruco_info[mid]}")

        if (num_markers == 0):
            return
        else:
            self.get_logger().info(f"Detected {num_markers} markers in the left image")

        # process right image
        if (self.stereo_on):
            right = self.cam.get_bgr_right()
            self.aruco_right.detect_bgr(right)
            self.aruco_right.update_center()
            num_markers = len(self.aruco_right.get_all_ids())
            # Get the 3D coordinates of the markers
            for mid in self.aruco_right.get_all_ids():
                center = self.aruco_right.aruco_info[mid]["center"]
                _, xyz = point_cloud.get_value(int(center[0]), int(center[1]))
                xyz = self.cam.clean_xyz(xyz)
                self.aruco_right.update_idXYZ(mid, xyz)
                #print(f"Marker {mid} all: {self.aruco_right.aruco_info[mid]}")

         # pack and publish all
        if self.stereo_on:
            self.pub_aruco.publish(pack_aruco(ZED_SERIAL_NUMBER, self.aruco_left.aruco_info, self.aruco_right.aruco_info))
        else:
            self.pub_aruco.publish(pack_aruco(ZED_SERIAL_NUMBER, self.aruco_left.aruco_info, {}))

        ##=========================================================================================================
        # # # left plot
        # left = self.aruco_left.drawMarkers(left)

        # # # Optionally, display the images for debugging
        # cv2.imshow("Left Image", left)
        # cv2.waitKey(1)

        


def main(args=None):
    rclpy.init(args=args)
    node = ZedArucoNode()
    rclpy.spin(node)
    node.cam.close_cam()
    node.destroy_node()
    rclpy.shutdown()
    


if __name__ == "__main__":
    main()
