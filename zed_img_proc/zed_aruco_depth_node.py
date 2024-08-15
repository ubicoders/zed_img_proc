import cv2
import rclpy
from rclpy.node import Node
import numpy as np
from .zed_utils import ZEDCam
from .aruco2d_utils import ArucoDetector
from .perf_utils import TimerTicTok
from aruco_interface.msg import ImageMarkers
from .aruco_msgpack import pack_aruco
from .body_msgpack import pack_body
from skeleton_interface.msg import BodyKeyPoints

ZED_SERIAL_NUMBER = 17437

class ZedArucoNode(Node):
    def __init__(self):
        super().__init__('zed_aruco_with_depth')
        # Create a Camera object
        self.cam = ZEDCam(body_track=False, serial_number=ZED_SERIAL_NUMBER)
        self.cam.open_cam()

        # Create ArucoDetector objects
        self.aruco_left = ArucoDetector()
        self.aruco_right = ArucoDetector()

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

        left = self.cam.get_bgr_left()
        point_cloud = self.cam.get_point_cloud()
        

        # Detect ArUco markers in the left image
        self.aruco_left.detect_bgr(left)
        self.aruco_left.update_center()
        for mid in self.aruco_left.get_ids():
            center = self.aruco_left.idCenterMap[mid]
            self.get_logger().info(f"Marker {mid} center: {center}")
            _, xyz = point_cloud.get_value(int(center[0]), int(center[1]))
            xyz = xyz[ :-1]
            self.aruco_left.update_idXYZ(mid, xyz)
        
        # pack and publish all
        self.pub_aruco.publish(pack_aruco(ZED_SERIAL_NUMBER, self.aruco_left.idCornerMap, {}, idXYZMap=self.aruco_left.idXYZMap))

        ##=========================================================================================================
        # # # left plot
        # left = self.aruco_left.drawMarkers(left)


        # # # Optionally, display the images for debugging
        # cv2.imshow("Left Image", left)
        # cv2.waitKey(1)

        self.tictok.update()
        self.tictok.pprint()


def main(args=None):
    rclpy.init(args=args)
    node = ZedArucoNode()
    rclpy.spin(node)
    node.cam.close_cam()
    node.destroy_node()
    rclpy.shutdown()
    


if __name__ == "__main__":
    main()
