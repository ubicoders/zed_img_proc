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


class ZED_SEKELETAL_NODE(Node):
    def __init__(self):
        super().__init__('zed_node')
        # Create a Camera object
        self.cam = ZEDCam(body_track=True , serial_number=14100309)
        self.cam.open_cam()

        # Create ArucoDetector objects
        self.aruco_left = ArucoDetector()
        self.aruco_right = ArucoDetector()

        # tic-tok timer
        self.tictok = TimerTicTok()

        # ROS 2 Publisher for 3D vector arrays
        self.pub_left_aruco = self.create_publisher(ImageMarkers, '/left/arcuo', 10)
        self.pub_right_aruco = self.create_publisher(ImageMarkers, '/right/arcuo', 10)
        self.pub_body = self.create_publisher(BodyKeyPoints, '/body_keypoints', 10)

        # Timer for periodic callback
        self.timer = self.create_timer(0.01, self.timer_callback)

    def timer_callback(self):
        if not self.cam.update_cam():
            self.get_logger().error("Camera error")
            return

        # body tracking
        bodies = self.cam.get_body_tracking()
        
        for body in bodies.body_list:
            body_msg = pack_body(body)
            self.pub_body.publish(body_msg)
            # only 1 body for now
            break

        ##=========================================================================================================
        # # left plot
        #left = self.aruco_left.drawMarkers(left)

        # # right plot
        # right = self.aruco_right.drawMarkers(right)
        # for i, info in enumerate(right_full_info):
        #     print(info)

        # # Optionally, display the images for debugging
        # cv2.imshow("Left Image", left)
        # cv2.imshow("Right Image", right)
        # cv2.waitKey(1)
        

        self.tictok.update()
        self.tictok.pprint()


def main(args=None):
    rclpy.init(args=args)
    node = ZED_SEKELETAL_NODE()
    rclpy.spin(node)
    node.cam.close_cam()
    node.destroy_node()
    rclpy.shutdown()
    


if __name__ == "__main__":
    main()
