import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import cv2
import numpy as np
from .zed_utils import ZEDCam
from .aruco_utils import ArucoDetector
from .detection_utils import TimerTicTok, calculate_depth_zed, calculate_XYZ_zed
import json

class ZedArucoNode(Node):
    def __init__(self):
        super().__init__('zed_aruco_node')
        
        self.declare_parameter('has_depth', False)
        
        # Create a Camera object
        has_depth = self.get_parameter('has_depth').get_parameter_value().bool_value
        self.cam = ZEDCam(has_depth=has_depth)
        self.cam.open_cam()

        # Create ArucoDetector objects
        self.aruco_left = ArucoDetector()
        self.aruco_right = ArucoDetector(is_left=False)

        # tic-tok timer
        self.tictok = TimerTicTok()

        # ROS 2 Publisher for 3D vector arrays
        self.vector_pub = self.create_publisher(Float32MultiArray, 'zed/markers/vec3', 10)

        # Timer for periodic callback
        self.timer = self.create_timer(0.01, self.timer_callback)

    def timer_callback(self):
        if not self.cam.update_cam(left=True, right=False, depth=False, point_cloud=True):
            self.get_logger().error("Camera error")
            return

        left, right = self.cam.get_bgr_images()

        # Detect ArUco markers in the left image
        self.aruco_left.detect_bgr(left, estimate_pose=False)
        ids, marker_xy_list, tvecs = self.aruco_left.get_xy_XYZ_info()
        left = self.aruco_left.drawMarkers(left)

        # # Calculate depth for the detected markers
        # depth_map = self.cam.get_depth_map()
        # if ids is not None and marker_xy_list is not None:
        #     m_info = calculate_depth_zed(marker_xy_list, depth_map)
        #     self.publish_3d_vectors(m_info)

        point_cloud = self.cam.get_point_cloud()
        if (ids is not None) :
            m_info = calculate_XYZ_zed(marker_xy_list, point_cloud)
            for i in m_info:
                print(json.dumps(i, indent=4))

        # Optionally, display the images for debugging
        cv2.imshow("Left Image", left)
        #cv2.imshow("Right Image", right)
        cv2.waitKey(1)
        

        self.tictok.update()
        self.tictok.pprint()


  

    def publish_3d_vectors(self, m_info):
        print(m_info)
            
        # # Prepare the message with 3D vectors
        # msg = Float32MultiArray()
        # msg.data = np.array(m_info).flatten().tolist()  # Flatten the array to a 1D list
        # self.vector_pub.publish(msg)
        # self.get_logger().info(f'Published 3D vectors: {msg.data}')


def main(args=None):
    rclpy.init(args=args)
    node = ZedArucoNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
