import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import cv2
import numpy as np
from .zed_utils import ZEDCam
from .aruco_utils import ArucoDetector
from .detection_utils import TimerTicTok, calculate_depth_zed, calculate_XYZ_zed
import json
from aruco_interface.msg import StereoImageMarkers, ImageMarkers, Marker, Point2D

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
        self.aruco_full_pub = self.create_publisher(StereoImageMarkers, '/stereo_aruco', 10)

        # Timer for periodic callback
        self.timer = self.create_timer(0.01, self.timer_callback)

    def timer_callback(self):
        if not self.cam.update_cam(left=True, right=True, depth=False, point_cloud=False):
            self.get_logger().error("Camera error")
            return

        left, right = self.cam.get_bgr_images()

        # Detect ArUco markers in the left image
        self.aruco_left.detect_bgr(left, estimate_pose=True)
        left_ids, left_full_info = self.aruco_left.get_xy_XYZ_TR_info()

        # Detect ArUco markers in the right image
        self.aruco_right.detect_bgr(right, estimate_pose=True)
        right_ids, right_full_info = self.aruco_right.get_xy_XYZ_TR_info()
        
        # Pack the full info into a message
        stereo_img_markers = self.pack_full_info(left_full_info, right_full_info)
        self.aruco_full_pub.publish(stereo_img_markers)
       
       
        ##=========================================================================================================
        # # left plot
        # left = self.aruco_left.drawMarkers(left)
        # for i, info in enumerate(left_full_info):
        #     print(info)

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

    def pack_full_info(self, left_full_info, right_full_info):
        left_img_markers = self.pack_markers(left_full_info)
        left_img_markers.image_name = "left"

        right_img_markers = self.pack_markers(right_full_info)
        right_img_markers.image_name = "right"

        stereo_img_markers = StereoImageMarkers()
        stereo_img_markers.left_image = left_img_markers
        stereo_img_markers.right_image = right_img_markers
        return stereo_img_markers

 

    def pack_markers(self, full_info):
        img_markers = ImageMarkers()
        img_markers.markers = []
        for info in full_info:
            marker = Marker()
            marker.id = int(info["id"])
            marker.xy.x = float(info["xy"]["x"])
            marker.xy.y = float(info["xy"]["y"])
            marker.tvec.x = info["tvec"]["x"]
            marker.tvec.y = info["tvec"]["y"]
            marker.tvec.z = info["tvec"]["z"]
            marker.dist = info["dist"]
            marker.eul.x = info["eul"]["x"]
            marker.eul.y = info["eul"]["y"]
            marker.eul.z = info["eul"]["z"]
            # print(marker)
            img_markers.markers.append(marker)
        return img_markers
            
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
