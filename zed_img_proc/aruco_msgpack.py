from aruco_interface.msg import ImageMarkers, ArucoMarker, Point2D
from std_msgs.msg import Header
import rclpy

def pack_aruco(cam_idx, idCornerMap):
    # Initialize a Header message
    header = Header()

    # Populate the Header fields
    header.stamp = rclpy.clock.Clock().now().to_msg()  # Set the timestamp to the current time
    header.frame_id = "base_link"  # Set the frame_id to some meaningful frame

    # Initialize an ImageMarkers message
    image_markers = ImageMarkers()
    image_markers.header = header
    image_markers.cam_idx = cam_idx
    image_markers.aruco_markers = pack_aruco_markers(idCornerMap)
    return image_markers


def pack_aruco_markers(idCornerMap):
    keys = idCornerMap.keys()
    aruco_markers = []
    for key in keys:
        corners = idCornerMap[key]
        marker = pack_each_aruco(key, corners)
        aruco_markers.append(marker)
    return aruco_markers

def pack_each_aruco(mid, nd_corners):
    marker = ArucoMarker()
    marker.mid = int(mid)
    all_corners = []
    for i in range(nd_corners.shape[0]):
        point = Point2D()
        point.x = int(nd_corners[i, 0])
        point.y = int(nd_corners[i, 1])
        all_corners.append(point)
    marker.corners = all_corners

    return marker