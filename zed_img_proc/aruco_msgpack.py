from aruco_interface.msg import ImageMarkers, ArucoMarker, Point2D
from std_msgs.msg import Header
from std_msgs.msg import String
from geometry_msgs.msg import Vector3
import rclpy


def pack_aruco(serial_number, left_idCornerMap, right_idCornerMap, idXYZMap=None):
    # Initialize a Header message
    header = Header()

    # Populate the Header fields
    header.stamp = rclpy.clock.Clock().now().to_msg()  # Set the timestamp to the current time
    header.frame_id = "zed_frame"  # Set the frame_id to some meaningful frame

    # Initialize an ImageMarkers message
    image_markers = ImageMarkers()
    image_markers.header = header
    cam_name_msg = String()
    cam_name_msg.data = f"zed_{serial_number}"
    image_markers.cam_name = cam_name_msg
    image_markers.aruco_markers_0 = pack_aruco_markers(left_idCornerMap, idXYZMap)
    image_markers.aruco_markers_1 = pack_aruco_markers(right_idCornerMap)
    return image_markers


def pack_aruco_markers(idCornerMap, idXYZMap=None):
    keys = idCornerMap.keys()
    aruco_markers = []
    for key in keys:
        if key not in [0, 1, 2, 3, 4, 5]:
            continue
        corners = idCornerMap[key]
        marker = pack_each_aruco(key, corners, idXYZMap[key])
        aruco_markers.append(marker)
    return aruco_markers

def pack_each_aruco(mid, nd_corners, nd_xyz=None):
    marker = ArucoMarker()
    marker.mid = int(mid)
    all_corners = []
    for i in range(nd_corners.shape[0]):
        point = Point2D()
        point.x = int(nd_corners[i, 0])
        point.y = int(nd_corners[i, 1])
        all_corners.append(point)
    marker.corners = all_corners

    if (nd_xyz is not None):
        vec3 = Vector3()
        vec3.x = nd_xyz[0]
        vec3.y = nd_xyz[1]
        vec3.z = nd_xyz[2]
        marker.xyz = vec3

    return marker