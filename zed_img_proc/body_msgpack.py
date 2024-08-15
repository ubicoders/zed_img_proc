from skeleton_interface.msg import BodyKeyPoints
from geometry_msgs.msg import Vector3
import numpy as np

# def pack_body(confidence, body_keypoints):
def pack_body(body):
    confidence = body.confidence
    keypoint_2d = body.keypoint_2d
    tracking_state = body.tracking_state
    body_msg = BodyKeyPoints()
    body_msg.body_id = body.id
    body_msg.confidence = confidence
    body_msg.position = pack_vec3(body.position)    
    body_msg.velocity = pack_vec3(body.velocity)
    body_msg.key_points_2d = []
    for i in range(keypoint_2d.shape[0]):
        k2dmsg = pack_vec2(keypoint_2d[i])
        body_msg.key_points_2d.append(k2dmsg)
    return body_msg

def pack_vec3(nd_xyz):
    vec3 = Vector3()
    vec3.x = get_safe_val(nd_xyz[0])
    vec3.y = get_safe_val(nd_xyz[1])
    vec3.z = get_safe_val(nd_xyz[2])
    return vec3

def pack_vec2(nd_xy):
    vec2 = Vector3()
    vec2.x = get_safe_val(nd_xy[0])
    vec2.y = get_safe_val(nd_xy[1])
    vec2.z = 0.0
    return vec2


def get_safe_val(value):
    if value is np.nan or value is np.inf or value is -np.inf or value is None:
        return -1.0
    return value