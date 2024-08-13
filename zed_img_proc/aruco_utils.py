import cv2.aruco as aruco
import cv2
import numpy as np
import os

def read_calibration_data(file_name):
    try:
        with np.load(file_name) as data:
            return data['mtx'], data['dist']
    except (FileNotFoundError, KeyError, IOError) as e:
        print(f"Error: {e}")
        return None, None

def rotation_matrix_to_euler_angles(R):
    """
    Converts a rotation matrix to Euler angles (in radians).
    """
    sy = np.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])
    singular = sy < 1e-6
    if not singular:
        x = np.arctan2(R[2, 1], R[2, 2])
        y = np.arctan2(-R[2, 0], sy)
        z = np.arctan2(R[1, 0], R[0, 0])
    else:
        x = np.arctan2(-R[1, 2], R[1, 1])
        y = np.arctan2(-R[2, 0], sy)
        z = 0
    return np.array([x, y, z])    
   
class ArucoDetector:
    def __init__(self, is_left=True) -> None:
        base_path = os.path.dirname(os.path.abspath(__file__))
        aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_1000)
        arucoParameters = aruco.DetectorParameters()
        self.detector = aruco.ArucoDetector(aruco_dict, arucoParameters)
        self.is_left = is_left
        self.calibration_file = f"{base_path}/calibration_left.npz" if is_left else f"{base_path}/calibration_right.npz"
        self.camera_matrix, self.dist_coeffs = read_calibration_data(self.calibration_file)

        self.corners = None
        self.ids = None
        self.rvecs = None
        self.tvecs = None
        self.pos_estimate = False
        self.marker_size = 0.155
        self.tr_info = []

    def detect_bgr(self, bgr_img, estimate_pose=False):
        self.pos_estimate = estimate_pose
        gray_img = cv2.cvtColor(bgr_img, cv2.COLOR_BGR2GRAY)
        self.detect_gray(gray_img, estimate_pose)

    def detect_gray(self, gray_img, estimate_pose=False):
        self.pos_estimate = estimate_pose
        # return corners, ids, rejectedImgPoints
        self.corners, self.ids, _ = self.detector.detectMarkers(gray_img)        
        if estimate_pose:
            self.estimate_position()

        
    def drawMarkers(self, img):
        if (self.corners is None) or (self.ids is None):
            return img
        img = aruco.drawDetectedMarkers(img, self.corners)
        if self.pos_estimate is False:
            return img
        for i in range(len(self.ids)):
            cv2.drawFrameAxes(img, self.camera_matrix, self.dist_coeffs, self.rvecs[i], self.tvecs[i], 0.1)
        return img
    
    def get_xy_info(self):
        marker_xy_list = []
        for i, corner in enumerate(self.corners):
            # Calculate the center (x, y) coordinates of the marker
            center_x = int(np.mean(corner[:, :, 0]))
            center_y = int(np.mean(corner[:, :, 1]))
            # Get the marker ID
            marker_id = self.ids[i][0]
            marker_xy_list.append((marker_id, center_x, center_y))
        return self.ids, marker_xy_list
    
    def get_xy_XYZ_info(self):
        _, marker_xy_list = self.get_xy_info()
        return self.ids, marker_xy_list, self.tvecs
    
    def get_xy_XYZ_TR_info(self):
        _, marker_xy_list, _ = self.get_xy_XYZ_info()
        full_info = []
        if (self.ids is None) or (self.tvecs is None) or (len(self.tvecs) == 0):
            return self.ids, full_info
        for i in range(len(self.ids)):
            mid, cx, cy = marker_xy_list[i]
            tr_info = self.get_tr_info(mid)
            temp_info = {"id": mid, 
                         "xy": {"x": marker_xy_list[i][1], "y": marker_xy_list[i][2]}, 
                         "tvec": {"x": self.tvecs[i][0][0], "y": self.tvecs[i][0][1], "z": self.tvecs[i][0][2]},
                         "dist": tr_info["dist"],
                         "eul": tr_info["eul"]}
            full_info.append(temp_info)
        return self.ids, full_info
        
    def get_tr_info(self, id):
        for info in self.tr_info:
            if info["id"] == id:
                return info
        return None

    def estimate_position(self):
        if (self.camera_matrix is None) or (self.dist_coeffs is None):
            print("Error: Camera calibration data not found")
            return None, None
        if (self.corners is None) or (self.ids is None):
            #print("Error: No ArUco markers detected")
            return None, None
        self.rvecs, self.tvecs, _ = aruco.estimatePoseSingleMarkers(self.corners,     self.marker_size , self.camera_matrix, self.dist_coeffs)
        for i in range(len(self.ids)):
            R, _ = cv2.Rodrigues(self.rvecs[i])
            # Calculate distance to the marker
            distance = np.linalg.norm(self.tvecs[i])
            eul = rotation_matrix_to_euler_angles(R)
            temp_tr_info = {"id": self.ids[i][0], "dist": distance, "eul": {"x": eul[0], "y": eul[1], "z": eul[2]}}
            self.tr_info.append(temp_tr_info)
