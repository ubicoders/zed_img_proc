import cv2
import pyzed.sl as sl

class ZEDCam:
    def __init__(self, has_depth=False):
        self.init_params = sl.InitParameters()
        self.init_params.camera_resolution = sl.RESOLUTION.HD720
        self.init_params.camera_fps = 60
        self.runtime_parameters = sl.RuntimeParameters()
        if has_depth:
            self.init_params.depth_mode = sl.DEPTH_MODE.ULTRA
            self.init_params.coordinate_units = sl.UNIT.MILLIMETER        
    
        self.image_left = sl.Mat()
        self.image_right = sl.Mat()
        self.depth_map = sl.Mat()
        self.point_cloud = sl.Mat()
    
        self.zed = sl.Camera()

    def open_cam(self):
        if self.zed.open(self.init_params) != sl.ERROR_CODE.SUCCESS:
            print("Failed to open the ZED camera")
            return
    
    def update_cam(self, left=True, right=False, depth=False, point_cloud=False):
        if self.zed.grab(self.runtime_parameters) == sl.ERROR_CODE.SUCCESS:
            # Retrieve left image
            if left:
                self.zed.retrieve_image(self.image_left, sl.VIEW.LEFT)
                self.bgr_left = cv2.cvtColor(self.image_left.get_data(),  cv2.COLOR_BGRA2BGR)
            # Retrieve right image
            if right:
                self.zed.retrieve_image(self.image_right, sl.VIEW.RIGHT)    
                self.bgr_right = cv2.cvtColor(self.image_right.get_data(),  cv2.COLOR_BGRA2BGR)
            # Retrieve depth map
            if depth:
                self.zed.retrieve_measure(self.depth_map, sl.MEASURE.DEPTH) # Retrieve depth
            # Retrieve point cloud
            if point_cloud:
                self.zed.retrieve_measure(self.point_cloud, sl.MEASURE.XYZRGBA)
            
            return True
        else:
            return False

    def close_cam(self):
        self.zed.close()

    def get_bgr_images(self):
        if hasattr(self, 'bgr_left') and hasattr(self, 'bgr_right'):
            return self.bgr_left, self.bgr_right
        elif hasattr(self, 'bgr_left'):
            return self.bgr_left, None
        elif hasattr(self, 'bgr_right'):
            return None, self.bgr_right
        else:
            return None, None
    
    def get_depth_map(self):
        if hasattr(self, 'depth_map'):
            return self.depth_map
        else:
            return None

    def get_point_cloud(self):
        if hasattr(self, 'point_cloud'):
            return self.point_cloud
        else:
            return None