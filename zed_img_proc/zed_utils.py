import cv2
import pyzed.sl as sl

class ZEDCam:
    def __init__(self, ):
        self.init_params = sl.InitParameters()
        self.init_params.camera_resolution = sl.RESOLUTION.HD720
        self.init_params.camera_fps = 60
        self.runtime_parameters = sl.RuntimeParameters()
        
        # depth mode config
        self.init_params.depth_mode = sl.DEPTH_MODE.PERFORMANCE
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
    
    def update_cam(self):
        if self.zed.grab(self.runtime_parameters) == sl.ERROR_CODE.SUCCESS:
            return True
        else:
            return False

    def close_cam(self):
        self.zed.close()

    def get_bgr_left(self):
        self.zed.retrieve_image(self.image_left, sl.VIEW.LEFT)
        return cv2.cvtColor(self.image_left.get_data(),  cv2.COLOR_BGRA2BGR)
    
    def get_bgr_right(self):
        self.zed.retrieve_image(self.image_right, sl.VIEW.RIGHT)
        return cv2.cvtColor(self.image_right.get_data(),  cv2.COLOR_BGRA2BGR)

    def get_depth(self):
        # pyzed doesn't support GPU
        self.zed.retrieve_image(self.depth_image_zed, sl.VIEW.DEPTH)
        return self.depth_image_zed


    def get_point_cloud(self):
        self.zed.retrieve_measure(self.point_cloud, sl.MEASURE.XYZRGBA)
        return self.point_cloud