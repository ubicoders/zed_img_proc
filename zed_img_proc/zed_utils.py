import cv2
import pyzed.sl as sl

class ZEDCam:
    def __init__(self):
        self.init_params = sl.InitParameters()
        self.init_params.camera_resolution = sl.RESOLUTION.HD720
        self.init_params.camera_fps = 60
        self.runtime_parameters = sl.RuntimeParameters()
    
        self.image_left = sl.Mat()
        self.image_right = sl.Mat()
    
        self.zed = sl.Camera()

    def open_cam(self):
        if self.zed.open(self.init_params) != sl.ERROR_CODE.SUCCESS:
            print("Failed to open the ZED camera")
            return
    
    def update_cam(self):
        if self.zed.grab(self.runtime_parameters) == sl.ERROR_CODE.SUCCESS:
            # Retrieve left image
            self.zed.retrieve_image(self.image_left, sl.VIEW.LEFT)
            # Retrieve right image
            self.zed.retrieve_image(self.image_right, sl.VIEW.RIGHT)

            self.bgr_left = cv2.cvtColor(self.image_left.get_data(),  cv2.COLOR_BGRA2BGR)
            self.bgr_right = cv2.cvtColor(self.image_right.get_data(),  cv2.COLOR_BGRA2BGR)
            return True
        else:
            return False

    def close_cam(self):
        self.zed.close()

    def get_bgr_images(self):
        return self.bgr_left, self.bgr_right