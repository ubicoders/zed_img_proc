import cv2
import pyzed.sl as sl

# 17437, 14100309


class ZEDCam:
    def __init__(self, body_track = False , serial_number=14100309):
        self.zed = sl.Camera()

        self.init_params = sl.InitParameters()
        self.init_params.set_from_serial_number(serial_number)
        self.init_params.camera_resolution = sl.RESOLUTION.HD720
        self.init_params.camera_fps = 60       
        
        # depth mode config
        self.init_params.depth_mode = sl.DEPTH_MODE.PERFORMANCE
        self.init_params.coordinate_units = sl.UNIT.METER      
        self.init_params.coordinate_system = sl.COORDINATE_SYSTEM.RIGHT_HANDED_Y_UP  
    
        self.image_left = sl.Mat()
        self.image_right = sl.Mat()
        self.depth_map = sl.Mat()
        self.point_cloud = sl.Mat()
        self.bodies = sl.Bodies()
        self.body_track = body_track
            

    def open_cam(self):
        if self.zed.open(self.init_params) != sl.ERROR_CODE.SUCCESS:
            print("Failed to open the ZED camera")
            return       

        if self.body_track:
            positional_tracking_parameters = sl.PositionalTrackingParameters()
            self.zed.enable_positional_tracking(positional_tracking_parameters)

            body_param = sl.BodyTrackingParameters()
            body_param.enable_tracking = True                # Track people across images flow
            body_param.enable_body_fitting = False            # Smooth skeleton move
            body_param.detection_model = sl.BODY_TRACKING_MODEL.HUMAN_BODY_FAST 
            body_param.body_format = sl.BODY_FORMAT.BODY_18  # Choose the BODY_FORMAT you wish to use
            # Enable Object Detection module
            self.zed.enable_body_tracking(body_param)
            self.body_track_runtime_parameters = sl.BodyTrackingRuntimeParameters()
            self.body_track_runtime_parameters.detection_confidence_threshold = 40
        else:
            self.runtime_parameters = sl.RuntimeParameters()
    
    def update_cam(self):
        if self.body_track:
            return self.zed.grab() == sl.ERROR_CODE.SUCCESS
        else:
            return self.zed.grab(self.runtime_parameters) == sl.ERROR_CODE.SUCCESS
           

    def close_cam(self):
        self.zed.close()

    def get_bgr_left(self):
        self.zed.retrieve_image(self.image_left, sl.VIEW.LEFT)
        return cv2.cvtColor(self.image_left.get_data(),  cv2.COLOR_BGRA2BGR)
    
    def get_bgr_right(self):
        self.zed.retrieve_image(self.image_right, sl.VIEW.RIGHT)
        return cv2.cvtColor(self.image_right.get_data(),  cv2.COLOR_BGRA2BGR)

    def get_body_tracking(self):
        self.zed.retrieve_bodies(self.bodies, self.body_track_runtime_parameters)
        return self.bodies

    def get_depth(self):
        # pyzed doesn't support GPU
        self.zed.retrieve_image(self.depth_image_zed, sl.VIEW.DEPTH)
        return self.depth_image_zed

    def get_point_cloud(self):
        self.zed.retrieve_measure(self.point_cloud, sl.MEASURE.XYZRGBA)
        return self.point_cloud