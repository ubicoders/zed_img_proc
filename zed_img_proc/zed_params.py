import numpy as np

class ZedParams:
    def __init__(self) -> None:
        self.width = 0
        self.height = 0
        self.cx = 0
        self.cy = 0     
        self.fx = 0
        self.fy = 0
        self.k1 = 0
        self.k2 = 0
        self.k3 = 0
        self.p1 = 0
        self.p2 = 0

    def get_camera_matrix(self):
        return np.array([[self.fx, 0.0, self.cx], [0.0, self.fy, self.cy], [0.0, 0.0, 1.0]])

    def get_dist_coeffs(self):
        return np.array([self.k1, self.k2, self.p1, self.p2, self.k3])


class ZedParamsLeft(ZedParams):
    def __init__(self):
        self.width = 1280
        self.height = 720
        self.cx = 642.686
        self.cy = 336.524
        self.fx = 695.943
        self.fy = 696.294
        self.k1 = -0.16649131853850563
        self.k2 = 0.012775384670700843
        self.k3 = 0.009074423303381905
        self.p1 = 0.000286885415779246
        self.p2 = -0.0005858358143788869


class ZedParamsRight(ZedParams):
    def __init__(self):
        self.width = 1280
        self.height = 720
        self.cx=633.435
        self.cy=359.371
        self.fx=699.316
        self.fy=699.77
        self.k1=-0.16785365565962448
        self.k2=0.015367247811478621
        self.k3=0.006068558673254045
        self.p1=-0.0004954562168215774
        self.p2=-0.0004519859550389168



left_params = ZedParamsLeft()
right_params = ZedParamsRight()