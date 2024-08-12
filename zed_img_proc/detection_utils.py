import time
import numpy as np


class TimerTicTok:
    def __init__(self):
        self.prev = time.time()
    
    def update(self):
        now = time.time()
        self.elapse = now - self.prev
        self.prev = now
    
    def pprint(self):
        # print up to 2 decimal places
        print(f"dt = {self.elapse:.2f} sec, freq = {1.0/self.elapse:.2f}")


def calculate_XYZ_zed(marker_xy, point_cloud):
    N = len(marker_xy)
    result = []
    for i in range(0, N):
        marker_id, mx, my = marker_xy[i]
        _, xyz_mm = point_cloud.get_value(mx, my)
        #zed_xyz_mm = np.array([xyz_mm[0], xyz_mm[1], xyz_mm[2]])
        result.append(
            {
                "m_id": marker_id*1.0,
                "xy": {
                    "x": mx,
                    "y": my
                },
                "zed_xyz_mm": {
                    "x": xyz_mm[0],
                    "y": xyz_mm[1],
                    "z": xyz_mm[2]
                }
            }
        )
    return result

def calculate_depth_zed(marker_xy, depth_map):
    N = len(marker_xy)
    result = []
    for i in range(0, N):
        marker_id, mx, my = marker_xy[i]
        zed_depth_mm = depth_map.get_value(mx, my)
        result.append(
            {
                "m_id": marker_id,
                "xy": [mx, my],
                "zed_depth_mm": zed_depth_mm
            }
        )
    return result