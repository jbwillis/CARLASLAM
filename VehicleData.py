# custom class for keeping track of a vehicle's data

# import glob
# import os
# import sys
# try:
    # sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        # sys.version_info.major,
        # sys.version_info.minor,
        # 'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
# except IndexError:
    # pass

# import carla
import numpy as np
from utils import *

class VehicleData:
    def __init__(self, max_steer_angle=70.0)
        self.max_steer_angle = max_steer_angle

        self.position_truth = []
        self.velocity_data  = []
        self.throttle_data  = []
        self.steer_data     = []

    def appendVelocityData(vd)
        # vd is a carla.Vector3D object
        vd_arr = vector3DToNp(vd)
        self.velocity_data.append(vd_arr)
        
    def appendPositionTruth(pos)
        # pos is a carla.Vector3D object
        pos_arr = vector3DToNp(pos)
        self.position_data.append(pos_arr)

    def appendControlData(cd)
        self._appendThrottleData(cd.throttle)
        self._appendSteerData(cd.steer)
        
    def _appendThrottleData(throttle)
        # throttle is a single float in [0, 1]
        self.throttle_data.append(throttle)

    def _appendSteerData(steer)
        # steer is a single float in [-1, 1]
        self.steer_data.append(steer)
