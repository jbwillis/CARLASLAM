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
    def __init__(self, max_steer_angle=70.0):
        self.max_steer_angle = max_steer_angle

        self.position_truth = []
        self.velocity_data  = []
        self.throttle_data  = []
        self.steer_data     = []

    def appendVelocityData(self, vel):
        # vel is a carla.Vector3D object
        vel_arr = vector3DToNp(vel)
        self.velocity_data.append(vel_arr)
        
    def appendPositionTruth(self, pos):
        # pos is a carla.Vector3D object
        pos_arr = vector3DToNp(pos)

        if self.position_zero is None:
            self.position_zero = pos_arr

        # transform to coordinate frame with initial position as origin
        pos_arr = pos_arr - self.position_zero

        self.position_truth.append(pos_arr)

    def appendControlData(self, cd):
        self._appendThrottleData(cd.throttle)
        self._appendSteerData(cd.steer)
        
    def _appendThrottleData(self, throttle):
        # throttle is a single float in [0, 1]
        self.throttle_data.append(throttle)

    def _appendSteerData(self, steer):
        # steer is a single float in [-1, 1]
        self.steer_data.append(steer)
