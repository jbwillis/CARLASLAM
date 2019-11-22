#! /usr/bin/env python
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

from plotWindow.plotWindow import plotWindow
import matplotlib.pyplot as plt

class VehicleData:
    def __init__(self, max_steer_angle=70.0):
        self.max_steer_angle = max_steer_angle

        self.position_zero  = None
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
        # steer is a single float in [-1, 1], make it an angle by
        # multiplying by the max steer angle
        steer = steer*self.max_steer_angle
        self.steer_data.append(steer)

    def plot(self):
        pw = plotWindow()

        pw.addPlot("Map", self._plotPosition2D())
        pw.addPlot("Positions", self._plotPositionSubplots())
        pw.addPlot("Velocities", self._plotVelocity())
        pw.addPlot("Controls", self._plotControl())

        pw.show()

    def _plotVelocity(self):
        # creates and returns a matplotlib figure object
        vd_np = np.array(self.velocity_data)

        f = plt.figure()
        spx = f.add_subplot(3,1,1)
        spx.plot(vd_np[:,0]);
        spx.legend("x")

        spy = f.add_subplot(3,1,2)
        spy.plot(vd_np[:,1]);
        spy.legend("y")

        spz = f.add_subplot(3,1,3)
        spz.plot(vd_np[:,2]);
        spz.legend("z")

        return f

    def _plotPositionSubplots(self):
        # create 3 subplots of the position, returning a figure object
        pos_np = np.array(self.position_truth)

        f = plt.figure()
        spx = f.add_subplot(3,1,1)
        spx.plot(pos_np[:,0]);
        spx.legend("x")

        spy = f.add_subplot(3,1,2)
        spy.plot(pos_np[:,1]);
        spy.legend("y")

        spz = f.add_subplot(3,1,3)
        spz.plot(pos_np[:,2]);
        spz.legend("z")

        return f

    def _plotPosition2D(self):
        # creates a 2D map of the position, returning a figure object
        pos_np = np.array(self.position_truth)

        f = plt.figure()
        sp = f.add_subplot(1,1,1)
        sp.plot(pos_np[:,0], pos_np[:,1])

        sp.set_aspect('equal', 'box')
        # flip x axis to align with CARLA coordinate frame
        xl = sp.get_xlim()
        sp.set_xlim(xl[::-1])

        return f

    def _plotControl(self):
        # plot the throttle and steering commands

        throt_np     = np.array(self.throttle_data)

        # convert the steering commands into angles
        steer_ang_np = np.array(self.steer_data)

        f = plt.figure()
        spt = f.add_subplot(2,1,1)
        spt.plot(throt_np)
        spt.legend("Throttle")

        sps = f.add_subplot(2,1,2)
        sps.plot(steer_ang_np)
        sps.set_ylabel('degrees')
        sps.legend("Steering Angle")

        return f


