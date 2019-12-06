#! /usr/bin/env python
# custom class for keeping track of a vehicle's data

import numpy as np
from utils import *

from MotionModel import modelStep

from plotWindow.plotWindow import plotWindow
import matplotlib.pyplot as plt

class VehicleData:
    def __init__(self, wheelbase, max_steer_angle=70.0):
        self.max_steer_angle = max_steer_angle
        self.wheelbase      = wheelbase

        self.time_vec       = [] 
        self.position_zero  = None
        self.position_truth = []
        self.heading_truth  = []
        self.velocity_data  = []
        self.throttle_data  = []
        self.steer_data     = []

    def appendTime(self, t):
        self.time_vec.append(t)

    def appendVelocityData(self, vel):
        # vel is a carla.Vector3D object
        vel_arr = vector3DToNp(vel)
        self.velocity_data.append(vel_arr)
        
    def appendTransformTruth(self, txfm):
        # txfm is a carla.Transform object
        pos_arr = vector3DToNp(txfm.location)

        if self.position_zero is None:
            self.position_zero = pos_arr

        # transform to coordinate frame with initial position as origin
        pos_arr = pos_arr - self.position_zero

        self.position_truth.append(pos_arr)
        self.heading_truth.append(txfm.rotation.yaw)

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
        t     = np.array(self.time_vec)
        vd_np = np.array(self.velocity_data)

        f = plt.figure()
        spx = f.add_subplot(3,1,1)
        spx.plot(t, vd_np[:,0]);
        spx.legend("x")

        spy = f.add_subplot(3,1,2)
        spy.plot(t, vd_np[:,1]);
        spy.legend("y")

        spz = f.add_subplot(3,1,3)
        spz.plot(t, vd_np[:,2]);
        spz.legend("z")

        return f

    def _plotPositionSubplots(self):
        # create 3 subplots of the position, returning a figure object
        t       = np.array(self.time_vec)
        pos_np  = np.array(self.position_truth)
        head_np = np.array(self.heading_truth)

        f = plt.figure()
        spx = f.add_subplot(4,1,1)
        spx.plot(t, pos_np[:,0]);
        spx.legend("x")

        spy = f.add_subplot(4,1,2)
        spy.plot(t, pos_np[:,1]);
        spy.legend("y")

        spz = f.add_subplot(4,1,3)
        spz.plot(t, pos_np[:,2]);
        spz.legend("z")

        spz = f.add_subplot(4,1,4)
        spz.plot(t, head_np);
        spz.legend("heading")

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

        t        = np.array(self.time_vec)
        throt_np = np.array(self.throttle_data)

        # convert the steering commands into angles
        steer_ang_np = np.array(self.steer_data)

        f = plt.figure()
        spt = f.add_subplot(2,1,1)
        spt.plot(t, throt_np)
        spt.legend("Throttle")

        sps = f.add_subplot(2,1,2)
        sps.plot(t, steer_ang_np)
        sps.set_ylabel("degrees")
        sps.legend("Steering Angle")

        return f

    def saveToFile(self, filename):

        # create numpy arrays of everything
        t_np         = np.array(self.time_vec)
        vd_np        = np.array(self.velocity_data)
        pos_np       = np.array(self.position_truth)
        throt_np     = np.array(self.throttle_data)
        steer_ang_np = np.array(self.steer_data)

        # concatenate the arrays into one big array
        all_data = np.column_stack([t_np, vd_np, pos_np, throt_np, steer_ang_np])

        # save it
        np.savetxt(filename, all_data, delimiter=', ', 
                header='time, x_vel, y_vel, z_vel, x_pos_tr, y_pos_tr, z_pos_tr, throttle, steering angle')

    def loadFromFile(self, filename):
        # load from file
        all_data = np.loadtxt(filename, delimiter=', ')

        # extract arrays
        self.time_vec       = all_data[:,0:1]
        self.velocity_data  = all_data[:,1:4]
        self.position_truth = all_data[:,4:7]
        self.throttle_data  = all_data[:,7:8]
        self.steer_data     = all_data[:,8:9]
        

if __name__ == '__main__':
    import sys
    if len(sys.argv) > 1:
        fname = sys.argv[1]
        vd = VehicleData()
        vd.loadFromFile(fname)
        vd.plot()
    else:
        print('Please specify a filename')
