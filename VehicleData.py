#! /usr/bin/env python3
# custom class for keeping track of a vehicle's data

import numpy as np
from utils import *
import map

# from MotionModel import modelStep

from plotWindow.plotWindow import plotWindow
import matplotlib.pyplot as plt

class VehicleData:
    def __init__(self):

        self.time_vec       = [] 
        self.position_zero  = None
        self.position_truth = []
        self.heading_truth  = []
        self.velocity_data  = []
        self.throttle_data  = []
        self.steer_data     = []
        self.odom_state     = []
        self.lidar_data     = []
        self.max_steer_angle = None
        self.wheelbase      = None

    def config(self, wheelbase, max_steer_angle):
        self.max_steer_angle = max_steer_angle
        self.wheelbase      = wheelbase

    def appendTime(self, t):
        self.time_vec.append(t)

    def appendVelocityData(self, vel):
        # vel is a carla.Vector3D object
        vel_arr = vector3DToNp(vel)
        self.velocity_data.append(vel_arr)

    def appendLidarData(self, new_data):
        self.lidar_data.append(new_data)
        
    def appendTransformTruth(self, txfm):
        # txfm is a carla.Transform object
        pos_arr = vector3DToNp(txfm.location)

        yaw_rad = np.deg2rad(txfm.rotation.yaw)

        if self.position_zero is None:
            self.position_zero = pos_arr

        # transform to coordinate frame with initial position as origin
        pos_arr = pos_arr - self.position_zero

        self.position_truth.append(pos_arr)
        self.heading_truth.append(yaw_rad)

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

    def runMotionModelFull(self):
        # run the motion model using already saved data and save off data for plotting
        
        pass
        # state_km1 = [0, 0, self.heading_truth[0]]

        # vd_np        = np.array(self.velocity_data)
        # steer_ang_np = np.array(self.steer_data)
        # gamma_prev = 0
        # alpha = .5

        # for indx in range(0, len(self.time_vec)):

            # v = np.linalg.norm(vd_np[indx, 0:2])
            # gamma = np.deg2rad(steer_ang_np[indx])

            # if (gamma - gamma_prev) > np.pi/8:
                # gamma = gamma_prev + np.pi/8
            # elif (gamma - gamma_prev) < -np.pi/8:
                # gamma = gamma_prev - np.pi/8

            # gamma = alpha*gamma + (1 - alpha)*gamma_prev


            # state_k = modelStep(state_km1, [v, gamma])
            # self.odom_state.append(np.array(state_k))
            # state_km1 = state_k

            # gamma_prev = gamma

    def plot(self):
        pw = plotWindow()

        pw.addPlot("Map", self._plotPosition2D())
        pw.addPlot("Positions", self._plotPositionSubplots())
        pw.addPlot("Velocities", self._plotVelocity())
        pw.addPlot("Controls", self._plotControl())

        scan = thresholdScan(self.lidar_data[10])
        pw.addPlot("Lidar", self._plotLidarScan(scan))

        pw.show()

    def _plotLidarScan(self, scan):

        scan = thresholdScan(scan)
        # generate occupancy grid map
        m  = map.gridMapFromScan(scan, 100)

        f, ax = plt.subplots()
        im = ax.pcolormesh(m.gridmap, vmax=10, vmin=-10)
        f.colorbar(im, ax=ax)
        ax.axis('equal')

        return f

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
        if len(self.odom_state) > 0:
            odom_np = np.array(self.odom_state)
        else:
            odom_np = np.NaN*np.ones((len(self.time_vec), 3))

        f = plt.figure()
        spx = f.add_subplot(4,1,1)
        spx.plot(t, pos_np[:,0])
        spx.plot(t, odom_np[:,0])
        spx.legend(["x", "x_odom"])

        spy = f.add_subplot(4,1,2)
        spy.plot(t, pos_np[:,1])
        spy.plot(t, odom_np[:,1])
        spy.legend(["y", "y_odom"])

        spz = f.add_subplot(4,1,3)
        spz.plot(t, pos_np[:,2]);
        spz.legend(["z"])

        spth = f.add_subplot(4,1,4)
        spth.plot(t, head_np);
        spth.plot(t, odom_np[:,2])
        spth.legend(["heading", "heading_odom"])

        return f

    def _plotPosition2D(self):
        # creates a 2D map of the position, returning a figure object
        pos_np = np.array(self.position_truth)

        if len(self.odom_state) > 0:
            odom_np = np.array(self.odom_state)
        else:
            odom_np = np.NaN*np.ones((len(self.time_vec), 3))

        f = plt.figure()
        sp = f.add_subplot(1,1,1)
        sp.plot(pos_np[:,0], pos_np[:,1])
        sp.plot(odom_np[:,0], odom_np[:,1])

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
        head_np      = np.array(self.heading_truth)
        throt_np     = np.array(self.throttle_data)
        steer_ang_np = np.array(self.steer_data)
        lidar_np     = np.array(self.lidar_data)

        wheelbase_np = np.array([self.wheelbase])
        max_steer_np = np.array([self.max_steer_angle])

        # save using savez
        np.savez(filename,
                t_np = t_np,
                vd_np = vd_np,
                pos_np = pos_np,
                head_np = head_np,
                throt_np = throt_np,
                steer_ang_np = steer_ang_np,
                wheelbase_np = wheelbase_np,
                max_steer_np = max_steer_np,
                lidar_np = lidar_np)

def loadFromFile(filename):
    # load from file
    all_data = np.load(filename, allow_pickle=True, encoding='bytes')

    wheelbase      = all_data['wheelbase_np'].item(0)
    max_steer_angle= all_data['max_steer_np'].item(0)

    # create vehicle data object
    vd = VehicleData()
    vd.config(wheelbase = wheelbase, max_steer_angle = max_steer_angle)

    # extract arrays
    vd.time_vec       = all_data['t_np']
    vd.velocity_data  = all_data['vd_np']
    vd.position_truth = all_data['pos_np']
    vd.heading_truth  = all_data['head_np']
    vd.throttle_data  = all_data['throt_np']
    vd.steer_data     = all_data['steer_ang_np']
    vd.lidar_data     = all_data["lidar_np"]

    return vd

if __name__ == '__main__':
    import sys
    from params import global_params as GP

    if len(sys.argv) > 1:
        fname = sys.argv[1]
        vd = loadFromFile(fname)
        
        # set global parameters
        GP.wheelbase = vd.wheelbase
        GP.Ts = vd.time_vec[1] - vd.time_vec[0] 

        vd.runMotionModelFull()
        vd.plot()
    else:
        print('Please specify a filename')
