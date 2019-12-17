#! /usr/bin/env python3

import numpy as np
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
from plotWindow.plotWindow import plotWindow

import transforms as tfm
import VehicleData
import utils
from params import global_params as GP


def testAll(i):

    pw = plotWindow()

    vd = VehicleData.loadFromFile("captured_data/route1_vd.npz")

    # extract data from file
    scan0 = vd.lidar_data[i]
    xy0 = vd.position_truth[i,:]
    head0 = vd.heading_truth[i]
    pose0 = np.concatenate([xy0, np.array([head0])])

    # threshold the scan
    scan_th = utils.thresholdScan(scan0)

    # plot the thresholded scan in 3D
    f = plt.figure()
    ax = f.add_subplot(111, projection='3d')
    ax.scatter(scan_th[:,0], scan_th[:,1], scan_th[:,2])
    ax.set_xlabel('x_L')
    ax.set_ylabel('y_L')
    f.suptitle('3D Plot of Thresholded Scan')
    pw.addPlot('3D Scan F_L', f)

    # plot the thresholded scan in 2D
    f, ax = plt.subplots()
    ax.scatter(scan_th[:,0], scan_th[:,1], c=scan_th[:,2])
    ax.set_xlabel('x_L')
    ax.set_ylabel('y_L')
    ax.set_xlim([-60, 60])
    ax.set_ylim([-60, 60])
    f.suptitle('2D Plot of Thresholded Scan')
    pw.addPlot('2D Scan F_L', f)

    # >>>> transform the scan to the body frame
    scan_bd = tfm.lidar2Body(scan_th.T)

    # plot the transformed scan in 3D
    f = plt.figure()
    ax = f.add_subplot(111, projection='3d')
    ax.scatter(scan_bd[0, :], scan_bd[1, :], scan_bd[2, :])
    ax.set_xlabel('x_B')
    ax.set_ylabel('y_B')
    f.suptitle('3D Plot of Body-Frame Scan')
    pw.addPlot('3D Scan F_B', f)

    # plot the transformed scan in 2D
    f, ax = plt.subplots()
    # plot last -> first to get the highest data on top
    ax.scatter(scan_bd[0, ::-1], scan_bd[1, ::-1], c=scan_bd[2, ::-1])
    ax.set_xlabel('x_B')
    ax.set_ylabel('y_B')
    ax.set_xlim([-60, 60])
    ax.set_ylim([-60, 60])
    f.suptitle('2D Plot of Body-Frame Scan')
    pw.addPlot('2D Scan F_B', f)

    # >>>> transform the body frame scan to the vehicle frame
    scan_vh = tfm.body2Vehicle(scan_bd, pose0[3])

    # plot the transformed scan in 3D
    f = plt.figure()
    ax = f.add_subplot(111, projection='3d')
    ax.scatter(scan_vh[0, :], scan_vh[1, :], scan_vh[2, :])
    ax.set_xlabel('x_V')
    ax.set_ylabel('y_V')
    f.suptitle('3D Plot of Vehicle-Frame Scan')
    pw.addPlot('3D Scan F_V', f)

    # plot the transformed scan in 2D
    f, ax = plt.subplots()
    # plot last -> first to get the highest data on top
    ax.scatter(scan_vh[0, ::-1], scan_vh[1, ::-1], c=scan_vh[2, ::-1])
    ax.set_xlabel('x_V')
    ax.set_ylabel('y_V')
    ax.set_xlim([-60, 60])
    ax.set_ylim([-60, 60])
    f.suptitle('2D Plot of Vehicle-Frame Scan')
    pw.addPlot('2D Scan F_V', f)

    # >>>> transform the vehicle frame scan to the global frame
    scan_gl = tfm.vehicle2Global(scan_vh, pose0)

    # plot the transformed scan in 3D
    f = plt.figure()
    ax = f.add_subplot(111, projection='3d')
    ax.scatter(scan_gl[0, :], scan_gl[1, :], scan_gl[2, :])
    ax.set_xlabel('x_G')
    ax.set_ylabel('y_G')
    f.suptitle('3D Plot of Global-Frame Scan')
    pw.addPlot('3D Scan F_G', f)

    # plot the transformed scan in 2D
    f, ax = plt.subplots()
    # plot last -> first to get the highest data on top
    ax.scatter(scan_gl[0, ::-1], scan_gl[1, ::-1], c=scan_gl[2, ::-1])
    ax.set_xlabel('x_G')
    ax.set_ylabel('y_G')
    ax.set_xlim([-60, 60])
    ax.set_ylim([-60, 60])
    f.suptitle('2D Plot of Global-Frame Scan')
    pw.addPlot('2D Scan F_G', f)

    # prepare a matrix to act as a map
    map_dim = 150
    map_offset = map_dim/2
    map_res = 1
    mp = np.zeros((map_dim, map_dim))
    
    # >>>> transform the global frame scan to map indices
    scan_id = tfm.global2Map(scan_gl, map_offset, map_dim, map_res)
    pose_id = tfm.global2Map(pose0[:,np.newaxis], map_offset, map_dim, map_res) #+ map_res/2

    # plot
    mp[scan_id[0,:], scan_id[1,:]] = 1
    f, ax = plt.subplots()
    im = ax.pcolormesh(mp)
    ax.scatter(pose_id.item(0), pose_id.item(1))
    ax.set_xlabel('y_G (x_M)')
    ax.set_ylabel('x_G (y_M)')
    ax.axis('equal')
    f.colorbar(im, ax=ax)
    f.suptitle('Map of scan points')
    pw.addPlot('Map', f)

    pw.show()


if __name__ == "__main__":
    import sys
    if len(sys.argv) > 1:
        testAll(int(sys.argv[1]))
    else:
        testAll(10)
