#! /usr/bin/env python3
# top level algorithm for occupancy grid mapping
import numpy as np
import matplotlib.pyplot as plt

from MotionModel import *
from map import *
from VehicleData import *
from copy import deepcopy
from particle import Particle

def plotResults(times, belief_states, actual_states):
    plt.figure()
    plt.plot(times, actual_states[:, 0], "red", alpha=.5, label="Actual X")
    plt.plot(times, belief_states[:, 0], "blue", alpha=.5, label="Belief X")
    plt.legend()

    plt.figure()
    plt.plot(times, actual_states[:, 1], "red", alpha=.5, label="Actual Y")
    plt.plot(times, belief_states[:, 1], "blue", alpha=.5, label="Belief Y")
    plt.legend()

    plt.figure()
    plt.plot(times, actual_states[:, 2], "red", alpha=.5, label="Actual theta")
    plt.plot(times, belief_states[:, 2], "blue", alpha=.5, label="Belief theta")
    plt.legend()

    plt.show()

def plotMap(m, pos_cells):
    f, ax = plt.subplots()
    pos_cells_np = np.array([pos_cells]).T

    # convert log-odds to probability
    pm = 1 - 1/(1 + np.exp(m.gridmap))

    im = ax.pcolormesh(pm)
    ax.axis('equal')
    f.colorbar(im, ax=ax)
    ax.plot(pos_cells_np[1,:], pos_cells_np[0,:], linewidth=3, c='r')
    
    # ax.axvline(x=m.center_cell[0], color='r')
    # ax.axhline(y=m.center_cell[1], color='r')


def runGMapping():

    vd = loadFromFile("captured_data/route1_vd.npz")

    actual_positions = []
    pos_cells = []
    times = []
    pos_truth_np = np.array(vd.position_truth)
 
    max_xy = max(np.amax(pos_truth_np), -np.amin(pos_truth_np))
    
    n_cells = int(2*(max_xy + GP.scan_max_xy + 1)/GP.resolution_m)
    # create map
    ogmap = Map(n_cells, np.array([0., 0.]))

    N_iter = len(vd.lidar_data) -1
    print(N_iter)
    # for indx in range(0, 10, 5):
    for indx in range(50, N_iter):
    # for indx in [0, 25, 50, 75, 100]:
        scan  = vd.lidar_data[indx]
        time = vd.time_vec[indx]
        actual_pos = vd.position_truth[indx][0:2]
        actual_head = vd.heading_truth[indx]
        pose_t = np.concatenate([actual_pos, np.array([actual_head])])

        print("Processing {}/{}".format(indx, N_iter))

        # transform scan into vehicle frame
        scan = np.column_stack([-scan[:,1], -scan[:,0], scan[:,2]])

        # threshold the scan
        scan_t = thresholdScan(scan)

        # ogmap = Map(n_cells, np.array([0., 0.]))
        # integrate the scan into the map
        ogmap, pose_cell = integrateScan(ogmap, scan_t, pose_t)
        
        print("actual = ", pose_t)
        actual_positions.append(pose_t)
        times.append(time)
        pos_cells.append(pose_cell)
        print("pos_cell = ", pose_cell)
    
        # plotMap(ogmap, pos_cells)
        # plt.savefig("ogmap/ogmap_{}.png".format(indx))

    plotMap(ogmap, pos_cells)
    plt.show()

    actual_positions = np.array(actual_positions)
    times = np.array(times)
    # plotResults(times, belief_positions, actual_positions)

    print("Done")

if __name__ == '__main__':
    runGMapping()
