#! /usr/bin/env python3
# top level algorithm for occupancy grid mapping
import numpy as np
import matplotlib.pyplot as plt

from MotionModel import *
from map import *
from VehicleData import *
from copy import deepcopy
from particle import Particle

def plotMap(m, pos_cells):
    f, ax = plt.subplots(num=0)
    pos_cells_np = np.column_stack(pos_cells)

    # convert log-odds to probability
    pm = 1 - 1/(1 + np.exp(m.gridmap))

    im = ax.pcolormesh(pm.T)
    ax.axis('equal')
    f.colorbar(im, ax=ax)
    ax.plot(pos_cells_np[0,:], pos_cells_np[1,:], linewidth=1, c='r')
    
    # ax.axvline(x=m.center_cell[0], color='r')
    # ax.axhline(y=m.center_cell[1], color='r')


def runGMapping(fname):

    vd = loadFromFile(fname)

    actual_positions = []
    pos_cells = []
    times = []
    pos_truth_np = np.array(vd.position_truth)
 
    max_xy = max(np.amax(pos_truth_np), -np.amin(pos_truth_np))
    
    n_cells = int(2*(max_xy + GP.scan_max_xy + 1)/GP.resolution_m)
    # create map
    ogmap = Map(n_cells/2, n_cells, GP.resolution_m)

    N_iter = len(vd.lidar_data) -1
    print(N_iter)
    # for indx in range(0, 10, 5):
    for indx in range(0, N_iter):
    # for indx in range(0, N_iter, 25):
        scan  = vd.lidar_data[indx]
        time = vd.time_vec[indx]
        actual_pos = vd.position_truth[indx][0:2]
        actual_head = vd.heading_truth[indx]
        pose_t = np.concatenate([actual_pos, np.array([actual_head])])

        print("Processing {}/{}".format(indx, N_iter))

        # >>>> transform the scan to the body frame
        scan_bd = tfm.lidar2Body(scan.T)

        # threshold the scan
        scan_th = thresholdScan(scan_bd.T)

        # integrate the scan into the map
        ogmap, pose_cell = integrateScan(ogmap, scan_th.T, pose_t)
        
        print("actual = ", pose_t)
        actual_positions.append(pose_t)
        times.append(time)
        pos_cells.append(pose_cell)
        print("pos_cell = ", pose_cell.T)
    
        plotMap(ogmap, pos_cells)
        plt.savefig("ogmap/ogmap_{}.png".format(indx), dpi=600)
        # plt.show()
        plt.close()

    plotMap(ogmap, pos_cells)
    plt.show()

    print("Done")

if __name__ == '__main__':
    import sys
    if len(sys.argv) > 1:
        fname = sys.argv[1]
        runGMapping(fname)
    else:
        print('Please specify a filename')
