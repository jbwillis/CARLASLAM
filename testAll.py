#! /usr/bin/env python3

# collection of functions used at some point to test something

import matplotlib.pyplot as plt
import numpy as np

from VehicleData import *
from map import *
from utils import *
from plotWindow.plotWindow import plotWindow
from MotionModel import *

def testTransformScanPlot(pw, vd): 
    scan = thresholdScan(vd.lidar_data[10])
    scanR1 = map.transformScan(scan, [0, 0, np.pi/4])
    scanR2 = map.transformScan(scan, [0, 0, -np.pi/2])

    pw.addPlot("Lidar", vd._plotLidarScan(scan))
    pw.addPlot("Lidar + pi/4", vd._plotLidarScan(scanR1))
    pw.addPlot("Lidar - pi/2", vd._plotLidarScan(scanR2))

def testSampleMotionModel(pw):
    # generate 250 random samples for different values of 
    # alpha noise parameters
    
    # default
    samps = np.zeros((1000, 3))
    for i in range(0, samps.shape[0]):
        samp1      = sampleMotionModel(np.array([0., 0., 0.]), np.array([10, np.pi/4]))
        samps[i,:] = sampleMotionModel(samp1, np.array([10, np.pi/4]))
    f = plt.figure()
    plt.scatter(samps[:,0], samps[:,1], c=samps[:,2])
    plt.axis('equal')

    pw.addPlot("MM Default Noise", f)

def testProbMotionModel(pw):
    # generate 250 random samples for different values of 
    # alpha noise parameters
    
    # default
    samps = np.zeros((100, 100))
    pose0 = np.array([0., 0., np.pi/4])
    for rr in range(0, samps.shape[0]):
        for cc in range(0, samps.shape[1]):
            x = cc*2./100. -1
            y = rr*2./100. -1
            theta = np.arctan2(y, x)
            samps[rr, cc] = probMotionModel(np.array([x, y, theta]), pose0, np.array([10, 0.]))
        
    f = plt.figure()
    plt.pcolormesh(np.abs(samps))
    plt.axis('equal')

    pw.addPlot("MM Prob", f)

def testRotateScan(vd): 
    scan = thresholdScan(vd.lidar_data[10])
    scanR1 = map.transformScan(scan, [0, 0, np.pi/4])
    scanR2 = map.transformScan(scanR1, [0, 0, -np.pi/4])

    # scanR2 should be identical to scan
    np.testing.assert_array_almost_equal(scan, scanR2)

def testTransformScan(vd): 
    scan = thresholdScan(vd.lidar_data[10])
    scanR1 = map.transformScan(scan, [10, -1.2, 3*np.pi/7])

    # reverse must be done in two steps
    scanR2 = map.transformScan(scanR1, [-10, 1.2, 0])
    scanR3 = map.transformScan(scanR2, [0, 0, -3*np.pi/7])

    # scanR3 should be identical to scan
    np.testing.assert_array_almost_equal(scan, scanR3)

def testLikelihoodField(pw):
    m = np.zeros([100, 100])
    m[20:50, 60:95] = 10
    m[45:70, 5:30] = 10
    
    f = plt.figure()
    plt.pcolormesh(m)
    plt.axis('equal')

    pw.addPlot("Solid Map", f)

    blurred_m = likelihoodField(m)

    f = plt.figure()
    plt.pcolormesh(blurred_m)
    plt.axis('equal')

    pw.addPlot("blurred", f)

def testIntegrateScan(vd, pw):
    scan  = vd.lidar_data[10]
    scan_t = thresholdScan(scan)
    m_empty = Map(1000, np.array([0., 0.]))
    m = integrateScan(m_empty, scan_t, np.array([0., 0., 0.]))

    f, ax = plt.subplots()
    im = ax.pcolormesh(m.gridmap)
    f.colorbar(im, ax=ax)
    ax.axis('equal')

    pw.addPlot("Raw scan", vd._plotLidarScan(scan))
    pw.addPlot("Integrated Scan", f)

def testScanMatching(pw):
    testScan = np.array([ [10., 5., 0.], 
                          [10., 6., 0.], 
                          [10., 7., 0.], 
                          [10., 8., 0.],  
                          [10., 9., 0.],
                          [10., 10., 0.],
                          [5., 10., 0.], 
                          [6., 10., 0.], 
                          [7., 10., 0.], 
                          [8., 10., 0.], 
                          [9., 10., 0.]])


if __name__ == '__main__':
      
    vd = loadFromFile("captured_data/route1_vd.npz")

    
    testRotateScan(vd)
    testTransformScan(vd)

    # Plotting related tests
    pw = plotWindow()
    testIntegrateScan(vd, pw)
    testTransformScanPlot(pw, vd)
    testSampleMotionModel(pw)
    testProbMotionModel(pw)
    testLikelihoodField(pw)

    pw.show()
