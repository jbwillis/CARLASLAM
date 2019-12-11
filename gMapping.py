# top level algorithm for gMapping SLAM
from MotionModel import modelStep
from map import *

def runStep(particle_set_tm1, scan_t, odom_tm1):
    
    particle_set_t  = []

    # convert scan into occupancy grid map

    gridMapFromScan(scan, radius):

    for particle in particle_set_tm1:
        
        # particle has:
        # particle.pose
        # particle.weight
        # particle.map

        #################################
        # scan matching
        x_t_pr  = modelStep(particle.pose, odom_tm1) # propagate the particle's state forward

        x_t_hat = scanmatch(subliklihoodfield, scan_t, x_t_pr)





