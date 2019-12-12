# top level algorithm for gMapping SLAM
from MotionModel import modelStep
from map import *
from VehicleData import *

def runStep(particle_set_tm1, scan_t, odom_tm1):
    
    particle_set_t  = []

    # convert scan into occupancy grid map

    #gridMapFromScan(scan, radius)

    for particle in particle_set_tm1:
        
        # particle has:
        # particle.pose
        # particle.weight
        # particle.map

        #################################
        # scan matching
        x_t_pr  = modelStep(particle.pose, odom_tm1) # propagate the particle's state forward

        x_t_hat, success = scanmatch(subliklihoodfield, scan_t, x_t_pr)

        if not success:
            # scanmatch didn't find an alignment in the map
            # new_pose =  


def runGMapping():

    vd = loadFromFile("captured_data/route1_vd.npz")

    map = gridMapFromScan(vd.lidar_data[20], 100.0)

    print("Done")

if __name__ == '__main__':
    runGMapping()
