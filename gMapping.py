# top level algorithm for gMapping SLAM
from MotionModel import *
from map import *
from VehicleData import *

def runStep(particle_set, scan_t, odom_tm1):

    # convert scan into occupancy grid map

    #gridMapFromScan(scan, radius)

    weights_total = 0 # keep track of all weights for normalization

    for particle in particle_set:

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
            particle.pose = sampleMotionModel(particle.pose, odom_tm1)
            # particle.weight = particle.weight * TODO
        else:

            # sample around the mode
            xK = (np.random.rand(GP.sample_K, 3) - .5)*GP.sample_delta + x_t_hat

            # compute Gaussian proposal

            # to avoid calling probMotionModel and probScan more than once
            # per xj, compute an array of those first
            pK = np.zeros(GP.sample_K)

            for ii in range(GP.sample_K):
                pK[ii] = probMotionModel(xK[ii,:], particle.pose, odom_tm1) \
                        # * probScan() TODO

            mu  = xK * pK[:,np.newaxis] # multiply each pj by it's respective pj
            eta = np.sum(pK)

            mu /= eta

            Sig = np.zeros((3,3))

            for ii in range(GP.sample_K):
                xj = xK[ii,:]
                diff = xj - mu

                Sig += p[ii] * (diff * diff[:,np.newaxis])

            Sig /= eta

            # sample new pose
            # TODO
            # particle.pose = 

            # update importance weights
            particle.weight *= eta

        # update map
        # TODO: Verify this function call/return value
        particle.map.gridmap = integrateScan(particle.map.gridmap, particle.pose, scan_t)

        # sum each particles weight
        weights_total += particle.weight

    # normalize particle weights
    for particle in particle_set:
        particle.weight /= weights_total

    # determine if particles should be resampled
    N_eff = computeEffectiveSampleSize(particle_set)

    if N_eff < GP.resample_threshold:
        particle_set = resampleParticleSet(particle_set)

    return particle_set

def computeEffectiveSampleSize(particle_set):
    # effective sample size N_eff
    weight_sq = 0
    for particle in particle_set:
        weight_sq += (particle.weight)**2

    N_eff = 1/weight_sq

    return N_eff

def runGMapping():

    vd = loadFromFile("captured_data/route1_vd.npz")

    map = gridMapFromScan(vd.lidar_data[20], 100.0)

    print("Done")

if __name__ == '__main__':
    runGMapping()
