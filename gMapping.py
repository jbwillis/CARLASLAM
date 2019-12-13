#! /usr/bin/env python3
# top level algorithm for gMapping SLAM
import numpy as np
import matplotlib.pyplot as plt

from MotionModel import *
from map import *
from VehicleData import *
from copy import deepcopy

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
        x_t_hat, success = scanmatch(particle.map, scan_t, x_t_pr)

        if not success:
            # scanmatch didn't find an alignment in the map
            particle.pose = sampleMotionModel(particle.pose, odom_tm1)
            particle.weight = particle.weight * probScan(scan_t, particle.map, particle.pose)
        else:

            # sample around the mode
            xK = (np.random.rand(GP.sample_K, 3) - .5)*GP.sample_delta + x_t_hat

            # compute Gaussian proposal

            # to avoid calling probMotionModel and probScan more than once
            # per xj, compute an array of those first
            pK = np.zeros(GP.sample_K)

            for ii in range(GP.sample_K):
                pK[ii] = probMotionModel(xK[ii,:], particle.pose, odom_tm1) \
                        * probScan(scan_t, particle.map, xK[ii,:]) 

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
            particle.pose = np.random.multivariate_normal(mu, Sig)

            # update importance weights
            particle.weight *= eta

        # update map
        particle.map = integrateScan(particle.map, scan_t, particle.pose)

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

def resampleParticleSet(particle_set):
    # low variance sampler
    # particle set is assumed to already have normalized weights

    new_particle_set = []

    M = len(particle_set) # number of particles
    r = np.random.rand()/M # uniformly distribute r in (0, 1/M)
    c = particle_set[0].weight
    i = 1
    indx = []

    for m in range(M):
        U = r + m/M
        while U > c: # accumulate weights
            i = i + 1
            c = c + particle_set[i].weight

        # line 12, add x_t[i] to X_bar
        new_particle_set.append(deepcopy(particle_set[i]))
        indx.append(i)

    return new_particle_set

def intiParticleSet(N_particles):
    particle_set = []
    for i in range(N_particles):
        particle_set.append([0., 0., 0.], 1., map(1000, [0., 0., 0.]))

    return particle_set

def runGMapping():

    vd = loadFromFile("captured_data/route1_vd.npz")

    particle_set = intiParticleSet(GP.N_particles)

    for scan, vel, steer in zip(vd.lidar_data, vd.velocity_data, vd.steer_data):
        odom = [vel, steer]
        particle_set = runStep(particle_set, scan, odom)

    f = plt.figure()
    plt.pcolormesh(particle_set[0].map.gridmap)
    plt.axis('equal')

    print("Done")

if __name__ == '__main__':
    runGMapping()
