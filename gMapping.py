#! /usr/bin/env python3
# top level algorithm for gMapping SLAM
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

def runStep(particle_set, scan_t, odom_tm1):

    # threshold the scan
    scan_t = thresholdScan(scan_t)

    weights_total = 0 # keep track of all weights for normalization

    for particle in particle_set:

        # particle has:
        # particle.pose
        # particle.weight
        # particle.map

        #################################
        # scan matching
        x_t_pr  = modelStep(particle.pose, odom_tm1) # propagate the particle's state forward
        # x_t_hat, success = scanmatch(particle.map, scan_t, x_t_pr)
        x_t_hat = x_t_pr
        success = True

        
        if not success:
            # scanmatch didn't find an alignment in the map
            particle.pose = sampleMotionModel(particle.pose, odom_tm1)
            particle.weight = particle.weight * probScan(scan_t, particle.map, particle.pose)
        else:

            # sample around the mode
            xK = (np.random.rand(GP.sample_K, 3) - .5)*(GP.sample_delta*odom_tm1[0]**2) + x_t_hat
            

            # compute Gaussian proposal

            # to avoid calling probMotionModel and probScan more than once
            # per xj, compute an array of those first
            pK = np.zeros(GP.sample_K)

            for ii in range(GP.sample_K):
                pK[ii] = probMotionModel(xK[ii,:], particle.pose, odom_tm1) #\
                        # + probScan(scan_t, particle.map, xK[ii,:])

            mu  = xK * pK[:,np.newaxis] # multiply each pj by it's respective pj
            mu  = np.sum(mu, 0)
            eta = np.sum(pK)

            eta = max(1e-5, eta)
            mu /= eta

            Sig = np.zeros((3,3))

            for ii in range(GP.sample_K):
                xj = xK[ii,:]
                diff = xj - mu

                Sig += pK[ii] * (diff * diff[:,np.newaxis])

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

    # if N_eff < GP.resample_threshold:
        # particle_set = resampleParticleSet(particle_set)

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

    print("Resampling")

    new_particle_set = []

    M = len(particle_set) # number of particles
    r = np.random.rand()/M # uniformly distribute r in (0, 1/M)
    c = particle_set[0].weight
    i = 0
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

def initParticleSet(N_particles, pose0):
    particle_set = []
    for i in range(N_particles):
        particle_set.append(Particle(pose0, 1., Map(1000, np.array([0., 0.]))))

    return particle_set

def runGMapping():

    vd = loadFromFile("captured_data/route1_vd.npz")

    heading_truth_0 = vd.heading_truth[0]
    particle_set = initParticleSet(GP.N_particles, np.array([0., 0., heading_truth_0]))

    # N_iter = len(vd.velocity_data)
    #N_iter = len(vd.lidar_data)

    actual_positions = []
    belief_positions = []
    times = []

    gamma_prev = 0
    alpha = .5

    N_iter = len(vd.lidar_data)
    for indx in range(20, 50):
        scan  = vd.lidar_data[indx]
        vel   = vd.velocity_data[indx]
        steer = vd.steer_data[indx]
        time = vd.time_vec[indx]
        actual_pos = vd.position_truth[indx][0:2]
        actual_head = vd.heading_truth[indx]

        actual_pos = np.concatenate([actual_pos, np.array([actual_head])])

        print("Processing {}/{}".format(indx, N_iter))

        v = np.linalg.norm(vel[0:2])
        gamma = np.deg2rad(steer)

        ## filter gamma
        if (gamma - gamma_prev) > np.pi/8:
            gamma = gamma_prev + np.pi/8
        elif (gamma - gamma_prev) < -np.pi/8:
            gamma = gamma_prev - np.pi/8

        gamma = alpha*gamma + (1 - alpha)*gamma_prev

        odom = np.array([v, gamma])
        particle_set = runStep(particle_set, scan, odom)

        belief_pos = sum([particle.pose[0:3]*particle.weight for particle in particle_set])
        print("belief = ", belief_pos)
        print("actual = ", actual_pos)
        actual_positions.append(actual_pos)
        belief_positions.append(belief_pos)
        times.append(time)

    # f = plt.figure()
    # plt.pcolormesh(particle_set[0].map.gridmap)
    # plt.axis('equal')
    # plt.show()
    actual_positions = np.array(actual_positions)
    belief_positions = np.array(belief_positions)
    times = np.array(times)
    plotResults(times, belief_positions, actual_positions)

    print("Done")

if __name__ == '__main__':
    runGMapping()
