import numpy as np
from params import global_params as GP
import skimage.draw as skd
from scipy.ndimage.filters import gaussian_filter
import transforms as tfm

def transformScan(scan, motion):
    # motion is array [x,y,theta]
    x,y,theta = motion
    scan = np.copy(scan)

    rot_matrix = np.array([[np.cos(theta), -np.sin(theta), 0, x],
                           [np.sin(theta), np.cos(theta),  0, y], 
                           [0, 0, 1, 0],
                           [0, 0, 0, 1]])

    scan_aug = np.column_stack([scan, np.ones(scan.shape[0])])

    scan = np.matmul(rot_matrix, scan_aug.T).T

    return scan[:, :3]

def correlationFit(likelihood_map, scan, motion):

    scan = transformScan(scan, motion)

    score = 0
    for beam in scan:
        point_cell = likelihood_map._poseToMapIndex(np.array([beam.item(0), beam.item(1)]))
        if point_cell.item(0) < likelihood_map.gridmap.shape[0] and point_cell.item(1) < likelihood_map.gridmap.shape[1]:
            score += likelihood_map.gridmap[point_cell.item(0), point_cell.item(1)]

    return score

def probScan(scan, map, pose):
    # get portion of map that surrounds pose
    submap = map.getSubmap(pose[:2], GP.scan_max_xy)
    
    submap.gridmap = likelihoodField(submap.gridmap)

    # get correlationFit in submap
    log_odds = correlationFit(submap, scan, [0., 0., -pose[2]])

    # convert to probability
    p = 1 - 1/(1 + np.exp(log_odds))

    return p

def scanmatch(map, scan, pose):
    # returns the most likely pose that the scan was taken from

    # get portion of map that surrounds pose
    submap = map.getSubmap(pose[:2], GP.scan_max_xy)

    submap.gridmap = likelihoodField(submap.gridmap)

    shift = np.array( [0., 0., -pose[2]])
    best_fit = correlationFit(submap, scan, shift)
    last_improvement = 1
    xy_incr = 0.01 # m
    theta_incr = 2 * np.pi / 180.0
    best_shift = np.copy(shift)

 
    while best_fit < 0:
        
        dirs = np.array([[xy_incr, 0, 0],
                        [-xy_incr, 0, 0],
                        [0, xy_incr, 0],
                        [0, -xy_incr, 0],
                        [0,0,theta_incr], 
                        [0,0,-theta_incr]])

        fits = []
        for direction in fits:
            temp_shift = np.copy(shift)
            temp_shift += direction
            fit = correlationFit(submap, scan,  direction)
            if fit > best_fit:
                    best_fit = fit
                    best_shift = temp_shift
                        
        if(np.allclose(shift, best_shift)):
            break
        else:
            shift = best_shift
                    
    #print(best_fit)
    return pose + best_shift + np.array([0.0, 0.0, pose[2]]), True # TODO success is always true

def likelihoodField(map):
    # generate a likelihood field of a given map
    occupied_thresh = 0.0
    occupied = np.copy(map)

    blurred = gaussian_filter(occupied, sigma=GP.sigma_d)
    return blurred

def integrateScan(map, scan_bd, pose):
    # >>>> transform the body frame scan to the vehicle frame
    scan_vh = tfm.body2Vehicle(scan_bd, pose[2])
    # >>>> transform the vehicle frame scan to the global frame
    scan_gl = tfm.vehicle2Global(scan_vh, pose)

    # >>>> transform the global frame scan to map indices
    scan_id = tfm.global2Map(scan_gl, map.offset, map.dimension, map.resolution)
    pose_id = tfm.global2Map(pose[:2,np.newaxis], map.offset, map.dimension, map.resolution)

    for beam_id in range(scan_id.shape[1]):

        beampoint = scan_id[:,beam_id]

        # get cells along line between pose cells and beampoint cells
        beam_rr, beam_cc = skd.line(pose_id.item(0), pose_id.item(1),
                                    beampoint.item(0), beampoint.item(1))
        
        if scan_gl[2, beam_id] > GP.scan_low_z: 
            # this point cooresponds to an obstacle
            # remove point_cell from line
            beam_rr = beam_rr[:-1] 
            beam_cc = beam_cc[:-1]
            # set point cell to occupied
            map.gridmap[beampoint.item(0), beampoint.item(1)] += GP.ell_occ

        # mark beam line as free
        map.gridmap[beam_rr, beam_cc] = map.gridmap[beam_rr, beam_cc] + GP.ell_free

    return map, pose_id

def gridMapFromScan(scan, radius):
    # generate a local coordinate occupancy grid map given a lidar scan
    # the map will be n_cells X n_cells and the pose origin is at the center
    n_cells = 2*int(radius/GP.resolution_m)

    ogmap = Map(n_cells, np.array([0., 0.]))
    ogmap.gridmap = 0.0*ogmap.gridmap

    pose_cell = ogmap._poseToMapIndex(np.array([0, 0]))
    # for each beam, update the map
    import matplotlib.pyplot as plt

    for beam in scan:
        # convert global coordinates of beampoint to gridmap coordinates
        point_cell = ogmap._poseToMapIndex(np.array([beam.item(0), beam.item(1)]))

        # get cells along line between pose cells and beampoint cells
        beam_rr, beam_cc = skd.line(pose_cell.item(0), pose_cell.item(1), 
                                    point_cell.item(0), point_cell.item(1))

        ogmap.gridmap[beam_rr, beam_cc] = ogmap.gridmap[beam_rr, beam_cc] + GP.ell_free
        ogmap.gridmap[point_cell.item(0), point_cell.item(1)] = ogmap.gridmap[point_cell.item(0), point_cell.item(1)] + GP.ell_occ
    return ogmap

class Map:
    def __init__(self, offset, dimension, resolution):

        self.offset     = offset
        self.dimension  = dimension
        self.gridmap    = np.zeros((dimension, dimension))
        self.resolution = resolution
        
