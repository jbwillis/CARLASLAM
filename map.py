import numpy as np
from params import global_params as GP
import skimage.draw as skd
from scipy.ndimage.filters import gaussian_filter


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
        point_cell = map._poseToMapIndex(np.array([beam.item(0), beam.item(1)]))
        score += likelihood_map.gridmap[point_cell]

    return score

def scanmatch(subliklihoodfield, scan, pose):
    # returns the most likely pose that the scan was taken from
    # searches within
    pass



def likelihoodField(map):
    # scan is a set of points
    # generate a likelihood field of a given map
    occupied_thresh = 1.0
    occupied = np.copy(map)
    occupied[occupied < occupied_thresh] = 0.0
    occupied[occupied >= occupied_thresh] = 1.0

    blurred = gaussian_filter(occupied, sigma=GP.sigma_d)
    blurred[blurred > 1] = 1.0
    return blurred

def integrateScan(map, scan, pose_xy):
    for beam in scan:
        # convert global coordinates of beampoint to gridmap coordinates
        point_cell = map._poseToMapIndex(np.array([beam.item(0), beam.item(1)]))

        pose_cell = map._poseToMapIndex(pose_xy)

        # get cells along line between pose cells and beampoint cells
        beam_rr, beam_cc = skd.line(pose_cell.item(0), pose_cell.item(1),
                                    point_cell.item(0), point_cell.item(1))

        map.gridmap[beam_rr, beam_cc] = map.gridmap[beam_rr, beam_cc] + GP.ell_free
        map.gridmap[point_cell.item(0), point_cell.item(1)] = map.gridmap[
                                                                    point_cell.item(0), point_cell.item(1)] + GP.ell_occ

def gridMapFromScan(scan, radius):
    # generate a local coordinate occupancy grid map given a lidar scan
    # the map will be n_cells X n_cells and the pose origin is at the center
    n_cells = 2*int(radius/GP.resolution_m)

    ogmap = Map(n_cells, np.array([n_cells/2, n_cells/2]))
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
    def __init__(self, n_cells, global_origin):
        # origin of map with respect to global robot coordinate frame
        self.global_origin  = global_origin # integer

        self.gridmap      = np.ones((n_cells, n_cells))

    def getSubmap(self, centerpoint, radius):
        # given a centerpoint (in the robot's global coordinates)
        # and a square radius (in meters)
        # return the portion of the map containing those cells

        # convert centerpoint to index 
        top_right = self._poseToMapIndex(centerpoint + radius)
        bot_left  = self._poseToMapIndex(centerpoint - radius)

        # get submap
        submap = self.gridmap[bot_left.item(0):top_right.item(0), bot_left.item(1):top_right.item(1)]
        
        return submap
        

    def _mapIndexToPose(self, coord_yx):
        # center of map is pose (0,0)
        # transform map coordinates to pose
        # returns (x, y) pose of center of cell
        # x in pose = 2nd map axis
        # y in pose = 1st map axis

        if not (coord_yx.size == 2):
            print("Error: coord_yx is wrong dimension")
            import pdb; pdb.set_trace()

        coord = coord_yx[::-1] # flip x and y to coorespond to correct global coordinates
        
        coord = coord - self.global_origin
        pose_xy = coord*GP.resolution_m
        pose_xy = pose_xy.astype(np.double)

        return pose_xy
    
    def _poseToMapIndex(self, pose_xy):
        # center of map is pose (0,0)
        # transform pose to map coordinates
        # x in pose = 2nd map axis
        # y in pose = 1st map axis

        if not (pose_xy.size == 2):
            print("Error: pose_xy is wrong dimension")
            import pdb; pdb.set_trace()

        
        coord = (pose_xy/GP.resolution_m)
        coord = coord.astype(np.int)
        coord = coord + self.global_origin.astype(np.int)

        coord = coord[::-1] # flip x and y to coorespond to correct matrix coordinates
        
        return coord

