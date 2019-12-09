import numpy as np
import params as P

def scanmatch(subliklihoodfield, scan, pose):
    # returns the most likely pose that the scan was taken from
    pass

def liklihoodField(map):
    # generate a liklihood field of a given map
    pass

def integrateScan(map, scan, pose_xy):
        pass

def gridMapFromScan(scan, resolution_m, radius):
    # generate a local coordinate occupancy grid map given a lidar scan
    # the map will be n_cells X n_cells and the pose origin is at the center
    n_cells = 2*int(radius/resolution_m)
    gridmap = np.zeros((n_cells, n_cells))

def _inverseSensorModel():
    pass
    
        


class map:
    def __init__(self, resolution_m, n_cells, global_origin):
        self.resolution_m = resolution_m

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
        pose_xy = coord*self.resolution_m
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

        
        coord = (pose_xy/self.resolution_m)
        coord = coord.astype(np.int)
        coord = coord + self.global_origin

        coord = coord[::-1] # flip x and y to coorespond to correct matrix coordinates
        
        return coord

