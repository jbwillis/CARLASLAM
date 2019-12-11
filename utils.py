# various utility functions

import numpy as np
import params as P

# convert a Vector3D to a numpy array
def vector3DToNp(v3d):
    return np.array([v3d.x, v3d.y, v3d.z]);

def wrapToPi(x):
    wrap = np.mod(x, 2*np.pi)
    if np.abs(wrap) > np.pi:
        wrap -= 2*np.pi*np.sign(wrap)
    return wrap

def thresholdScan(scan):
    # threshold scan to only have points in a region near the vehicle
    
    # threshold x values
    scan = scan[abs(scan[:,0]) < P.scan_max_xy, :]
    scan = scan[abs(scan[:,0]) > P.scan_min_xy, :]

    # threshold y values
    scan = scan[abs(scan[:,1]) < P.scan_max_xy, :]
    scan = scan[abs(scan[:,1]) > P.scan_min_xy, :]

    # threshold z values
    scan = scan[scan[:,2] < P.scan_max_z, :]
    scan = scan[scan[:,2] > P.scan_min_z, :]

    return scan

