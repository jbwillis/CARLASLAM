# transformations for taking Nx3 vectors of data from one space to another

import numpy as np


def lidar2Body(points):
    # points - 3xN vector
    points = np.copy(points)
    R_L2B = np.array([ [  0., -1.,  0.],
                       [  1.,  0.,  0.],
                       [  0.,  0., -1.]])

    points_tr = R_L2B @ points

    return points_tr

def body2Vehicle(points, theta):
    # points - 3xN vector [x_B, y_B, z_B].T
    # theta - angle from vehicle frame to body frame (heading/yaw)
    print(theta)
    points = np.copy(points)
    c_th = np.cos(theta)
    s_th = np.sin(theta)
    R_B2V = np.array([[ c_th, -s_th, 0.],
                      [ s_th,  c_th, 0.],
                      [   0.,    0., 1.]])

    points_tr = R_B2V @ points

    return points_tr

def vehicle2Global(points, pose):
    # points - 3xN vector [x_B, y_B, z_B]
    # pose - the translation from the global origin to the vehicle
    points = np.copy(points)
    points[0, :] += pose[0]
    points[1, :] += pose[1]

    return points

def global2Map(points, mapOffset, mapDim, mapRes):
    # points - 3xN vector [x_B, y_B, z_B]
    pts_xy  = np.copy(points[:2,:])
    pts_xy /= mapRes
    pts_yx  = pts_xy[::-1, :] # swap x and y
    pts_map = pts_yx + np.array([[mapOffset, mapOffset]]).T 
    pts_map = pts_map.astype(int)

    # make sure these are valid coordinates
    if np.any(pts_map < 0):
        print("Error, negative coordinates")
        import pdb; pdb.set_trace()
    
    # make sure these are valid coordinates
    if np.any(pts_map >= mapDim):
        print("Error, large coordinates")
        import pdb; pdb.set_trace()

    return pts_map

