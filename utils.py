# various utility functions

# try:
    # sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        # sys.version_info.major,
        # sys.version_info.minor,
        # 'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
# except IndexError:
    # pass

# import carla
import numpy as np

# convert a Vector3D to a numpy array
def vector3DToNp(v3d):
    return np.array([v3d.x, v3d.y, v3d.z]);

def wrapToPi(x):
    wrap = np.mod(x, 2*np.pi)
    if np.abs(wrap) > np.pi:
        wrap -= 2*np.pi*np.sign(wrap)
    return wrap
