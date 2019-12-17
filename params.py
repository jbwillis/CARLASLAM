# parameters for CARLASLAM
import numpy as np

class SingletonParent:
    # Borg - all instances share the state
    # from http://www.aleax.it/Python/5ep.html
    _shared_state = {}
    def __init__(self):
        self.__dict__ = self._shared_state

class Params(SingletonParent):
    def __init__(self):
        SingletonParent.__init__(self)
        #################################
        # Parameters - shared by everyone 
        
        # timestep
        self.Ts = .05

        # wheelbase
        self.wheelbase = 2

        # scan thresholding
        self.scan_max_xy = 80.
        self.scan_min_xy = 1.
        self.scan_max_z  = .5
        self.scan_low_z  = -2.0
        self.scan_min_z  = -2.5
        self.scan_low_z_min_xy = 30.

        # gaussian blur standard deviation
        self.sigma_d = 2

        # Occupancy grid log-odds
        p_occ    = .8
        self.ell_occ  = np.log(p_occ/(1-p_occ))
        p_free   = .49
        self.ell_free = np.log(p_free/(1-p_free))

        # resolution of all occupancy grid maps
        self.resolution_m = 1

        # Motion model noise
        alpha_1 = .005
        alpha_2 = .05
        self.alpha_v = np.array([alpha_1, alpha_2])

        alpha_3 = .005
        alpha_4 = .05
        self.alpha_g = np.array([alpha_3, alpha_4])

        # samples around the most likely scan pose
        self.sample_K     = 10
        self.sample_delta = .005

        # particle set related parameters
        self.N_particles = 25
        self.resample_threshold = self.N_particles/2



global_params = Params()
