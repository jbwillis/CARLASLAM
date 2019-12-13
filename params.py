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
        self.scan_max_xy = 80
        self.scan_min_xy = .1
        self.scan_max_z  = 1
        self.scan_min_z  = -1.5
        self.sigma_d = 0.01

        # Occupancy grid log-odds
        p_occ    = .7
        self.ell_occ  = np.log(p_occ/(1-p_occ))
        p_free   = .3
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
        self.sample_delta = .5


global_params = Params()
