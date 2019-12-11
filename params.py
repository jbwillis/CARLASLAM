# parameters for CARLASLAM
import numpy as np

# scan thresholding
scan_max_xy = 80
scan_min_xy = .1
scan_max_z  = 1
scan_min_z  = -1.5

# Occupancy grid log-odds
p_occ    = .7;
ell_occ  = np.log(p_occ/(1-p_occ));
p_free   = .3;
ell_free = np.log(p_free/(1-p_free));
