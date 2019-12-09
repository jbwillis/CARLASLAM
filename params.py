# parameters for CARLASLAM
import numpy as np


# Occupancy grid log-odds
p_occ    = .7;
ell_occ  = np.log(p_occ/(1-p_occ));
p_free   = .3;
ell_free = np.log(p_free/(1-p_free));
