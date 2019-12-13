# propagate the motion model for the vehicle
import numpy as np
from utils import wrapToPi, prob_normal_distribution

from params import global_params as GP

def modelStep(state_km1, odom):
    L  = GP.wheelbase
    Ts = GP.Ts

    x_km1     = state_km1[0]
    y_km1     = state_km1[1]
    theta_km1 = state_km1[2]

    v         = odom[0]
    gamma     = odom[1]

    x_k = x_km1 + v * np.cos(theta_km1) * Ts
    y_k = y_km1 + v * np.sin(theta_km1) * Ts
    theta_k = theta_km1 + (v/L)*np.tan(gamma) * Ts

    theta_k = wrapToPi(theta_k)

    return [x_k, y_k, theta_k]

def sampleMotionModel(state_km1, odom):
    # generate noisy odometry
    sig_v = np.sqrt(GP.alpha_v @ odom**2)
    sig_g = np.sqrt(GP.alpha_g @ odom**2)

    odom_noise = odom + np.diag([sig_v, sig_g]) @ np.random.randn(2)

    return modelStep(state_km1, odom_noise)

def probMotionModel(state_k, state_km1, odom):
    # compute the probability
    # p(x_k | x_km1, u_k)
    # Similar to table 5.1 in Probabilistic Robotics

    L  = GP.wheelbase
    Ts = GP.Ts

    x_k       = state_k[0]
    y_k       = state_k[1]
    theta_k   = state_k[2]

    x_km1     = state_km1[0]
    y_km1     = state_km1[1]
    theta_km1 = state_km1[2]

    v         = odom[0]
    gamma     = odom[1]

    v_hat = (1/Ts) * np.sqrt((x_k - x_km1)**2 + (y_k - y_km1)**2)

    theta_diff = wrapToPi(theta_k - theta_km1)
    gamma_hat = np.arctan2(L*theta_diff, v_hat*Ts)

    gamma_diff = wrapToPi(gamma - gamma_hat)
    v_diff = v - v_hat

    var_v = GP.alpha_v @ odom**2
    var_g = GP.alpha_g @ odom**2

    p = prob_normal_distribution(v_diff, var_v) * prob_normal_distribution(gamma_diff, var_g)

    return p


