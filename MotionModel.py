# propagate the motion model for the vehicle
import numpy as np

def modelStep(state_km1, v, gamma, L, Ts):

    x_km1     = state_km1[0]
    y_km1     = state_km1[1]
    theta_km1 = state_km1[2]

    x_k = x_km1 + v * np.cos(theta_km1) * Ts
    y_k = y_km1 + v * np.sin(theta_km1) * Ts
    theta_k = theta_km1 + (v/L)*np.tan(gamma) * Ts

    return [x_km1, y_km1, theta_km1]

