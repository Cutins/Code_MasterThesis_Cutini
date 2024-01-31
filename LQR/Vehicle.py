import numpy as np
import math

from LQR.Parameters import *

params = Parameters()

def kart_linear_dyn(x_t, u_t):
    """ Updates kart state according to linear model approximation given actual state x_t and actual input u_t """
    
    x_next = params.A @ x_t + params.B @ u_t

    return x_next

def kart_non_linear_dyn(x_t, u_t):
    """ Updates kart state according to non linear bicycle model """

    sign_C_roll = 1
    if x_t[1] < 0:
        sign_C_roll = -1


    Fx = params.Cm1 * u_t[0] - params.Cd * x_t[1]**2 - (params.Croll*sign_C_roll)

    # beta = math.atan2(math.tan(steer)*params.lr, params.lf+params.lr)
    # x_dot = x_t[1]*math.cos(phi+beta)
    # y_dot = x_t[1]*math.sin(phi+beta)
    # phi_dot = x_t[1]*math.sin(beta)/params.lr
    # v_dot = u_t/params.m

    x_plus = x_t[0] + params.dt * x_t[1]
    v_plus = x_t[1] + (params.dt/params.m) * Fx
    l_plus = 1.0

    # x_rate = np.vstack([x_dot, y_dot, phi_dot, v_dot])

    # x_next = x_t + (np.vstack([x_rate[0], x_rate[3]]) * params.dt).squeeze()

    x_next = np.vstack([x_plus, v_plus, l_plus]).squeeze()

    return x_next