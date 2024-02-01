import numpy as np
import math

from MPC_offset_free.Parameters import *

params = Params()

def kart_linear_dyn(x_t, u_t):
    """ Updates kart state according to linear model approximation given actual state x_t and actual input u_t """
    
    x_next = params.A @ x_t + params.B @ u_t

    return x_next

def kart_non_linear_dyn(x_t, u_t):
    """ Updates kart state according to non linear bicycle model with rectilinear motion approximation"""

    sign_C_roll = 1
    if x_t[1] < 0:
        sign_C_roll = -1


    Fx = params.Cm1 * u_t[0] - params.Cd * x_t[1]**2 - (params.Croll*sign_C_roll)

    x_plus = x_t[0] + params.dt * x_t[1]
    v_plus = x_t[1] + (params.dt/params.m) * Fx

    x_next = np.vstack([x_plus, v_plus]).squeeze()

    return x_next