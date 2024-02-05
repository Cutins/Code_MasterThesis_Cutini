import matplotlib.pyplot as plt
import sys, os
import signal
from matplotlib.animation import FuncAnimation
import numpy as np

cwd = os.getcwd()
sys.path.append(os.getcwd())

from MPC.Parameters import *
from MPC.Controller import *

ni = 1
nsp = 3

def main():
    params = Parameters()
    t_end = 18.0
    t_hor = 1.0
    num_steps = int(t_end/params.dt)
    num_steps_mpc = int(t_hor/params.dt)
    print("MPC predicts for", num_steps_mpc, "steps.")

    # Initialize controller
    MPC = MPCController(parameters=params)
    MPC.build_optcon_problem(t_hor=t_hor, dt=params.dt)

    # Collect vectors for plots
    xk = np.zeros((nsp, num_steps)) # x = [pk, vk]
    uk = np.zeros((ni, num_steps-1)) # u = uak

    h_ref = np.zeros((nsp, num_steps))
    h_ref[0] = params.offset_ref * np.ones(num_steps)

    # Initial conditions
    xk[0,0] = 9.70            # Initial position
    xk[1,0] = -2.18            # Initial velocity
    xk[2,0] = 0.09

    ar = 3.5 #3.86 #3.5 #0.08893

    u_max = np.ones(num_steps-1)
    u_min = np.zeros(num_steps-1)

    # Reference trajectory - Error variables # e = [(pk-pr)-2.5, vk-vr, 0]
    h_ref = np.zeros((nsp, num_steps))
    h_ref[0] = params.offset_ref * np.ones(num_steps)

    h_meas = np.zeros(nsp)
    h_meas = xk[:,0]

    h_opt, uk_opt = MPC.solve_optcon_problem(x_meas=h_meas, ar = ar, x_tar=h_ref[:,0])

    print("Optimal trajectory", h_opt)
    print("Optimal input", uk_opt)

if __name__ == "__main__":
    main()

