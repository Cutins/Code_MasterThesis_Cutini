import matplotlib.pyplot as plt
import sys, os
import signal
from matplotlib.animation import FuncAnimation
import numpy as np

font = {'family' : 'serif',
        'weight' : 'normal',
        'size' : '16'}
plt.rc('font', **font)

cwd = os.getcwd()
sys.path.append(os.getcwd())

# Allow Crtl-C to work despite plotting
signal.signal(signal.SIGINT, signal.SIG_DFL)

from MPC_offset_free.Parameters import *
from MPC_offset_free.Controller import *
from MPC_offset_free.Utils import *
from MPC_offset_free.Estimator import *
from MPC_offset_free.Vehicle import *

ns = 2
ni = 1

nsp = 3

def main():
    params = Params()
    t_end = 12.0
    t_hor = 1.0
    num_steps = int(t_end/params.dt)
    num_steps_mpc = int(t_hor/params.dt)
    print("MPC predicts for", num_steps_mpc, "steps.")

    # Initialize controller
    MPC = MPCController(parameters=params)
    MPC.build_optcon_problem(t_hor=t_hor, dt=params.dt)

    # Collect vectors for plots
    xk = np.zeros((ns, num_steps)) # x = [pk, vk]
    uk = np.zeros((ni, num_steps-1)) # u = uak

    # Initial conditions
    xk[0,0] = 2.5            # Initial position
    xk[1,0] = 0.0            # Initial velocity

    mismatch = np.zeros((ns, num_steps))

    u_max = np.ones(num_steps-1)
    u_min = np.zeros(num_steps-1)

    # Reference trajectory - Error variables # e = [(pk-pr)-2.5, vk-vr, 0]
    h_ref = np.zeros((nsp, num_steps))
    h_ref[0] = params.offset_ref * np.ones(num_steps)

    # Measured error variables from lidar + acceleration runner estimated
    h_meas = np.zeros(nsp)

    # LQR profile
    speed_profile = [0, 7, 8, 9, 10, 10, 10]  
    time_profile = [0, 2, 4, 8, 10, 13, 16]
    init_distance = 5
    xk[0,0] = params.offset_ref + init_distance

        # Initialize State and Disturbance Estimator
    Est = Estimator(parameters=params)
    x_est = np.zeros((nsp, num_steps)) # Estimated state
    # x_est[0,0] = init_distance + params.offset_ref
    d_est = np.zeros((1, num_steps)) # Estimated disturbance

    position, velocity, acceleration = create_runner_profile(speed_profile, time_profile, params.dt, t_end)

    for t in range(0, num_steps-1):

        h_meas[0] = xk[0,t] - position[t]
        h_meas[1] = xk[1,t] - velocity[t]
        h_meas[2] = xk[1,t]

        ar = acceleration[t]

        # State and disturbance estimation
        Est.observer() # Build L matrix with poles-placement

        h_opt, uk_opt = MPC.solve_optcon_problem(x_est=x_est[:,t], d_est=d_est[:,t], ar = ar, u_max=u_max[t], u_min=u_min[t], h_ref=h_ref[:,t])

        # print("Optimal input: ", uk_opt[:,0])

        uk[:,t] = uk_opt[:,0] # Drive-train acceleration

        ## Forward simulate on non linear dynamics (real plant)
        xk[:,t+1] = kart_non_linear_dyn(xk[:,t], uk[:,t])

        ## Forward simulate on linear dynamics (no model mismatch)
        xlin = kart_linear_dyn(xk[:,t], uk[:,t])
        mismatch[:,t] = xk[:,t+1] - xlin

        x_est[:,t+1], d_est[:,t+1] = Est.estimate(x_est=x_est[:,t], d_est=d_est[:,t], ar=ar, h_meas=h_meas, u_t=uk[:,t])
        # print("Estimated disturbance at time", t, "is", d_est[:,t])

    ### Save trajectories for comparison
    Path = 'MPC_offset_free/'
    np.save(Path+'Error_traj_MPCOF.npy', np.vstack([xk[0]-position, xk[1]-velocity]).squeeze())
    np.save(Path+'Input_traj_MPCOF.npy', uk)

    time_hor = np.linspace(0, t_end, num=num_steps)
    time_hor_u = np.linspace(0, t_end-params.dt, num=num_steps-1)

    positive_mask_x = xk[0] - position > 2.5
    negative_mask_x = xk[0] - position < 2.5

    positive_mask_v = xk[1] - velocity > 0
    negative_mask_v = xk[1] - velocity < 0

    plt.figure("Absolute position", figsize=(10,8))
    plt.plot(time_hor, position, label='Runner position reference')
    plt.plot(time_hor, xk[0], label='Kart tracking position (closed loop)')
    plt.scatter(time_hor[0], xk[0,0], color='black', marker='o', label='Initial position')
    plt.xlabel('Time[s]')
    plt.ylabel('Position[m]')
    plt.legend()
    plt.grid(True)

    plt.figure("Absolute velocity", figsize=(10,8))
    plt.plot(time_hor, velocity, label='Runner velocity reference')
    plt.plot(time_hor, xk[1], label='Kart tracking velocity (closed loop)')
    plt.scatter(time_hor[0], xk[1,0], color='black', marker='o', label='Initial velocity')
    plt.xlabel('Time[s]')
    plt.ylabel(r'Velocity [$\frac{m}{s}$]')
    plt.legend()
    plt.grid(True)

    plt.figure("Drive-train acceleration", figsize=(10,8))
    plt.plot(time_hor_u, uk[0], label='Kart tracking acceleration (closed loop)', linewidth=2.0)
    plt.plot(time_hor_u, u_max, label='Acceleration limits', linestyle='-.', color='red', linewidth=1.0)
    plt.plot(time_hor_u, u_min, linestyle='-.', color='red', linewidth=1.0)
    plt.xlabel('Time[s]')
    plt.ylabel(r'Acceleration [$\frac{m}{s^2}$]')
    plt.legend()
    plt.grid(True)

    plt.figure("Position and velocity errors", figsize=(10,8))
    plt.subplot(2,1,1)
    plt.plot(time_hor, xk[0]-position, 'k-', label='Distance')
    plt.plot(time_hor, 2.5 * np.ones(num_steps), label='Reference', linestyle='--', color='red', linewidth=1.5)
    # plt.plot(time_hor, params.safe_distance * np.ones(num_steps), label='Safe distance', linestyle='-.', color='green', linewidth=1.5)
    plt.fill_between(time_hor, 2.5, xk[0] - position, where=positive_mask_x, facecolor='green', alpha=0.2, label='Offset > 2.5m')
    plt.fill_between(time_hor, 2.5, xk[0] - position, where=negative_mask_x, facecolor='red', alpha=0.2, label='Offset < 2.5m')
    plt.xlabel('Time[s]')
    plt.ylabel('Distance [m]')
    plt.legend(loc="upper right")
    plt.grid(True)

    plt.subplot(2,1,2)
    plt.plot(time_hor, xk[1]-velocity, 'k-', label='Relative velocity')
    plt.plot(time_hor, np.zeros(num_steps), label='Reference', linestyle='--', color='red', linewidth=1.5)
    plt.fill_between(time_hor, 0, xk[1] - velocity, where=positive_mask_v, facecolor='green', alpha=0.2, label='Kart faster')
    plt.fill_between(time_hor, 0, xk[1] - velocity, where=negative_mask_v, facecolor='red', alpha=0.2, label='Kart slower')
    plt.xlabel('Time[s]')
    plt.ylabel(r'Velocity [$\frac{m}{s}$]')
    plt.legend()
    plt.grid(True)

    plt.figure("Observed disturbance", figsize=(10,8))
    plt.plot(time_hor, mismatch[1], 'k-', label='MPM', linewidth=2.0)
    plt.plot(time_hor, d_est[0], label='Estimated', linestyle='--', color='red', linewidth=1.5)
    plt.xlabel('Time[s]')
    plt.ylabel(r'Disturbance [$\frac{m}{s^2}$]')
    plt.legend()
    plt.grid(True)

    plt.figure("Real State VS Estimated State", figsize=(10,8))
    plt.subplot(3,1,1)
    plt.plot(time_hor, xk[0]-position, 'k-', label='Real')
    plt.plot(time_hor, x_est[0], label='Estimated', linestyle='--', color='red', linewidth=1.5)
    plt.xlabel('Time[s]')
    plt.ylabel('Distance [m]')
    plt.legend(loc="upper right")
    plt.grid(True)

    plt.subplot(3,1,2)
    plt.plot(time_hor, xk[1]-velocity, 'k-', label='Relative velocity')
    plt.plot(time_hor, x_est[1], label='Estimated', linestyle='--', color='red', linewidth=1.5)
    plt.xlabel('Time[s]')
    plt.ylabel(r'Velocity [$\frac{m}{s}$]')
    plt.legend()
    plt.grid(True)

    plt.subplot(3,1,3)
    plt.plot(time_hor, xk[1], 'k-', label='Relative velocity')
    plt.plot(time_hor, x_est[2], label='Estimated', linestyle='--', color='red', linewidth=1.5)
    plt.xlabel('Time[s]')
    plt.ylabel(r'Velocity [$\frac{m}{s}$]')
    plt.legend()
    plt.grid(True)
        

    plt.show()









if __name__ == "__main__":
    main()