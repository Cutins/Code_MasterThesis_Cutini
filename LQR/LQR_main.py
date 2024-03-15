import matplotlib.pyplot as plt
font = {'family' : 'serif',
        'weight' : 'normal',
        'size' : '12'} # 16
plt.rc('font', **font)
from matplotlib.animation import FuncAnimation

import sys, os
import signal
import time as ttime

cwd = os.getcwd()
sys.path.append(os.getcwd())

# Allow Crtl-C to work despite plotting
signal.signal(signal.SIGINT, signal.SIG_DFL)

from LQR.Parameters import *
from LQR.Controller import *
from LQR.Utils import *
from LQR.Vehicle import *

ns = 2 # xk = [pk, vk]
ni = 1 # uk = uak (drive-train acceleration)

Calculate_opt = False # True, False
Animation = False # True, False
Save_animation = False # True, False

def main():
    params = Parameters()
    t_end = 15
    num_steps = int(t_end/params.dt)
    
    # Create profile for runner position and velocity
    # Profile with saturation
    # speed_profile = [0, 2, 7, 8, 9, 10, 10, 10]  
    # time_profile = [0, 1, 2, 4, 8, 10, 13, 16]
    # init_distance = 5 

    # Usain Bolt
    # speed_profile = [0, 8, 11, 12, 12, 12, 12]  
    # time_profile = [0, 2, 4, 8, 10, 13, 16]
    # init_distance = 12

    # No saturation profile
    # speed_profile = [0, 2, 4.5, 6, 7, 7, 7, 7]  
    # time_profile = [0, 1, 2, 4, 8, 10, 13, 16]
    # init_distance = 0.5

    # Mujinga profile
    time_profile = [0, 2.03, 3.17, 4.21, 5.21, 6.21, 7.21, 8.23, 9.25, 10.30, 11.41,]
    speed_profile = [0, 4.93, 8.77, 9.62, 10, 10, 10, 9.80, 9.80, 9.52, 9]
    init_distance = 6

    # speed_profile = [0, 2, 4.5, 9, 9, 9, 9, 9]  
    # time_profile = [0, 1, 2, 4, 8, 10, 13, 16]
    # init_distance = 2

    # Marie-Josèe Ta-Lou (Diamond League Lausanne 30th June 2023)
    time_profile = [0, 1.97, 3.1, 4.11, 5.08, 6.03, 6.99, 7.94, 8.9, 9.88, 10.88]
    speed_profile = [0, 5.07, 8.85, 9.9, 10.31, 10.53, 10.42, 10.53, 10.42, 10.2, 10]
    init_distance = 4

    # # Christian Coleman (Diamond League Eugene 16th September 2023)
    # time_profile = [0, 1.87, 2.89, 3.8, 4.67, 5.52, 6.36, 7.21, 8.07, 8.93, 9.83]
    # speed_profile = [0, 5.35, 9.80, 10.99, 11.49, 11.76, 11.90, 11.76, 11.63, 11.63, 11.11]
    # init_distance = 13

    pr, vr, ar = create_runner_profile(speed_profile, time_profile, params.dt, t_end)
    xr = np.vstack((pr, vr))

    # Initialize vector for kart state and input
    uk = np.zeros((ni, num_steps-1))
    xk = np.zeros((ns, num_steps))
    ak = np.zeros((ni, num_steps-1))
    xk[0,0] = params.offset_ref + init_distance

    # Create reference for kart given the runner profile
    xk_ref = create_kart_reference(xr, params.dt, t_end)

    # Error variable [xk-xr]
    x_err = np.zeros((ns, num_steps))

    mismatch = []
    times = []

    ###### Initialize Controller ######
    LQR = LQRController(parameters=params)
    LQR.solve_LQR(Q=params.Qcatch, R=params.Rcatch)

    # Compute optimal trajectory for reference profile
    if Calculate_opt == True:
        x_opt, u_opt = LQR.solve_optcon_problem(x_init=xk[:,0], kart_ref=xk_ref, dt=params.dt, t_end=t_end)
    else:
        u_opt = np.zeros(num_steps-1)

    # Collect data for plots
    u_min = np.zeros(num_steps-1)
    u_max = np.ones(num_steps-1)
    uk_unconstrained = np.zeros(num_steps-1)

    speed_matched_before = False

    ## Control loop
    for t in range(0, num_steps-1):

        x_err[0,t] = xk[0,t] - xr[0,t] - params.offset_ref
        x_err[1,t] = xk[1,t] - xr[1,t]

        start_time = ttime.time()
        uk_unconstrained[t], uk[:,t], speed_matched = LQR.track_LQR(Calculate_opt, u_opt=u_opt[t], error=x_err[:,t], car_speed=xk[1,t], input_limits=[u_min[t], u_max[t]])
        end_time = ttime.time()

        exec_time = end_time - start_time
        times.append(exec_time*1000)
        print("Iter ", t, "time {:.12f}".format(exec_time*1000), "ms")

        # Understand when catch-up maneuver is finished for plots
        if speed_matched == True and speed_matched_before == False:
            time = t*params.dt

        speed_matched_before = speed_matched

        ## Forward simulate on linear dynamics (no model mismatch)
        lin = kart_linear_dyn(xk[:,t], uk[:,t])

        ## Forward simulate on non linear dynamics (real plant)
        # xk[:,t+1] = kart_non_linear_dyn(xk[:,t], uk[:,t])
        xk[:,t+1] = kart_non_linear_dyn(xk[:,t], uk[:,t])
        model_mismatch = xk[:,t+1] - lin
        # print(model_mismatch)
        mismatch.append(model_mismatch)

    avg_model_mismatch = np.mean(mismatch)
    print(avg_model_mismatch)

    print("Average execution time controller", np.mean(times))

    ### Save trajectories for comparison
    x_err[0,-1] = xk[0,-1] - xr[0,-1] - params.offset_ref
    x_err[1,-1] = xk[1,-1] - xr[1,-1]
    Path = 'LQR/'
    np.save(Path+'Error_traj_LQR.npy', np.vstack([x_err[0]+2.5, x_err[1]]).squeeze())
    np.save(Path+'Input_traj_LQR.npy', uk)

    time_hor = np.linspace(0, t_end, num=num_steps)
    time_hor_u = np.linspace(0, t_end-params.dt, num=num_steps-1)

    positive_mask_x = x_err[0] > 0
    negative_mask_x = x_err[0] < 0

    positive_mask_v = x_err[1] > 0
    negative_mask_v = x_err[1] < 0

    plt.figure("Relative position", figsize=(10,4))
    plt.plot(time_hor, x_err[0] + 2.5, 'k-', label='Relative distance', linewidth=2.0)
    plt.plot(time_hor, 2.5*np.ones(num_steps), label="Reference", linestyle='--', color='red', linewidth=1.5)
    plt.axvline(x=time, color='gray', linestyle='--', label='Catch accomplished')
    plt.fill_between(time_hor, 2.5, x_err[0] + 2.5, where=positive_mask_x, facecolor='green', alpha=0.2, label='Offset > 2.5m')
    plt.fill_between(time_hor, 2.5, x_err[0] + 2.5, where=negative_mask_x, facecolor='red', alpha=0.2, label='Offset < 2.5m')
    plt.xlabel(r'Time[$s$]')
    plt.ylabel(r'$\Delta p$ [$m$]')
    plt.ylim(0.5, init_distance+3)
    plt.legend()
    plt.grid(True)

    plt.figure("Relative velocity", figsize=(10,4))
    plt.plot(time_hor, x_err[1], 'k-', label='Relative velocity', linewidth=2.0)
    plt.plot(time_hor, np.zeros(num_steps), label="Reference", linestyle='--', color='red', linewidth=1.5)
    plt.axvline(x=time, color='gray', linestyle='--', label='Catch accomplished')
    plt.fill_between(time_hor, 0, x_err[1], where=positive_mask_v, facecolor='green', alpha=0.2, label='Kart faster')
    plt.fill_between(time_hor, 0, x_err[1], where=negative_mask_v, facecolor='red', alpha=0.2, label='Kart slower')
    plt.xlabel(r'Time[$s$]')
    plt.ylabel(r'$\Delta v$ [$\frac{m}{s}$]')
    plt.legend()
    plt.grid(True)

    # plt.figure("Absolute Position", figsize=(10,8))
    # plt.plot(time_hor, xr[0], label='Runner position', linestyle='-.', color='black', linewidth=0.5)
    # plt.plot(time_hor, xk_ref[0], label='Kart reference position')
    # plt.plot(time_hor, xk[0], label='Kart tracking position (closed loop)')
    # if Calculate_opt == True:
    #     plt.plot(time_hor, x_opt[0], label='Kart optimal position (open loop)')
    # plt.xlabel('Time[s]')
    # plt.ylabel('Position[m]')
    # plt.legend()
    # plt.grid(True)

    # plt.figure("Absolute Velocity", figsize=(10,8))
    # plt.plot(time_hor, xr[1], label='Runner velocity', linestyle='-.', color='black', linewidth=2.0)
    # plt.plot(time_hor, xk_ref[1], label='Kart reference velocity', linewidth=2.0)
    # plt.plot(time_hor, xk[1], label='Kart tracking velocity (closed loop)', linewidth=2.0)
    # # plt.plot(time_hor_u, kart_speed, label='Kart velocity from wheel encoders', linewidth=2.0)
    # plt.axvline(x=time, color='gray', linestyle='--', label='Catch accomplished')
    # if Calculate_opt == True:
    #     plt.plot(time_hor, x_opt[1], label='Kart optimal velocity (open loop)')
    # plt.xlabel('Time[s]')
    # plt.ylabel(r'Velocity [$\frac{m}{s}$]')
    # plt.legend(loc="lower right")
    # plt.grid(True)

    plt.figure("Input (Drive-train acceleration)", figsize=(10,4))
    plt.plot(time_hor_u, u_max, label='Input limit', linestyle='-.', color='red', linewidth=1.0)
    plt.plot(time_hor_u, u_min, linestyle='-.', color='red', linewidth=1.0)
    plt.axvline(x=time, color='gray', linestyle='--', label='Catch accomplished')
    plt.plot(time_hor_u, uk[0], label='Tracking input (clipped)', linewidth=2.0)
    # plt.plot(time_hor_u, uk_unconstrained, label='Tracking input (unclipped)', linewidth=1.0, linestyle='--', color='m')
    if Calculate_opt == True:
        plt.step(time_hor_u, u_opt, label='Optimal input')
    plt.xlabel(r'Time[$s$]')
    plt.ylabel(r'$u_k$')
    plt.legend(loc="center", bbox_to_anchor=(0.7, 0.7))
    plt.grid(True)

    plt.show()

if __name__ == "__main__":
    main()