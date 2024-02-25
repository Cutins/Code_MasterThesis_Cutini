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

from MPC.Parameters import *
from MPC.Controller import *
from MPC.Utils import *
from MPC.Vehicle import *

ns = 2
ni = 1

nsp = 3

def main():
    params = Parameters()
    t_end = 15.0
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

    u_max = np.ones(num_steps-1)
    u_min = np.zeros(num_steps-1)

    # Reference trajectory - Error variables # e = [(pk-pr)-2.5, vk-vr, 0]
    h_ref = np.zeros((nsp, num_steps))
    h_ref[0] = params.offset_ref * np.ones(num_steps)

    # Measured error variables from lidar + acceleration runner estimated
    h_meas = np.zeros(nsp)

    # Model Plant Mismatch 
    mismatch = np.zeros((ns, num_steps))

    # LQR profile
    # Saturation profile
    # speed_profile = [0, 7, 8, 9, 10, 10, 10]  
    # time_profile = [0, 2, 4, 8, 10, 13, 16]
    # init_distance = 5

    # No saturation profile
    speed_profile = [0, 2, 4.5, 6, 7, 7, 7, 7]  
    time_profile = [0, 1, 2, 4, 8, 10, 13, 16]
    init_distance = 0.5

    # Marie-Josèe Ta-Lou (Diamond League Lausanne 30th June 2023)
    time_profile = [0, 1.97, 3.1, 4.11, 5.08, 6.03, 6.99, 7.94, 8.9, 9.88, 10.88]
    speed_profile = [0, 5.07, 8.85, 9.9, 10.31, 10.53, 10.42, 10.53, 10.42, 10.2, 10]
    init_distance = 4

    # # Christian Coleman (Diamond League Eugene 16th September 2023)
    time_profile = [0, 1.87, 2.89, 3.8, 4.67, 5.52, 6.36, 7.21, 8.07, 8.93, 9.83]
    speed_profile = [0, 5.35, 9.80, 10.99, 11.49, 11.76, 11.90, 11.76, 11.63, 11.63, 11.11]
    init_distance = 13



    xk[0,0] = params.offset_ref + init_distance

    position, velocity, acceleration = create_runner_profile(speed_profile, time_profile, params.dt, t_end)

    for t in range(0, num_steps-1):

        h_meas[0] = xk[0,t] - position[t]
        h_meas[1] = xk[1,t] - velocity[t]
        h_meas[2] = xk[1,t]

        ar = acceleration[t]

        print("\nMeasured variables:\n"
              "Relative distance: {:.2f}\n"
              "Relative velocity: {:.2f}\n"
              "Absolute kart velocity: {:.2f}\n"
              "Actual runner acceleration: {:.2f}".format(h_meas[0], h_meas[1], h_meas[2], acceleration[t]))

        h_opt, uk_opt = MPC.solve_optcon_problem(x_meas=h_meas, ar = ar, x_tar=h_ref[:,t])

        uk[:,t] = uk_opt[:,0] # Drive-train acceleration
        print("Optimal input: ", uk[:,t], "\n\n")

        ## Forward simulate on linear dynamics (no model mismatch)
        xklin = kart_linear_dyn(xk[:,t], uk[:,t])

        ## Forward simulate on non linear dynamics (real plant)
        xk[:,t+1] = kart_non_linear_dyn(xk[:,t], uk[:,t])
        model_mismatch = xklin - xk[:,t+1]
        # print("At velocity", xk[1,t] , "model mismatch is: ", model_mismatch)
        mismatch[:,t] = model_mismatch

        time_hor_mpc = np.linspace(t, t+int(t_hor/params.dt), num=int(t_hor/params.dt))
        time_hor_mpc_u = np.linspace(t, t+int(t_hor/params.dt)-params.dt, num=int(t_hor/params.dt)-1)

        # if t%10 == 0 or t == 0:
            # print(t, 'of', num_steps)

            # plt.figure("Relative Distance ", figsize=(10,8))
            # plt.plot(time_hor_mpc, h_opt[0], label='Relative position predictions')
            # plt.scatter(time_hor_mpc[-1], 2.5, color='red', marker='*', label='Relative target position') 
            # plt.scatter(time_hor_mpc[0], h_meas[0], color='blue', marker='o', label='Initial position') 
            # # plt.plot(time_hor_mpc, params.safe_distance * np.ones(num_steps_mpc), label='Safe distance', linestyle='-.', color='green', linewidth=1.5)
            # plt.xlabel('Time[s]')
            # plt.ylabel('Relative distance[m]')
            # plt.grid(True)

            # plt.figure("Relative Velocity ", figsize=(10,8))
            # plt.plot(time_hor_mpc, h_opt[1], label='Relative velocity predictions')
            # plt.scatter(time_hor_mpc[-1], 0.0, color='red', marker='*', label='Relative target velocity') 
            # plt.scatter(time_hor_mpc[0], h_meas[1], color='blue', marker='o', label='Initial velocity')       
            # plt.xlabel('Time[s]')
            # plt.ylabel(r'Relative velocity [$\frac{m}{s}$]')
            # plt.grid(True)

            # plt.figure("Kart velocity ", figsize=(10,8))
            # plt.plot(time_hor_mpc, h_opt[2], label='Kart velocity predictions')
            # plt.scatter(time_hor_mpc[0], h_meas[2], color='blue', marker='o', label='Initial Kart velocity')       
            # plt.xlabel('Time[s]')
            # plt.ylabel(r'Kart acceleration [$\frac{m}{s^2}$]')
            # plt.grid(True)

            # plt.figure("Optimal input", figsize=(10,8))
            # plt.plot(time_hor_mpc_u, uk_opt[0,:], label='Kart acceleration predictions')
            # plt.xlabel('Time[s]')
            # plt.ylabel(r'Kart acceleration [$\frac{m}{s^2}$]')
            # plt.grid(True)

            # plt.figure("States predictions", figsize=(10,8))
            # plt.plot(h_opt[0], h_opt[1], label='States')
            # plt.scatter(h_ref[0,t], h_ref[1,t], color='red', marker = '*', label='Target')
            # plt.scatter(h_meas[0], h_meas[1], color='blue', marker = 'o', label='Initial prediction point')
            # plt.xlabel('Position[m]')
            # plt.ylabel(r'Velocity [$\frac{m}{s}$]')
            # plt.grid(True)

        # plt.figure("Slack variables", figsize=(10,8))
        # plt.plot(time_hor_mpc, eps_low, label='Lower slack variable')
        # plt.plot(time_hor_mpc, eps_high, label='Upper slack variable')
        # plt.xlabel('Time[s]')
        # plt.ylabel(r'Slack variable [N]')
        # plt.grid(True)
            
    avg_model_mismatch = np.mean(mismatch)
    print("Average model mismatch", avg_model_mismatch)

    ### Save trajectories for comparison
    Path = 'MPC/'
    np.save(Path+'Error_traj_MPC.npy', np.vstack([xk[0]-position, xk[1]-velocity]).squeeze())
    np.save(Path+'Input_traj_MPC.npy', uk)

    time_hor = np.linspace(0, t_end, num=num_steps)
    time_hor_u = np.linspace(0, t_end-params.dt, num=num_steps-1)

    positive_mask_x = xk[0] - position > 2.5
    negative_mask_x = xk[0] - position < 2.5

    positive_mask_v = xk[1] - velocity > 0
    negative_mask_v = xk[1] - velocity < 0

    # plt.figure("Absolute position", figsize=(10,8))
    # plt.plot(time_hor, position, label='Runner position reference')
    # plt.plot(time_hor, xk[0], label='Kart tracking position (closed loop)')
    # plt.scatter(time_hor[0], xk[0,0], color='black', marker='o', label='Initial position')
    # plt.xlabel('Time[s]')
    # plt.ylabel('Position[m]')
    # plt.legend()
    # plt.grid(True)

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
    plt.ylim(1.5, init_distance+3)
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

    # plt.figure("Model Plant Mismatch", figsize=(10,8))
    # plt.plot(time_hor, mismatch[0], label='MPM position')
    # plt.plot(time_hor, mismatch[1], label='MPM velocity')
    # plt.xlabel('Time[s]')
    # plt.ylabel('MPM')
    # plt.legend()
    # plt.grid(True)

    # plt.figure("Runner acceleration", figsize=(10,8))
    # plt.plot(time_hor, acceleration, label='ar')
    # plt.xlabel('Time[s]')
    # plt.ylabel(r'Acceleration [$\frac{m}{s^2}$]')
    # plt.legend()
    # plt.grid(True)


    # # Animation
    # fig,ax = plt.subplots(figsize=(12, 4))
    # plt.ylim(-0.02, 0.02)
    # plt.xlim(min(xk[0])-4, max(xk[0])+2)
    # plt.grid(True)
    # ax.set_yticklabels([])

    # linee, = ax.plot(xk[0], np.zeros_like(xk[0]), color='gray', linestyle='-')

    # kart = ax.scatter(xk[0,0], 0, c=xk[1,0], cmap='viridis', vmin=min(xk[1]), vmax=max(xk[1]), s=100, edgecolors='k', label='Kart')

    # run, = ax.plot(position[0], 0, 'ko', label='Runner')
    # ref = ax.axvline(x=(xk[0,0]), color='red', linestyle='--', label='Reference')


    # plt.colorbar(kart, label=r'Velocity [$\frac{m}{s}$]')

    # def update(frame):
    #     kart.set_offsets(np.array([[xk[0, frame], 0]]))
    #     kart.set_array(np.array([xk[1, frame]]))

    #     run.set_xdata(position[frame])

    #     linee.set_ydata(np.zeros_like(xk[0]))
    #     linee.set_xdata(xk[0])

    #     ref.set_xdata(xk[0,frame]-2.5)

    #     return kart, run, linee, ref
    
    # plt.legend()
    # ax.set_xlabel('Position [m]')

    # ani = FuncAnimation(fig, update, frames=num_steps, interval=params.dt, blit=True)
    # # ani.save('animationMPC.mp4', writer='ffmpeg', fps=60)



    plt.show()



if __name__ == "__main__":
    main()