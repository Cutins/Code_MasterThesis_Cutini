import numpy as np
import matplotlib.pyplot as plt
from tabulate import tabulate


Path_LQR = 'LQR/'
Path_MPC = 'MPC/'
Path_MPC_offset_free = 'MPC_offset_free/'

Error_traj_LQR = np.load(Path_LQR+'Error_traj_LQR.npy')
Error_traj_MPC = np.load(Path_MPC+'Error_traj_MPC.npy')
Error_traj_MPCOF = np.load(Path_MPC_offset_free+'Error_traj_MPCOF.npy')

Input_traj_LQR = np.load(Path_LQR+'Input_traj_LQR.npy')
Input_traj_MPC = np.load(Path_MPC+'Input_traj_MPC.npy')
Input_traj_MPCOF = np.load(Path_MPC_offset_free+'Input_traj_MPCOF.npy')

t_end = 18.0
dt = 0.05
num_steps = Error_traj_LQR.shape[1]

time_hor = np.linspace(0, t_end, num=num_steps)
time_hor_u = np.linspace(0, t_end-dt, num=num_steps-1)

u_max = np.ones(num_steps-1)
u_min = np.zeros(num_steps-1)


#### Evaluate errors
ref_pos = 2.5 * np.ones(num_steps)
ref_vel = np.zeros(num_steps)

Mean_err_LQR_pos = np.mean(np.abs(Error_traj_LQR[0]-ref_pos))
Mean_err_MPC_pos = np.mean(np.abs(Error_traj_MPC[0]-ref_pos))
Mean_err_MPCOF_pos = np.mean(np.abs(Error_traj_MPCOF[0]-ref_pos))

Mean_err_LQR_vel = np.mean(np.abs(Error_traj_LQR[1]))
Mean_err_MPC_vel = np.mean(np.abs(Error_traj_MPC[1]))
Mean_err_MPCOF_vel = np.mean(np.abs(Error_traj_MPCOF[1]))

Mean_err_LQR = np.mean(np.abs(np.vstack([Error_traj_LQR[0]-ref_pos, Error_traj_LQR[1]])))
Mean_err_MPC = np.mean(np.abs(np.vstack([Error_traj_MPC[0]-ref_pos, Error_traj_MPC[1]])))
Mean_err_MPCOF = np.mean(np.abs(np.vstack([Error_traj_MPCOF[0]-ref_pos, Error_traj_MPCOF[1]])))

Input_consump_LQR = np.mean(Input_traj_LQR)
Input_consump_MPC = np.mean(Input_traj_MPC)
Input_consump_MPCOF = np.mean(Input_traj_MPCOF)

d = [["LQR", Mean_err_LQR_pos, Mean_err_LQR_vel, Mean_err_LQR, Input_consump_LQR], 
     ["MPC", Mean_err_MPC_pos, Mean_err_MPC_vel, Mean_err_MPC, Input_consump_MPC], 
     ["MPC Offset Free", Mean_err_MPCOF_pos, Mean_err_MPCOF_vel, Mean_err_MPCOF, Input_consump_MPCOF]]

print(tabulate(d, headers=["Name", "Avg position error", "Avg velocity error", "Avg error", "Input consumption"]))

print("\n\n -----------------------------------")
print("Mean position error for LQR: ", Mean_err_LQR_pos)
print("Mean position error for MPC: ", Mean_err_MPC_pos)
print("Mean position error for MPCOF: ", Mean_err_MPCOF_pos)
print()
print("Mean velocity error for LQR: ", Mean_err_LQR_vel)
print("Mean velocity error for MPC: ", Mean_err_MPC_vel)
print("Mean velocity error for MPCOF: ", Mean_err_MPCOF_vel)
print()
print("Mean error for LQR: ", Mean_err_LQR)
print("Mean error for MPC: ", Mean_err_MPC)
print("Mean error for MPCOF: ", Mean_err_MPCOF)
print()
print("Mean energy consumption for LQR: ", Input_consump_LQR)
print("Mean energy consumption for MPC: ", Input_consump_MPC)
print("Mean energy consumption for MPCOF: ", Input_consump_MPCOF)
print()




plt.figure("Position and velocity errors", figsize=(10,5.5))
plt.subplot(2,1,1)
plt.plot(time_hor, Error_traj_LQR[0], 'b', label='LQR', linewidth=2.0)
plt.plot(time_hor, Error_traj_MPC[0], 'g', label='MPC', linewidth=2.0)
plt.plot(time_hor, Error_traj_MPCOF[0], 'r', label='MPC Offset Free', linewidth=2.0)
plt.plot(time_hor, 2.5 * np.ones(num_steps), label='Reference', linestyle='--', color='black', linewidth=1.5)
plt.xlabel('Time[s]')
plt.ylabel('Distance [m]')
plt.legend(loc="upper right")
plt.grid(True)

plt.subplot(2,1,2)
plt.plot(time_hor, Error_traj_LQR[1], 'b', label='LQR', linewidth=2.0)
plt.plot(time_hor, Error_traj_MPC[1], 'g', label='MPC', linewidth=2.0)
plt.plot(time_hor, Error_traj_MPCOF[1], 'r', label='MPC Offset Free', linewidth=2.0)
plt.plot(time_hor, np.zeros(num_steps), label='Reference', linestyle='--', color='black', linewidth=1.5)
plt.xlabel('Time[s]')
plt.ylabel(r'Velocity [$\frac{m}{s}$]')
plt.legend()
plt.grid(True)

plt.figure("Input (Energy consuption)", figsize=(10,4))
plt.plot(time_hor_u, Input_traj_LQR[0], 'b', label='LQR', linewidth=2.0)
plt.plot(time_hor_u, Input_traj_MPC[0], 'g', label='MPC', linewidth=2.0)
plt.plot(time_hor_u, Input_traj_MPCOF[0], 'r', label='MPC Offset Free', linewidth=2.0)
plt.plot(time_hor_u, u_max, label='Acceleration limits', linestyle='--', color='black', linewidth=1.5)
plt.plot(time_hor_u, u_min, linestyle='-.', color='black', linewidth=1.5)
plt.xlabel('Time[s]')
plt.ylabel(r'Acceleration [$\frac{m}{s^2}$]')
plt.legend(loc='upper right')
plt.grid(True)

plt.show()