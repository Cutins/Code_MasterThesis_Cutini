import numpy as np
import matplotlib.pyplot as plt
from tabulate import tabulate
from scipy.integrate import simps

font = {'family' : 'serif',
        'weight' : 'normal',
        'size' : '12'} # 16
plt.rc('font', **font)

Path_LQR = 'LQR/'
Path_MPC = 'MPC/'
Path_MPC_offset_free = 'MPC_offset_free/'

Error_traj_LQR = np.load(Path_LQR+'Error_traj_LQR.npy')
Error_traj_MPC = np.load(Path_MPC+'Error_traj_MPC.npy')
Error_traj_MPCOF = np.load(Path_MPC_offset_free+'Error_traj_MPCOF.npy')

Input_traj_LQR = np.load(Path_LQR+'Input_traj_LQR.npy')
Input_traj_MPC = np.load(Path_MPC+'Input_traj_MPC.npy')
Input_traj_MPCOF = np.load(Path_MPC_offset_free+'Input_traj_MPCOF.npy')

t_end = 15.0
dt = 0.05
num_steps = Error_traj_LQR.shape[1]

time_hor = np.linspace(0, t_end, num=num_steps)
time_hor_u = np.linspace(0, t_end-dt, num=num_steps-1)

u_max = np.ones(num_steps-1)
u_min = np.zeros(num_steps-1)

#### Find rising time = time needed to reach d_des
t_rise_LQR = 0
t_rise_MPC = 0
t_rise_MPCOF = 0

for t in range(0,int(t_end/dt)):
    if Error_traj_LQR[0,t] < 2.6 and t_rise_LQR==0:
       t_rise_LQR = t*dt
    if Error_traj_MPC[0,t] < 2.6 and t_rise_MPC==0:
       t_rise_MPC = t*dt
    if Error_traj_MPCOF[0,t] < 2.6 and t_rise_MPCOF==0:
       t_rise_MPCOF = t*dt
        

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

IAE_LQR_pos= simps(np.abs(Error_traj_LQR[0]-ref_pos), dx=dt)
IAE_MPC_pos= simps(np.abs(Error_traj_MPC[0]-ref_pos), dx=dt)
IAE_MPCOF_pos= simps(np.abs(Error_traj_MPCOF[0]-ref_pos), dx=dt)

IAE_LQR_vel= simps(np.abs(Error_traj_LQR[1]), dx=dt)
IAE_MPC_vel= simps(np.abs(Error_traj_MPC[1]), dx=dt)
IAE_MPCOF_vel= simps(np.abs(Error_traj_MPCOF[1]), dx=dt)

peak_LQR = np.min(Error_traj_LQR[0]-ref_pos)
peak_MPC = np.min(Error_traj_MPC[0]-ref_pos)
peak_MPCOF = np.min(Error_traj_MPCOF[0]-ref_pos)

ss_LQR = Error_traj_LQR[0,int(14/dt)]
ss_MPC = Error_traj_MPC[0,int(14/dt)]
ss_MPCOF = Error_traj_MPCOF[0,int(14/dt)]

d = [["LQR", Mean_err_LQR_pos, Mean_err_LQR_vel, Mean_err_LQR, Input_consump_LQR, t_rise_LQR], 
     ["MPC", Mean_err_MPC_pos, Mean_err_MPC_vel, Mean_err_MPC, Input_consump_MPC, t_rise_MPC], 
     ["MPC Offset Free", Mean_err_MPCOF_pos, Mean_err_MPCOF_vel, Mean_err_MPCOF, Input_consump_MPCOF, t_rise_MPCOF]]

print(tabulate(d, headers=["Name", "Avg position error", "Avg velocity error", "Avg error", "Input consumption", "Rise time"]))

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
print("Time needed to reach desired distance for LQR: ", t_rise_LQR)
print("Time needed to reach desired distance for MPC: ", t_rise_MPC)
print("Time needed to reach desired distance for MPCOF: ", t_rise_MPCOF)
print()
print("IAE position for LQR: ", IAE_LQR_pos)
print("IAE position for MPC: ", IAE_MPC_pos)
print("IAE position for MPCOF: ", IAE_MPCOF_pos)
print()
print("IAE velocity for LQR: ", IAE_LQR_vel)
print("IAE velocity for MPC: ", IAE_MPC_vel)
print("IAE velocity for MPCOF: ", IAE_MPCOF_vel)
print()
print("Peak value for LQR: ", peak_LQR + 2.5)
print("Peak value for MPC: ", peak_MPC + 2.5)
print("Peak value for MPCOF: ", peak_MPCOF + 2.5)
print()
print("Steady state LQR: ", (ss_LQR - 2.5))
print("Steady state MPC: ", (ss_MPC - 2.5))
print("Steady state MPCOF: ", (ss_MPCOF - 2.5))


plt.figure("Relative position", figsize=(10,4.5))
plt.plot(time_hor, 2.5 * np.ones(num_steps), label='Reference', linestyle='--', color='black', linewidth=1.0)
plt.plot(time_hor, Error_traj_LQR[0], 'b', label='LQR', linewidth=1.5)
plt.plot(time_hor, Error_traj_MPC[0], 'g', label='MPC', linewidth=1.5)
plt.plot(time_hor, Error_traj_MPCOF[0], 'r', label='MPC Offset Free', linewidth=1.5)
# plt.scatter(t_rise_LQR, 2.6, marker='o', color='b', label='Rise time LQR')
# plt.scatter(t_rise_MPC, 2.6, marker='o', color='g', label='Rise time MPC')
# plt.scatter(t_rise_MPCOF, 2.6, marker='o', color='r', label='Rise time MPC Offset Free')
plt.xlabel(r'Time[$s$]')
plt.ylabel(r'$\Delta p$ [$m$]')
plt.legend(loc="upper right")
plt.grid(True)

plt.figure("Relative velocity", figsize=(10,4.5))
plt.plot(time_hor, np.zeros(num_steps), label='Reference', linestyle='--', color='black', linewidth=1.0)
plt.plot(time_hor, Error_traj_LQR[1], 'b', label='LQR', linewidth=1.5)
plt.plot(time_hor, Error_traj_MPC[1], 'g', label='MPC', linewidth=1.5)
plt.plot(time_hor, Error_traj_MPCOF[1], 'r', label='MPC Offset Free', linewidth=1.5)
plt.xlabel(r'Time[$s$]')
plt.ylabel(r'$\Delta v$ [$\frac{m}{s}$]')
plt.legend()
plt.grid(True)

plt.figure("Input (Energy consuption)", figsize=(10,4.5))
plt.plot(time_hor_u, u_max, label='Input limits', linestyle='--', color='black', linewidth=1.0)
plt.plot(time_hor_u, u_min, linestyle='--', color='black', linewidth=1.5)
plt.plot(time_hor_u, Input_traj_LQR[0], 'b', label='LQR', linewidth=1.5)
plt.plot(time_hor_u, Input_traj_MPC[0], 'g', label='MPC', linewidth=1.5)
plt.plot(time_hor_u, Input_traj_MPCOF[0], 'r', label='MPC Offset Free', linewidth=1.5)
plt.xlabel(r'Time[$s$]')
plt.ylabel(r'$u_k$ [$\frac{m}{s^2}$]')
plt.legend(loc='upper right')
plt.grid(True)

plt.show()