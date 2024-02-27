import matplotlib.pyplot as plt
import csv
import signal
import numpy as np
import sys, os

font = {'family' : 'serif',
        'weight' : 'normal',
        'size' : '16'}
plt.rc('font', **font)

# Allow Crtl-C to work despite plotting
signal.signal(signal.SIGINT, signal.SIG_DFL)

cwd = os.getcwd()
sys.path.append(os.getcwd())

from MPC.Utils import *

#### Runner profile 
File = 'Bolt_WR.csv'

time_bolt = []
velocity_bolt = []

with open(File, newline='', encoding='utf-8') as file_csv:
    reader_csv = csv.reader(file_csv)

    header = next(reader_csv)

    for row in reader_csv:
        time = float(row[0])
        vel = float(row[1])

        # Lists for plots
        time_bolt.append(time)
        velocity_bolt.append(vel)

# No saturation profile - Profile 1
speed_profile1 = [0, 2, 4.5, 6, 7, 7, 7, 7] 
time_profile1 = [0, 1, 2, 4, 8, 10, 13, 16]

# Profile with saturation - Profile 2
speed_profile2 = [0, 2, 4.5, 9, 9, 9, 9, 9]  
time_profile2 = [0, 1, 2, 4, 8, 10, 13, 16]

# Mujinga times every 10m
time_mujinga = [0, 2.03, 3.17, 4.21, 5.21, 6.21, 7.21, 8.23, 9.25, 10.30, 11.41]
speed_mujinga = [0, 4.93, 8.77, 9.62, 10, 10, 10, 9.80, 9.80, 9.52, 9]

# Marie-Josèe Ta-Lou (Diamond League Lausanne 30th June 2023)
time_woman = [0, 1.97, 3.1, 4.11, 5.08, 6.03, 6.99, 7.94, 8.9, 9.88, 10.88]
speed_woman = [0, 5.07, 8.85, 9.9, 10.31, 10.53, 10.42, 10.53, 10.42, 10.2, 10]

# Christian Coleman (Diamond League Eugene 16th September 2023)
time_man = [0, 1.87, 2.89, 3.8, 4.67, 5.52, 6.36, 7.21, 8.07, 8.93, 9.83]
speed_man = [0, 5.35, 9.80, 10.99, 11.49, 11.76, 11.90, 11.76, 11.63, 11.63, 11.11]

t_end = 15
dt = 0.05
num_steps = int(t_end/dt)
pr, vr1, ar = create_runner_profile(speed_profile1, time_profile1, dt, t_end)
pr, vr2, ar = create_runner_profile(speed_profile2, time_profile2, dt, t_end)
pr, vr_mudj, ar = create_runner_profile(speed_mujinga, time_mujinga, dt, t_end)

pr, vr_man, ar = create_runner_profile(speed_man, time_man, dt, t_end)
pr, vr_wom, ar = create_runner_profile(speed_woman, time_woman, dt, t_end)

time_hor = np.linspace(0, t_end, num=num_steps)


# plt.figure('Test Velocities', figsize=(10, 5.5))
# # plt.plot(time_bolt, velocity_bolt, label='Bolt', linewidth=2.0)
# # plt.plot(time_hor, vr_mudj, label='Mujinga')
# plt.plot(time_hor, vr_man, label='Coleman')
# plt.plot(time_hor, vr_wom, label='Ta-Lou')
# plt.scatter(9.83, vr_man[int(9.83/dt)], marker='*', color='r', label='Finish line')
# plt.scatter(10.88, vr_wom[int(10.88/dt)], marker='*', color='r')

# # plt.plot(time_hor, vr1, label='Velocity Profile 1')
# # plt.plot(time_hor, vr2, label='Velocity Profile 2')
# plt.xlabel(r'Time[$s$]')
# plt.ylabel(r'Runner Velocity [$\frac{m}{s}$]')
# plt.legend(loc='lower right')
# plt.grid(True)

print(9.83/dt)
print(time_hor[-1])
print(vr_man[100])

print(num_steps)


plt.figure('Test Velocities 2', figsize=(10, 5.5))
time_before = [step * dt for step in range(num_steps) if step * dt <= 9.83]
vr_man_before = [vr_man[step] for step in range(num_steps) if step * dt <= 9.83]
vr_wom_before = [vr_wom[step] for step in range(num_steps) if step * dt <= 9.83]
plt.plot(time_before, vr_man_before, color='blue', label='Christian Coleman', linewidth=1.5)
plt.plot(time_before, vr_wom_before, color='orange', label='Marie-Josèe Ta Lou', linewidth=1.5)
time_between = [step * dt for step in range(num_steps) if 9.83 < step * dt <= 10.88]
vr_man_between = [vr_man[step] for step in range(num_steps) if 9.83 < step * dt <= 10.88]
vr_wom_between = [vr_wom[step] for step in range(num_steps) if 9.83 < step * dt <= 10.88]
plt.plot(time_between, vr_man_between, color='blue', alpha=0.4, linewidth=1.5)
plt.plot(time_between, vr_wom_between, color='orange', linewidth=1.5)
time_after = [step * dt for step in range(num_steps) if step * dt > 10.88]
vr_man_after = [vr_man[step] for step in range(num_steps) if step * dt > 10.88]
vr_wom_after = [vr_wom[step] for step in range(num_steps) if step * dt > 10.88]
plt.plot(time_after, vr_man_after, color='blue', alpha=0.4, linewidth=1.5)
plt.plot(time_after, vr_wom_after, color='orange', alpha=0.4, linewidth=1.5)
plt.scatter(9.83, vr_man[int(9.83/dt)], marker='o', color='r', label='Finish line')
plt.scatter(10.88, vr_wom[int(10.88/dt)], marker='o', color='r')
plt.xlabel(r'Time [$s$]')
plt.ylabel(r'Runner Velocity [$\frac{m}{s}$]')
plt.legend(loc='lower right')
plt.grid(True)
plt.show()






