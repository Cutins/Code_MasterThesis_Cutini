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

t_end = 13
dt = 0.05
num_steps = int(t_end/dt)
pr, vr1, ar = create_runner_profile(speed_profile1, time_profile1, dt, t_end)
pr, vr2, ar = create_runner_profile(speed_profile2, time_profile2, dt, t_end)
pr, vr_m, ar = create_runner_profile(speed_mujinga, time_mujinga, dt, t_end)
time_hor = np.linspace(0, t_end, num=num_steps)


plt.figure('[BOLT]', figsize=(10, 8))
plt.plot(time_bolt, velocity_bolt, label='Bolt', linewidth=2.0)
plt.plot(time_hor, vr_m, label='Mujinga')
plt.plot(time_hor, vr1, label='Velocity Profile 1')
plt.plot(time_hor, vr2, label='Velocity Profile 2')
plt.xlabel('Time[s]')
plt.ylabel('Velocity [m/s]')
plt.legend()
plt.grid(True)

plt.show()