import matplotlib.pyplot as plt
import csv
import signal
import numpy as np

font = {'family' : 'serif',
        'weight' : 'normal',
        'size' : '16'}
plt.rc('font', **font)

# Allow Crtl-C to work despite plotting
signal.signal(signal.SIGINT, signal.SIG_DFL)

#### Runner profile 
File_Runner = 'running_profiles/Profile_Andreas_100Hz.csv' #Profile_Andreas_100Hz.csv, usain_bolt_profile_python.csv

time_list_true = []
position_runner = []
velocity_runner = []
acceleration_runner = []
count = 0

with open(File_Runner, newline='', encoding='utf-8') as file_csv:
    reader_csv = csv.reader(file_csv)

    header = next(reader_csv)

    for row in reader_csv:
        if count%5 == 0:
        
            # Extract values from csv file
            time =  float(row[1])
            pos = float(row[2])
            vel = float(row[3])
            # acc = float(row[4])

            # Lists for plot
            time_list_true.append(time)
            position_runner.append(pos)
            velocity_runner.append(vel)
            # acceleration_runner.append(acc)

        count += 1




# Set time window we want to display from file
tmin = 0
tmax = 15

time_filt_true = [t for t in time_list_true if tmin <= t <= tmax]
pos_filt_runner = [pos for t, pos in zip(time_list_true, position_runner) if tmin <= t <= tmax]
vel_filt_runner = [vel for t, vel in zip(time_list_true, velocity_runner) if tmin <= t <= tmax]
# acc_filt_runner = [acc for t, acc in zip(time_list_true, acceleration_runner) if tmin <= t <= tmax]



plt.figure('[RUNNER PROFILE VS ESTIMATED]', figsize=(10, 8))
plt.subplot(3,1,1)
plt.plot(time_filt_true, pos_filt_runner, label='Runner true position', linewidth=2.0)
plt.xlabel('Time[s]')
plt.ylabel('Position [m]')
plt.legend()
plt.grid(True)

plt.subplot(3,1,2)
plt.plot(time_filt_true, vel_filt_runner, label='Runner true velocity', linewidth=2.0)
plt.xlabel('Time[s]')
plt.ylabel(r'Velocity [$\frac{m}{s}$]')
plt.legend()
plt.grid(True)

plt.subplot(3,1,3)
# plt.plot(time_filt_true, acc_filt_runner, label='Runner true acceleration', linewidth=2.0)
plt.xlabel('Time[s]')
plt.ylabel(r'Acceleration [$\frac{m}{s^2}$]')
plt.legend()
plt.grid(True)


plt.show()