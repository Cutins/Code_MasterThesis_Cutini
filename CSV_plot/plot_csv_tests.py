import matplotlib.pyplot as plt
import csv
import numpy as np
import signal

font = {'family' : 'serif',
        'weight' : 'normal',
        'size' : '18'}
plt.rc('font', **font)

# Allow Crtl-C to work despite plotting
signal.signal(signal.SIGINT, signal.SIG_DFL)


Test = 'CSV_plot/Test6'  # Test1_LQR_csv, Test2_LQR_csv, Test3_LQR_csv

command_file = Test + '/motors_cmd.csv'

time_cmd = []
cmd_values = []
max_lim = []
min_lim =[]

count_cmd = 0

with open(command_file, newline='', encoding='utf-8') as file_csv:
    reader_csv = csv.reader(file_csv)

    header = next(reader_csv)

    for row in reader_csv:

        # Extract values from csv file
        cmd = float(row[4])

        # Create lists for plot
        time_cmd.append(count_cmd/20)
        cmd_values.append(cmd)
        max_lim.append(1)
        min_lim.append(0)

        count_cmd += 1

time_crtl = []
pos_errors = []
vel_errors = []

control_file = Test + '/control.csv'

count_crtl = 0

with open(control_file, newline='', encoding='utf-8') as file_csv:
    reader_csv = csv.reader(file_csv)

    header = next(reader_csv)

    for row in reader_csv:

        # Extract values from csv file
        pos_error = float(row[8])
        vel_error = float(row[12])

        # Create lists for plot
        time_crtl.append(count_crtl/100)

        pos_errors.append(pos_error + 2.5)
        vel_errors.append(vel_error)

        count_crtl += 1

estimator_runner = Test + '/Estimator_runner.csv'

time_est = []
rel_acc = []

count_est = 0

with open(estimator_runner, newline='', encoding='utf-8') as file_csv:
    reader_csv = csv.reader(file_csv)

    header = next(reader_csv)

    for row in reader_csv:

        # Extract values from csv file
        relative_acc = float(row[12])

        # Create lists for plot
        time_est.append(count_est/20)

        rel_acc.append(relative_acc)

        count_est += 1

###### Set time window we want to display from file
tmin = 288 #400
tmax = 300 #415

time_cmd_filt = [t for t in time_cmd if tmin <= t <= tmax]
cmd_values_filt = [cmd_values[time_cmd.index(t)] for t in time_cmd_filt]
max_lim_filt = [max_lim[time_cmd.index(t)] for t in time_cmd_filt]
min_lim_filt = [min_lim[time_cmd.index(t)] for t in time_cmd_filt]

plt.figure('Input command - drivetrain acceleration', figsize=(12, 5.5))
plt.plot(time_cmd_filt, cmd_values_filt, label='Command', linewidth=1.5)
plt.plot(time_cmd_filt, max_lim_filt, label='Max', linestyle='--', color='black', linewidth=1.0)
plt.plot(time_cmd_filt, min_lim_filt, label='Min', linestyle='--', color='black', linewidth=1.0)
plt.xlabel('Time[s]')
plt.ylabel(r'Drivetrain acceleration [$\frac{m}{s^2}$]')
plt.legend()
plt.grid(True)

time_crtl_filt = [t for t in time_crtl if tmin <= t <= tmax]
pos_errors_filt = [pos_errors[time_crtl.index(t)] for t in time_crtl_filt]
vel_errors_filt = [vel_errors[time_crtl.index(t)] for t in time_crtl_filt]

plt.figure('Relative position', figsize=(12, 5.5))
plt.plot(time_crtl_filt, 2.5*np.ones(len(time_crtl_filt)),  linestyle='--', color = 'black', label='Reference', linewidth=1.0)
plt.plot(time_crtl_filt, pos_errors_filt, label='Relative position', linewidth=1.5)
plt.fill_between(time_crtl_filt, 0, -0.5, color=(0.7, 0.7, 0.7), where=(np.array(pos_errors_filt) > -0.5), label='Go-Kart')
plt.fill_between(time_crtl_filt, 1.5, 0, color='orange', alpha=0.2, where=(np.array(pos_errors_filt) > -0.5), label='Unsafe Area')
plt.xlabel(r'Time[$s$]')
plt.ylabel(r'Position [$m$]')
plt.legend()
plt.grid(True)

plt.figure('Relative velocity', figsize=(12, 5.5))
plt.plot(time_crtl_filt, vel_errors_filt, label='Relative velocity', linewidth=1.5)
plt.plot(time_crtl_filt, np.zeros(len(time_crtl_filt)),  linestyle='--', color = 'black', label='Reference', linewidth=1.0)
plt.xlabel(r'Time[$s$]')
plt.ylabel(r'Velocity [$\frac{m}{s}$]')
plt.legend()
plt.grid(True)

time_est_filt = [t for t in time_est if tmin <= t <= tmax]
rel_acc_filt = [rel_acc[time_est.index(t)] for t in time_est_filt]

plt.figure('Relative acceleration', figsize=(12, 5.5))
plt.plot(time_est_filt, rel_acc_filt, label='Relative acceleration', linewidth=1.5)
plt.xlabel(r'Time[$s$]')
plt.ylabel(r'Acceleration [$\frac{m}{s^2}$]')
plt.legend()
plt.grid(True)


plt.show()

