import matplotlib.pyplot as plt
import csv
import signal

font = {'family' : 'serif',
        'weight' : 'normal',
        'size' : '16'}
plt.rc('font', **font)

# Allow Crtl-C to work despite plotting
signal.signal(signal.SIGINT, signal.SIG_DFL)


Test = 'CSV_plot/Test1_LQR_csv' # Test1_LQR_csv, Test2_LQR_csv, Test3_LQR_csv

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

        pos_errors.append(-pos_error)
        vel_errors.append(vel_error)

        count_crtl += 1

###### Set time window we want to display from file
tmin = 0 #400
tmax = 1000 #415

time_cmd_filt = [t for t in time_cmd if tmin <= t <= tmax]
cmd_values_filt = [cmd_values[time_cmd.index(t)] for t in time_cmd_filt]
max_lim_filt = [max_lim[time_cmd.index(t)] for t in time_cmd_filt]
min_lim_filt = [min_lim[time_cmd.index(t)] for t in time_cmd_filt]

plt.figure('Input command - drivetrain acceleration', figsize=(10, 3))
plt.plot(time_cmd_filt, cmd_values_filt, label='Command', linewidth=2.0)
plt.plot(time_cmd_filt, max_lim_filt, label='Max', linestyle='-.', color='red', linewidth=1.5)
plt.plot(time_cmd_filt, min_lim_filt, label='Min', linestyle='-.', color='red', linewidth=1.5)
plt.xlabel('Time[s]')
plt.ylabel(r'Drivetrain acceleration [$\frac{m}{s^2}$]')
plt.legend()
plt.grid(True)

time_crtl_filt = [t for t in time_crtl if tmin <= t <= tmax]
pos_errors_filt = [pos_errors[time_crtl.index(t)] for t in time_crtl_filt]
vel_errors_filt = [vel_errors[time_crtl.index(t)] for t in time_crtl_filt]

plt.figure('Position error', figsize=(10, 3))
plt.plot(time_crtl_filt, pos_errors_filt, label='Position error', linewidth=2.0)
plt.xlabel('Time[s]')
plt.ylabel('Position error [m]')
plt.legend()
plt.grid(True)

plt.figure('Velocity error', figsize=(10, 3))
plt.plot(time_crtl_filt, vel_errors_filt, label='Velocity error', linewidth=2.0)
plt.xlabel('Time[s]')
plt.ylabel(r'Velocity error [$\frac{m}{s}$]')
plt.legend()
plt.grid(True)


plt.show()

