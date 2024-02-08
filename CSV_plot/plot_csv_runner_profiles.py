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
File_Runner = 'CSV_plot/Profile_Easy/Profile_Easy_100Hz.csv'

time_list_true = []
position_runner = []
velocity_runner = []
acceleration_runner = []
count = 0

with open(File_Runner, newline='', encoding='utf-8') as file_csv:
    reader_csv = csv.reader(file_csv)

    # header = next(reader_csv)

    for row in reader_csv:
        if count%5 == 0:
        
            # Extract values from csv file
            time =  int((count))
            pos = float(row[2])
            vel = float(row[3])
            acc = float(row[4])

            # Lists for plot
            time_list_true.append((time/5)+120)
            position_runner.append(pos)
            velocity_runner.append(vel)
            acceleration_runner.append(acc)

        count += 1




#### Runner estimated values
File_Est_Runner = 'CSV_plot/Profile_Easy/Estimator_ekf_runner.csv'

time_list_est_runner = []
position_rel = []
velocity_rel = []
acceleration_rel = []

count = 0

with open(File_Est_Runner, newline='', encoding='utf-8') as file_csv:
    reader_csv = csv.reader(file_csv)

    header = next(reader_csv)

    for row in reader_csv:
        
        # Extract values from csv file
        time = int(count*2)
        pos_rel = float(row[9]) 
        vel_rel = float(row[11]) 
        acc_rel = float(row[12]) 

        # Lists for plot
        time_list_est_runner.append(time)
        position_rel.append(pos_rel)
        velocity_rel.append(vel_rel)
        acceleration_rel.append(acc_rel)
        
        count += 1

print("Runner EKF dimension", count)


#### Vehicle estimated values
File_Est_Kart = 'CSV_plot/Profile_Easy/Estimator_ekf_vehicle.csv'

time_list_est_kart = []
velocity_kart = []
acceleration_kart= []

count = 0

with open(File_Est_Kart, newline='', encoding='utf-8') as file_csv:
    reader_csv = csv.reader(file_csv)

    header = next(reader_csv)

    for row in reader_csv:
        if count%5 == 0:
        
            # Extract values from csv file
            time = int((count*2)/5) # Wheel encoder runs at 100Hz while LIDAR at 20Hz
            vel_kart = float(row[6]) 
            acc_kart = float(row[7]) 

            # Lists for plot
            time_list_est_kart.append(time)
            velocity_kart.append(vel_kart)
            acceleration_kart.append(acc_kart)
        
        count += 1

print("Vehicle EKF dimension", count)

# Set time window we want to display from file
tmin = 0
tmax = 600

time_filt_true = [t for t in time_list_true if tmin <= t <= tmax]
pos_filt_runner = [pos for t, pos in zip(time_list_true, position_runner) if tmin <= t <= tmax]
vel_filt_runner = [vel for t, vel in zip(time_list_true, velocity_runner) if tmin <= t <= tmax]
acc_filt_runner = [acc for t, acc in zip(time_list_true, acceleration_runner) if tmin <= t <= tmax]

print("La lunghezza di time_list_est_kart è:", len(time_list_est_kart))
print("La lunghezza di velocity_kart è:", len(velocity_kart))
print("La lunghezza di acceleration_kart è:", len(acceleration_kart))
print("Lunghezza di velocity_rel:", len(velocity_rel))
print("Lunghezza di acceleration_rel:", len(acceleration_rel))

time_new = np.arange(0,len(time_list_est_kart))
##### Build runner absolute variables
vel_est_runner = [(velocity_kart[t] + velocity_rel[t]) for t in time_new]
acc_est_runner = [(acceleration_kart[t] + acceleration_rel[t]) for t in time_new]

# # ##### Build runner absolute variables
# vel_est_runner = []
# for t in time_list_est_kart:
#     if t < len(velocity_kart) and t < len(velocity_rel):
#         vel_est_runner.append(velocity_kart[t] + velocity_rel[t])

# acc_est_runner = []
# for t in time_list_est_kart:
#     if t < len(acceleration_kart) and t < len(acceleration_rel):
#         acc_est_runner.append(acceleration_kart[t] + acceleration_rel[t])


plt.figure('[RUNNER PROFILE VS ESTIMATED]', figsize=(10, 8))
plt.subplot(3,1,1)
plt.plot(time_filt_true, pos_filt_runner, label='Runner true position', linewidth=2.0)
plt.xlabel('Time[s]')
plt.ylabel('Position [m]')
plt.legend()
plt.grid(True)

plt.subplot(3,1,2)
plt.plot(time_filt_true, vel_filt_runner, label='Runner true velocity', linewidth=2.0)
plt.plot(time_new, vel_est_runner, label='Runner estimated velocity', linewidth=2.0)
plt.xlabel('Time[s]')
plt.ylabel(r'Velocity [$\frac{m}{s}$]')
plt.legend()
plt.grid(True)

plt.subplot(3,1,3)
plt.plot(time_filt_true, acc_filt_runner, label='Runner true acceleration', linewidth=2.0)
plt.plot(time_new, acc_est_runner, label='Runner estimated acceleration', linewidth=2.0)
plt.xlabel('Time[s]')
plt.ylabel(r'Acceleration [$\frac{m}{s^2}$]')
plt.legend()
plt.grid(True)

plt.figure('[RUNNER EKF]', figsize=(10, 8))
plt.subplot(3,1,1)
plt.plot(time_list_est_runner, position_rel, label='Estimated relative position', linewidth=2.0)
plt.xlabel('Time[s]')
plt.ylabel('Position [m]')
plt.legend()
plt.grid(True)

plt.subplot(3,1,2)
plt.plot(time_list_est_runner, velocity_rel, label='Estimated relative velocity', linewidth=2.0)
plt.xlabel('Time[s]')
plt.ylabel(r'Velocity [$\frac{m}{s}$]')
plt.legend()
plt.grid(True)

plt.subplot(3,1,3)
plt.plot(time_list_est_runner, acceleration_rel, label='Estimated relative acceleration', linewidth=2.0)
plt.xlabel('Time[s]')
plt.ylabel(r'Acceleration [$\frac{m}{s^2}$]')
plt.legend()
plt.grid(True)

plt.figure('[VEHICLE EKF]', figsize=(10, 8))
plt.subplot(2,1,1)
plt.plot(time_list_est_kart, velocity_kart, label='Kart absolute velocity', linewidth=2.0)
plt.xlabel('Time[s]')
plt.ylabel(r'Velocity [$\frac{m}{s}$]')
plt.legend()
plt.grid(True)

plt.subplot(2,1,2)
plt.plot(time_list_est_kart, acceleration_kart, label='Kart absolute accleration', linewidth=2.0)
plt.xlabel('Time[s]')
plt.ylabel(r'Acceleration [$\frac{m}{s^2}$]')
plt.legend()
plt.grid(True)




plt.show()