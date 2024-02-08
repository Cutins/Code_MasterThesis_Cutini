import numpy as np

class Parameters():
    # Reference distance between runner and Airshield
    offset_ref = 2.5 

    # Simulation - make sure dt (1/fs) is multiple of 0.01
    fs_simulation = 20.0
    dt = 1/fs_simulation
    
    # Vehicle Parameters (identified with Trailer)
    m = 300   # including trailer
    lr = 0.8
    lf = 0.8
    Cd = 1.5                # aerodynamic drag coefficient
    Croll = 73 #80           # rolling resistance coefficient
    Cm1 = 930               # Drivetrain modeling coefficient
    Cm2 = 0                 # Drivetrain modeling coefficient

    Cf = 10               

    # Linear affine model approximation - SS Matrices
    # xl = [pk, vk]
    # u = ua 
    A = np.array([[1, dt],
                  [0, 1-(Cf/m)*dt]])

    B = np.array([[0],
                  [(dt/m)*Cm1]])


    # Weight matrices for optimal feed-forward term
    Qo = np.diag([4e3, 4e3])
    Ro = np.array([[2e-3]])

    # LQR Tracking Weight matrices - default
    Q = np.diag([1e3, 4e3])
    R = np.array([[1e-3]])

    # LQR Tracking Weight matrices - catch-up
    Qcatch = np.diag([4e1, 1e3])
    Rcatch = np.array([[2e0]])
    