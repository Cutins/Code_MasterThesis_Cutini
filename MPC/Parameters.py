import numpy as np



class Parameters():
    # Reference distance between runner and Airshield
    offset_ref = 2.5 
    
    # Simulation - make sure dt (1/fs) is multiple of 0.01
    fs_simulation = 20.0
    dt = 1/fs_simulation

    safe_distance = 1.2
        
    # Vehicle Parameters 
    m = 300  
    lr = 0.8
    lf = 0.8
    Cd = 2.15               # aerodynamic drag coefficient
    Croll = 0 #80           # rolling resistance coefficient
    Cm1 = 920               # Drivetrain modeling coefficient
    Cm2 = 0                 # Drivetrain modeling coefficient

    Cf = 20                  

    # Linear prediction model with runner model
    # xp = [(pk-pr), (vk-vr), vk]
    # up = ua
    Ap = np.array([[1, dt, 0], 
                  [0, 1, -(Cf/m)*dt],
                  [0, 0, 1-(Cf/m)*dt]])
    
    Bp = np.array([[0], 
                  [dt*(Cm1/m)], 
                  [dt*(Cm1/m)]])
    
    
    # Linear affine model approximation - SS Matrices
    # xl = [pk, vk]
    # u = ua 
    A = np.array([[1, dt],
                  [0, 1-(Cf/m)*dt]])
    
    B = np.array([[0], 
                  [dt*(Cm1/m)]])
    
    Qo = np.diag([5, 5, 0]) 
    Ro = np.array([[1e-2]])    


    