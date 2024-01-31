import numpy as np

def create_runner_profile(speed_profile: list, time_profile: list, dt, t_end):
    """Create speed profile of person - assuming motion only in x-direction"""

    num_steps = int(t_end / dt)
    position = np.zeros(num_steps)
    velocity = np.zeros(num_steps)
    acceleration = np.zeros(num_steps)
    
    for i in range(0, num_steps):
        time = i * dt
        speed = np.interp(time, time_profile, speed_profile)
        acceleration[i] = (speed - velocity[i-1]) / dt
        velocity[i] = velocity[i-1] + acceleration[i] * dt
        position[i] = position[i-1] + velocity[i] * dt

    return position, velocity, acceleration

def create_kart_reference(runner_data, dt, t_end):
    """Creates a reference curve (non feasible) for kart
    kart_pos = runner_pos + offset_ref
    kart_vel = runner_vel"""

    num_steps = int(t_end/dt)
    kart_ref = np.zeros((3, num_steps))
    
    for t in range(0, num_steps):

        kart_ref[0,t] = runner_data[0,t] + 2.5
        kart_ref[1,t] = runner_data[1,t]

    return kart_ref