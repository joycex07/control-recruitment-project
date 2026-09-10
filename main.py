import numpy as np
from simulator import Simulator, centerline
import time

sim = Simulator()

distance = 0.0  
track_distance = 0.0
time = 0
theta_error = 0

e_previous = theta_error
e_current = 0


dt = 0.01

lapNum = 0


def controller(x):

    global distance, track_distance, time, lapNum, theta_error, e_current, e_previous

    xpos  = x[0]                   # current x position
    ypos  = x[1]                   # current y position
    phi   = np.mod(x[2], 2*np.pi)  # current heading (radians)
    v     = x[3]                   # current velocity
    theta = x[4]                   # current steering angle



    """
    optimizing:
    1. dynamic forward_distance (based on velocity and angle?)
    2. figure out pid control -- the derivative to add to the angle to stabilize it
    """

    distance += v * dt
    time += dt

    dist_to_origin = np.hypot(xpos, ypos)
    if distance > 30.0 and np.abs(xpos) <= 2.0 and np.abs(ypos) <= 2.0:
        track_distance = distance
        lapNum += 1
        print(f"Lap Distance: {track_distance}")
        print(f"Lap Number: {lapNum}")
        print(f"Lap Time: {time}" )
        print(" ")
        time = 0.0
        distance = 0


    if v > 15:
        forward_distance = 5
    else:
        forward_distance = 3


    next = centerline(distance + forward_distance)

    target_angle = np.arctan2(next[1]-ypos, next[0]-xpos)

    theta_error = target_angle - phi

    
    if theta_error > np.pi:
        theta_error -= 2*np.pi
    elif theta_error < -np.pi:
        theta_error += 2*np.pi

    theta_error = np.clip(theta_error, -0.7, 0.7)
    theta_error = theta_error - theta

    derivative = (e_current - e_previous)/dt
    e_previous = e_current
    e_current = theta_error
    derivative *= 4

    if v < 25: a = 4
    else: a = 0

    if theta_error > 0.15 and v > 15:
        a = -0.5
        p = 12
    else:
        p = 15
        
    theta_prime = np.clip(15 * theta_error + derivative, -1.0, 1.0)


    return np.array([a, theta_prime])





sim.set_controller(controller)
sim.run()
# sim.plot()
sim.animate()