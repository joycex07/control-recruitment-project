import numpy as np
from simulator import Simulator, centerline

sim = Simulator()

distance = 0.0  
track_distance = 0.0
time = 0.0

dt = 0.01

lapNum = 0


def controller(x):

    global distance, track_distance, time, lapNum

    xpos  = x[0]                   # current x position
    ypos  = x[1]                   # current y position
    phi   = np.mod(x[2], 2*np.pi)  # current heading (radians)
    v     = x[3]                   # current velocity
    theta = x[4]                   # current steering angle



    #step 1: calculate distance based on the time frame dt to plug into the centerline
    #step 2: calculate the steering angle - theta_dot
    #(use inverse trig functions to calculate how much to steer)


    distance += v * dt
    time += dt

    dist_to_origin = np.hypot(xpos, ypos)
    if time != 0.0 and distance > 30.0 and dist_to_origin <= 1.5:
        track_distance = distance
        lapNum += 1
        print(f"Lap Distance: {track_distance}")
        print(f"Lap Number: {lapNum}")
        print(f"Lap Time: {time}" )
        print(" ")
        time = 0.0
        distance %= track_distance


    lookahead_distance = 5.5
    next = centerline(distance + lookahead_distance)

    target_angle = np.arctan2(next[1]-ypos, next[0]-xpos)

    error_angle = target_angle - phi
    error_angle = (error_angle + np.pi) % (2 * np.pi) - np.pi

    desired_theta = np.clip(error_angle, -0.7, 0.7)
    
    theta_error = desired_theta - theta
    theta_dot = np.clip(15 * theta_error, -1.0, 1.0)

    if v > 15:
        a = 0
    else:
        a = 5


    return np.array([a, theta_dot])





sim.set_controller(controller)
sim.run()
# sim.plot()
sim.animate()