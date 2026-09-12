import numpy as np
from simulator import Simulator, centerline

sim = Simulator()

distance = 0.0  
track_distance = 0.0
time = 0.0
theta_error = 0

e_previous = 0

dt = 0.01

lapNum = 0

previous = 0

center_path = np.arange(1, 1051) / 2## little sections of car
c = np.array([centerline(i) for i in center_path])
distance_all = np.empty(c.shape[0])


def controller(x):

    global distance, track_distance, time, lapNum, theta_error, e_previous, previous

    xpos  = x[0]                   # current x position
    ypos  = x[1]                   # current y position
    phi   = np.mod(x[2], 2*np.pi)  # current heading (radians)
    v     = x[3]                   # current velocity
    theta = x[4]                   # current steering angle


    index = np.argmin((c[:, 0] - xpos)**2 + (c[:, 1] - ypos)**2)

    distance = center_path[index] % 105

    previous = index


    near = np.clip(0.3 * v, 3, 6)
    far = np.clip(0.5 * v + 6, 8, 15)

    near_angle = angle_offset(distance, near, xpos, ypos, phi)
    far_angle = angle_offset(distance, far, xpos, ypos, phi)

    if (np.abs(near_angle - far_angle) < 0.15):
        theta_error = far_angle
        p = 11
    else:
        theta_error = near_angle
        p = 10

    
    
    theta_error = theta_error - theta

    derivative = (theta_error - e_previous)/dt
    e_previous = theta_error
    derivative *= 0.75

    if v < 30: a = 4
    else: a = 0


    if np.abs(far_angle) > 0.2 and v > 20:
        a = -0.5


        
    theta_prime = np.clip(16 * theta_error + derivative, -1.0, 1.0)

    return np.array([a, theta_prime])





def angle_offset (distance, forward_distance, xpos, ypos, phi):

    next = centerline(distance + forward_distance)
    target_angle = np.arctan2(next[1]-ypos, next[0]-xpos)
    theta_error = target_angle - phi
    
    if theta_error > np.pi:
        theta_error -= 2*np.pi
    elif theta_error < -np.pi:
        theta_error += 2*np.pi
        
    theta_error = np.clip(theta_error, -0.7, 0.7)

    return theta_error



sim.set_controller(controller)
sim.run()
##sim.plot()
sim.animate()