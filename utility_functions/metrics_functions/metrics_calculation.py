# in this file we define the functions that compute the foot metrics

# import the necessary libraries
import numpy as np
from utility_functions.kinematics_functions.kinematics_simulation import kinematics_simulation
from utility_functions.statics_functions.statics_simulation import statics_simulation
from utility_functions.kinematics_functions.kinematics_calculation import finger_kinematics


#function that computes the area of a polygon given its vertices
def polygon_area(x, y):
    #Compute the area of a polygon given its vertices using the Shoelace formula.

    #Parameters:
    #x (list or np.array): List or array of x coordinates of the polygon's vertices.
    #y (list or np.array): List or array of y coordinates of the polygon's vertices.

    #Returns:
    #float: Area of the polygon.

    n = len(x)  # Number of vertices
    area = 0.0

    for i in range(n):
        j = (i + 1) % n  # Wrap around to the first vertex
        area += x[i] * y[j] - y[i] * x[j]

    return abs(area) / 2.0


def compute_hand_metric(finger, pulley_angles):
    # compute_hand_metrics calculates the average area of the polygon formed by the internal surface of the phalanges and the palm
    # INPUTS:
    # finger: finger object
    # pulley_angles: array of pulley angles
    # OUTPUTS:
    # hand_metric: average area of the polygon

    #first we calculate the number of phalanges and angles
    n_phalanges = finger.n_joints
    n_angles = len(pulley_angles)

    joint_angles, _, _, _ = kinematics_simulation(finger, pulley_angles)

    # we preallocate the variables that will be used to store the coordinates of the main points
    x_1_phalanx_down = np.zeros((n_angles,n_phalanges))
    y_1_phalanx_down = np.zeros((n_angles,n_phalanges))
    x_2_phalanx_down = np.zeros((n_angles,n_phalanges))
    y_2_phalanx_down = np.zeros((n_angles,n_phalanges))

    for i_iter in range(n_angles):
        _, _, _, _, _, _, _, _, x_1_phalanx_down[i_iter,:], y_1_phalanx_down[i_iter,:], x_2_phalanx_down[i_iter,:], y_2_phalanx_down[i_iter,:] = finger_kinematics(finger, joint_angles[i_iter,:])


    # we compute the matrices thata wil be used for area calculation
    x = np.zeros((n_angles, 2*n_phalanges))
    y = np.zeros((n_angles, 2*n_phalanges))

    # we create the appropriate matrices for the area calculation
    for i in range(n_phalanges):
        x[:, 2*i] = x_1_phalanx_down[:, i]
        x[:, 2*i+1] = x_2_phalanx_down[:, i]
        y[:, 2*i] = y_1_phalanx_down[:, i]
        y[:, 2*i+1] = y_2_phalanx_down[:, i]

    # we compute the area of the polygon
    area = np.zeros(n_angles)
    for i in range(n_angles):
        area[i] = polygon_area(x[i,:].transpose(), y[i,:].transpose())

    #we compute the average area over the pulley angles    
    hand_metric = np.trapz(area, pulley_angles) / (pulley_angles[-1] - pulley_angles[0])

    return hand_metric

def compute_foot_metric(finger,force):

    # calculates the average support segment of the foot given the force applied to the finger
    # INPUTS:
    # finger: finger object
    # force: force applied to the finger
    # OUTPUTS:
    # foot_metric: average support segment of the foot

    # we first calculate the number of phalanges and simulations
    n_phalanges = finger.n_joints
    n_simulations = len(force)

    #we extract useful variables
    r_1 = finger.r_joints[0]
    r_2 = finger.r_tip

    #we run the simulatioin
    joint_angles, _, _, _ = statics_simulation(finger, force)

    # we preallocate the variables that will be used to store the coordinates of the main points
    x_tip = np.zeros(n_simulations)
    y_tip = np.zeros(n_simulations)

    #we extract the finger positions from the kinematics
    for i_iter in range(n_simulations):
        (_, _, _, _, _, _, x_2, y_2, _, _, _, _) = finger_kinematics(finger, joint_angles[i_iter,:])
        x_tip[i_iter] = x_2[-1]
        y_tip[i_iter] = y_2[-1]

    #we compute the support segment for each iteration
    segment_length = np.zeros(n_simulations)
    for i_iter in range(n_simulations):
        segment_length[i_iter] = np.sqrt(x_tip[i_iter]**2 + y_tip[i_iter]**2 - (r_1 - r_2)**2)


    #we compute the average support segment over the simulations
    foot_metric = np.trapz(segment_length, np.abs(force)) / np.abs(force[-1] - force[0])

    return foot_metric
