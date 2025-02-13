# in this file we define the functions that compute the foot metrics

# import the necessary libraries
import numpy as np

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


def compute_hand_metric(x_1_phalanx_down, y_1_phalanx_down, x_2_phalanx_down, y_2_phalanx_down):
    # compute_hand_metrics calculates the average area of the polygon formed by the internal surface of the phalanges and the palm
    # INPUTS:
    # each input is a matrix where the rows indicate different phalanges and the columns indicate different pulley angles
    # x_1_phalanx_down: x coordinate of the first phalanx lower point
    # y_1_phalanx_down: y coordinate of the first phalanx lower point
    # x_2_phalanx_down: x coordinate of the second phalanx lower point
    # y_2_phalanx_down: y coordinate of the second phalanx lower point
    # OUTPUTS:
    # hand_metric: average area of the polygon formed by the internal surface of the phalanges and the palm

    #first we calculate the number of phalanges and angles
    n_phalanges = np.size(x_1_phalanx_down, 0)
    n_angles = np.size(x_1_phalanx_down, 1)

    # we compute the matrices thata wil be used for area calculation
    x = np.zeros((2*n_phalanges, n_angles))
    y = np.zeros((2*n_phalanges, n_angles))

    # we create the appropriate matrices for the area calculation
    for i in range(n_phalanges):
        x[2*i, :] = x_1_phalanx_down[i, :]
        x[2*i+1, :] = x_2_phalanx_down[i, :]
        y[2*i, :] = y_1_phalanx_down[i, :]
        y[2*i+1, :] = y_2_phalanx_down[i, :]

    # we compute the area of the polygon
    area = np.zeros(n_angles)
    for i in range(n_angles):
        area[i] = polygon_area(x[:, i], y[:, i])

    #we compute the average area
    hand_metric = np.mean(area)

    return hand_metric



