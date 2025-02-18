# in this file we define the functions that analyzes a design given the metrics
from finger_class.finger_class_definition import Finger
from scipy.stats import uniform
import numpy as np
import random


#we define the design space throgh a dictionary
r_min = 0.002
r_max = 0.02

L_min_phalanx = 0.02
L_min_palm = 0.08
L_tot = 0.18




#function that creates different geometries of fingers
def generate_designs(f_1,f_2,p_r,r_min,r_max,L_min_phalanx,L_min_palm,L_tot):

    #we generate the number of joints
    n_joints = random.choice([1,2,3])

    # we generate the phalanges. Their sum must be equal to L_tot and their length must be bigger than L_min
    # we generate joints position in the total length and we make sure that the phalanges are bigger than L_min
    L_metacarpal = random.uniform(L_min_palm,L_tot - n_joints*L_min_phalanx)
    if n_joints == 1:
        L_phalanxes = [L_tot - L_metacarpal]
    elif n_joints == 2:
        joint_position = random.uniform(L_metacarpal,L_tot)
        lengths = [joint_position - L_metacarpal,L_tot - joint_position]
        while any([l < L_min_phalanx for l in lengths]):
            joint_position = random.uniform(L_metacarpal,L_tot)
            lengths = [joint_position - L_metacarpal,L_tot - joint_position]
        L_phalanxes = lengths
    else:
        joints_position = sorted(np.random.uniform(L_metacarpal,L_tot,n_joints-1))
        lengths = [joints_position[0] - L_metacarpal] + [joints_position[i] - joints_position[i-1] for i in range(1,n_joints-1)] + [L_tot - joints_position[-1]]
        while any([l < L_min_phalanx for l in lengths]):
            joints_position = sorted(np.random.uniform(L_metacarpal,L_tot,n_joints-1))
            lengths = [joints_position[0] - L_metacarpal] + [joints_position[i] - joints_position[i-1] for i in range(1,n_joints-1)] + [L_tot - joints_position[-1]]
        L_phalanxes = lengths

    # we generate the radii of the joints
    r_joints = [0] * n_joints
    r_joints[0] = random.uniform(r_min,r_max)

    if n_joints > 1:
        for i in range(1, n_joints):
            r_joints[i] = random.uniform((r_min + r_joints[i-1])/2, r_joints[i-1])

    r_tip = random.uniform(r_min,r_joints[-1])

    b_a_metacarpal = r_joints[0]
    
