import numpy as np
from scipy.optimize import least_squares
from finger_class.finger_class_definition import Finger



# Run the test function
if __name__ == "__main__":
    # Create an instance of the Finger class with dummy data for a single joint
    name = "TestFinger"
    r_joints = [1]  # Single joint
    r_tip = 1
    L_phalanxes = [10]
    l_a = [0]
    l_b = [0]
    b_a_metacarpal = 1
    l_c = [0]
    l_d = [0]
    inf_stiff_tendons = [1,1]
    k_tendons = [0,0]
    l_springs = [1]
    l_0_springs = [0]
    k_springs = [1]
    pulley_radius_functions = [lambda x: 1]
    tendon_joint_interface = [["f"],["e"]]
    tendon_spring_interface = [[0],[1]]
    tendon_pulley_interface = [[1],[0]]

    finger = Finger(name, r_joints, r_tip, L_phalanxes, l_a, l_b, b_a_metacarpal, l_c, l_d, inf_stiff_tendons, k_tendons, l_springs, l_0_springs, k_springs, pulley_radius_functions, tendon_joint_interface, tendon_spring_interface, tendon_pulley_interface)
    print(finger.tendons[0].tension)
    print(finger.joints[0].theta)
    finger.update_given_flexor_length([1])
    print(finger.tendons[0].tension)
    print(finger.joints[0].theta)