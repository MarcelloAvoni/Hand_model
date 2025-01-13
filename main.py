import numpy as np
from scipy.optimize import least_squares
from finger_class.finger_class_definition import Finger



# Run the test function
if __name__ == "__main__":
    # Create an instance of the Finger class with dummy data for a double joint finger
    name = "TestFinger"
    r_joints = [1,1]  # Single joint
    r_tip = 1
    L_phalanxes = [5,5]
    l_a = [0,0]
    l_b = [0,0]
    b_a_metacarpal = 1
    l_c = [0,0]
    l_d = [0,0]
    inf_stiff_tendons = [1,1,1,1]
    k_tendons = [0,0,0,0]
    l_springs = [1,1]
    l_0_springs = [0,0]
    k_springs = [1,1]
    pulley_radius_functions = [lambda x: 1,lambda x: 1]
    tendon_joint_interface = [["e","n"],["t","e"],["f","n"],["f","f"]]
    tendon_spring_interface = [[1,0],[0,1],[0,0],[0,0]]
    tendon_pulley_interface = [[0,0],[0,0],[1,0],[0,1]]

    finger = Finger(name, r_joints, r_tip, L_phalanxes, l_a, l_b, b_a_metacarpal, l_c, l_d, inf_stiff_tendons, k_tendons, l_springs, l_0_springs, k_springs, pulley_radius_functions, tendon_joint_interface, tendon_spring_interface, tendon_pulley_interface)
    print("tendon tensions")
    print(finger.tendons[0].tension)
    print(finger.tendons[1].tension)
    print(finger.tendons[2].tension)
    print(finger.tendons[3].tension)
    print("angles")
    print(finger.joints[0].theta)
    print(finger.joints[1].theta)


