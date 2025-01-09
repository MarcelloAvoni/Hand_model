import numpy as np
from scipy.optimize import least_squares
from finger_class.finger_class_definition import Finger

def test_finger_equations():
    # Create an instance of the Finger class with dummy data for a single joint
    name = "TestFinger"
    r_joints = [1]  # Single joint
    r_tip = 1
    L_phalanxes = [10]
    l_a = [1]
    l_b = [1]
    b_a = [1]
    b_b = [1]
    l_c = [1]
    l_d = [1]
    inf_stiff_tendons = [1,1]
    k_tendons = [0,0]
    l_springs = [1]
    l_0_springs = [1]
    k_springs = [1]
    pulley_radius_functions = [lambda x: 1]
    tendon_joint_interface = [["f"],["e"]]
    tendon_spring_interface = [[0],[1]]
    tendon_pulley_interface = [[1],[0]]

    finger = Finger(name, r_joints, r_tip, L_phalanxes, l_a, l_b, b_a, b_b, l_c, l_d, inf_stiff_tendons, k_tendons, l_springs, l_0_springs, k_springs, pulley_radius_functions, tendon_joint_interface, tendon_spring_interface, tendon_pulley_interface)

    # Initial guess for the variables (joint angle and flexor tendon tension)
    initial_guess = np.zeros(finger.n_joints + finger.n_pulleys)

    # Solve for equilibrium using least_squares
    result = least_squares(finger.finger_equations, initial_guess)

    # Print the results
    print("Optimized Variables:", result.x)
    print("Residuals:", result.fun)
    print("Success:", result.success)
    print("Message:", result.message)

# Run the test function
if __name__ == "__main__":
    test_finger_equations()