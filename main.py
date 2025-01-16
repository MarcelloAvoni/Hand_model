import numpy as np
import matplotlib.pyplot as plt
from finger_class.finger_class_definition import Finger
from plot_functions.plot_results import plot_results
from plot_functions.plot_finger import make_animation



def run_simulation(finger, pulley_angles):
    num_simulations = len(pulley_angles)
    
    # Preallocate arrays for results
    joint_angles = np.zeros((num_simulations, finger.n_joints))
    tendon_tensions = np.zeros((num_simulations, finger.n_tendons))
    motor_torque = np.zeros(num_simulations)
    errors = np.zeros(num_simulations)

    for i in range(num_simulations):
        # Update the pulley angle
        finger.update_given_pulley_angle(pulley_angles[i])
        
        # Store the joint angles and tendon tensions
        for j in range(finger.n_joints):
            joint_angles[i, j] = finger.joints[j].theta
        
        for j in range(finger.n_tendons):
            tendon_tensions[i, j] = finger.tendons[j].tension
        
        # Store the motor torque and error
        motor_torque[i] = finger.motor_torque
        errors[i] = finger.error

    return joint_angles, tendon_tensions, motor_torque, errors


def main():
    # Create an instance of the Finger class with dummy data for a double joint finger
    name = "TestFinger"
    r_joints = [0.006, 0.005, 0.004]  # Three joints
    r_tip = 0.004
    L_phalanxes = [0.05, 0.03, 0.02]
    l_a = [0, 0, 0]
    l_b = [0, 0, 0]
    b_a_metacarpal = 1
    l_c = [0, 0, 0]
    l_d = [0, 0, 0]
    inf_stiff_tendons = [1, 1, 1, 1]
    k_tendons = [0, 0, 0, 0]
    l_springs = [0.001, 0.001]
    l_0_springs = [0, 0]
    k_springs = [2140, 2140]
    pulley_radius_functions = [lambda x: 0.01, lambda x: 0.012]
    tendon_joint_interface = [["e", "t", "e"], ["t", "e", "n"], ["f", "f", "n"], ["f", "f", "f"]]
    tendon_spring_interface = [[1, 0], [0, 1], [0, 0], [0, 0]]
    tendon_pulley_interface = [[0, 0], [0, 0], [1, 0], [0, 1]]

    finger = Finger(name, r_joints, r_tip, L_phalanxes, l_a, l_b, b_a_metacarpal, l_c, l_d, inf_stiff_tendons, k_tendons, l_springs, l_0_springs, k_springs, pulley_radius_functions, tendon_joint_interface, tendon_spring_interface, tendon_pulley_interface)

    # Simulation parameters
    num_simulations = 100
    pulley_angles = np.linspace(0, np.pi / 2, num_simulations)

    # Run the simulation
    joint_angles, tendon_tensions, motor_torque, errors = run_simulation(finger, pulley_angles)

    # Plot the results
    plot_results(pulley_angles, joint_angles, tendon_tensions, motor_torque, errors)

    # Plot the finger video
    make_animation(joint_angles, r_joints, r_tip, L_phalanxes)

if __name__ == "__main__":
    main()
