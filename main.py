import numpy as np
from finger_class.finger_class_definition import Finger
from utility_functions.plot_functions.plot_results import plot_results
from utility_functions.plot_functions.plot_finger import make_animation
from utility_functions.kinematics_functions.kinematics_simulation import kinematics_simulation
from math import atan2, sin, cos, pi



def main():

    # FINGER WITH 3 PHALANGES TEST
    # we initialize the finger parameters
    name = "Finger_3_phalanges"
    f_1 = 0.5
    f_2 = 0.5*0.5
    p_r = 0.0015*1.5

    r_joints = [0.006, 0.005, 0.004]  # Three joints
    r_tip = 0.003
    L_phalanxes = [0.04, 0.03, 0.02]
    b_a_metacarpal = 0.006
    l_a = [1, 0, 0]
    l_b = [0, 0, 0]
    l_c = [0, 0, 0]
    l_d = [0, 0, 0]

    for i_iter in range(len(r_joints)):

        if (i_iter == len(r_joints) - 1):

            r_1 = r_joints[i_iter]
            r_2 = r_tip

        else:
            r_1 = r_joints[i_iter]
            r_2 = r_joints[i_iter + 1]

        beta = atan2(2*(r_1-r_2),(L_phalanxes[i_iter]-r_1-r_2))

        if (i_iter == 0):
            l_a[i_iter] = 2*f_1*r_1
        elif(i_iter==2):
            l_a[i_iter] = (0.004 + 2*f_2*r_1)*cos(beta)
        else:
            l_a[i_iter] = (2*f_2*r_1)*cos(beta)

        l_b[i_iter] = f_1*r_1*cos(beta)

        if (i_iter == 0):
            l_c[i_iter] = p_r
        else:
            l_c[i_iter] = -p_r

        l_d[i_iter] = -p_r




    inf_stiff_tendons = [1, 1, 1, 1]
    k_tendons = [0, 0, 0, 0]
    l_springs = [1e-4, 1e-4]
    l_0_springs = [0, 0]
    k_springs = [320, 320]
    pulley_radius_functions = [lambda x: 0.01, lambda x: 0.0125]
    tendon_joint_interface = [["e", "t", "e"], ["t", "e", "n"], ["f", "f", "n"], ["f", "f", "f"]]
    tendon_spring_interface = [[1, 0], [0, 1], [0, 0], [0, 0]]
    tendon_pulley_interface = [[0, 0], [0, 0], [1, 0], [0, 1]]

    finger_3 = Finger(name, r_joints, r_tip, L_phalanxes, l_a, l_b, b_a_metacarpal, l_c, l_d, inf_stiff_tendons, k_tendons, l_springs, l_0_springs, k_springs, pulley_radius_functions, tendon_joint_interface, tendon_spring_interface, tendon_pulley_interface)

    # Simulation parameters
    num_simulations = 100
    pulley_angles = np.linspace(0, 3 * np.pi / 4, num_simulations)

    # Run the simulation
    joint_angles, tendon_tensions, motor_torque, errors = kinematics_simulation(finger_3, pulley_angles)

    # Plot the results
    plot_results(finger_3, pulley_angles, joint_angles, tendon_tensions, motor_torque, errors,"saved_media")

    # Plot the finger video
    make_animation(finger_3,joint_angles)
    
    # FINGER WITH 2 PHALANGES TEST
    # we initialize the finger parameters
    name = "Finger_2_phalanxes"
    f_1 = 0.5
    f_2 = 0.5*0.5
    p_r = 0.0015*1.5


    r_joints = [0.006, 0.005]  # Three joints
    r_tip = 0.004
    L_phalanxes = [0.04, 0.03]
    b_a_metacarpal = 0.006
    l_a = [0, 0]
    l_b = [0, 0]
    l_c = [0, 0]
    l_d = [0, 0]

    for i_iter in range(len(r_joints)):

        if (i_iter == len(r_joints) - 1):

            r_1 = r_joints[i_iter]
            r_2 = r_tip

        else:
            r_1 = r_joints[i_iter]
            r_2 = r_joints[i_iter + 1]

        beta = atan2(2*(r_1-r_2),(L_phalanxes[i_iter]-r_1-r_2))

        if (i_iter == 0):
            l_a[i_iter] = 2*f_1*r_1
        else:
            l_a[i_iter] = (2*f_2*r_1)*cos(beta)

        l_b[i_iter] = 2*f_1*r_1*cos(beta)

        if (i_iter == 0):
            l_c[i_iter] = p_r
        else:
            l_c[i_iter] = -p_r

        l_d[i_iter] = -p_r


    inf_stiff_tendons = [1, 1, 1]
    k_tendons = [0, 0, 0]
    l_springs = [1e-4, 1e-4]
    l_0_springs = [0, 0]
    k_springs = [320, 320]
    pulley_radius_functions = [lambda x: 0.01]
    tendon_joint_interface = [["e", "n"], ["t", "e"], ["f", "f"]]
    tendon_spring_interface = [[1, 0], [0, 1],[0, 0]]
    tendon_pulley_interface = [[0], [0], [1]]

    finger_2 = Finger(name, r_joints, r_tip, L_phalanxes, l_a, l_b, b_a_metacarpal, l_c, l_d, inf_stiff_tendons, k_tendons, l_springs, l_0_springs, k_springs, pulley_radius_functions, tendon_joint_interface, tendon_spring_interface, tendon_pulley_interface)

    # Simulation parameters
    num_simulations = 100
    pulley_angles = np.linspace(0, 3 * np.pi / 4, num_simulations)

    # Run the simulation
    joint_angles, tendon_tensions, motor_torque, errors = kinematics_simulation(finger_2, pulley_angles)

    # Plot the results
    plot_results(finger_2, pulley_angles, joint_angles, tendon_tensions, motor_torque, errors,"saved_media")

    # Plot the finger video
    make_animation(finger_2,joint_angles)

def debug():

    #we define a simple finger to be used for debugging
    # FINGER WITH 2 PHALANGES TEST
    # we initialize the finger parameters
    name = "debugging_finger"

    r_joints = [1,1]
    r_tip = 1
    L_phalanxes = [5, 5]
    b_a_metacarpal = 1
    l_a = [0, 0]
    l_b = [0, 0]
    l_c = [0, 0]
    l_d = [0, 0]

    inf_stiff_tendons = [1, 1, 1, 1]
    k_tendons = [0, 0, 0, 0]
    l_springs = [1, 1]
    l_0_springs = [0, 0]
    k_springs = [1, 1]
    pulley_radius_functions = [lambda x: 1, lambda x: 1]
    tendon_joint_interface = [["e", "n"], ["t", "e"], ["f", "n"], ["f", "f"]]
    tendon_spring_interface = [[1, 0], [0, 1], [0, 0], [0, 0]]
    tendon_pulley_interface = [[0, 0], [0, 0], [1, 0], [0, 1]]

    finger = Finger(name, r_joints, r_tip, L_phalanxes, l_a, l_b, b_a_metacarpal, l_c, l_d, inf_stiff_tendons, k_tendons, l_springs, l_0_springs, k_springs, pulley_radius_functions, tendon_joint_interface, tendon_spring_interface, tendon_pulley_interface)


    # we try to debug the system
    print("INITIAL STATE")
    print()
    for i_iter in range(finger.n_tendons):
        print(f'Tendon {finger.tendons[i_iter].name} has {finger.tendons[i_iter].tension} N of Tension and is {finger.tendons[i_iter].length} m long')
    print()


    theta1 = pi/2
    theta2 = pi/2
    l1 = 2 * (1 - sin(theta1/2))
    l2 = l1 + 2 * (1 - sin(theta2/2))

    finger.update_given_flexor_length([l1,l2])


    print("INTERMEDIATE STATE")
    for i_iter in range(finger.n_tendons):
        print(f'Tendon {finger.tendons[i_iter].name} has {finger.tendons[i_iter].tension} N of Tension and is {finger.tendons[i_iter].length} m long')
    print()

    finger.update_given_phalanx_wrenches([-1,-2],[-2,-1],[0,1])

    print("FINAL STATE")
    for i_iter in range(finger.n_tendons):
        print(f'Tendon {finger.tendons[i_iter].name} has {finger.tendons[i_iter].tension} N of Tension and is {finger.tendons[i_iter].length} m long')
    print()





if __name__ == "__main__":
    main()