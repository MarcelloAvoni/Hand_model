import numpy as np
from math import atan2, sin, cos, pi
from finger_class.finger_class_definition import Finger
from utility_functions.plot_functions.plot_results import plot_results
from utility_functions.plot_functions.plot_finger import make_animation
from utility_functions.kinematics_functions.kinematics_simulation import kinematics_simulation
from utility_functions.metrics_functions.metrics_calculation import compute_hand_metric, compute_foot_metric
from utility_functions.design_functions.design_analysis import NSGA2_analysis
import matplotlib.pyplot as plt




def main_debug():

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
    type_phalanxes = [1,2,3]
    L_metacarpal = 0.01


    inf_stiff_tendons = [1, 1, 1, 1]
    k_tendons = [0, 0, 0, 0]
    l_springs = [1e-4, 1e-4]
    l_0_springs = [0, 0]
    k_springs = [320, 320]
    pulley_radius_functions = [lambda x: 0.01, lambda x: 0.0125]
    tendon_joint_interface = [["e", "t", "e"], ["t", "e", "n"], ["f", "f", "n"], ["f", "f", "f"]]
    tendon_spring_interface = [[1, 0], [0, 1], [0, 0], [0, 0]]
    tendon_pulley_interface = [[0, 0], [0, 0], [1, 0], [0, 1]]

    finger_3 = Finger(name, r_joints, r_tip, L_phalanxes, type_phalanxes, L_metacarpal, b_a_metacarpal, f_1, f_2, p_r, inf_stiff_tendons, k_tendons, l_springs, l_0_springs, k_springs, pulley_radius_functions, tendon_joint_interface, tendon_spring_interface, tendon_pulley_interface)

    # Simulation parameters
    num_simulations = 100
    pulley_angles = np.linspace(0.01*3 * np.pi / 4, 3 * np.pi / 4, num_simulations)

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
    type_phalanxes = [1,3]
    L_metacarpal = 0.01


    inf_stiff_tendons = [1, 1, 1]
    k_tendons = [0, 0, 0]
    l_springs = [1e-4, 1e-4]
    l_0_springs = [0, 0]
    k_springs = [320, 320]
    pulley_radius_functions = [lambda x: 0.01]
    tendon_joint_interface = [["e", "n"], ["t", "e"], ["f", "f"]]
    tendon_spring_interface = [[1, 0], [0, 1],[0, 0]]
    tendon_pulley_interface = [[0], [0], [1]]

    finger_2 = Finger(name, r_joints, r_tip, L_phalanxes, type_phalanxes, L_metacarpal, b_a_metacarpal, f_1, f_2, p_r, inf_stiff_tendons, k_tendons, l_springs, l_0_springs, k_springs, pulley_radius_functions, tendon_joint_interface, tendon_spring_interface, tendon_pulley_interface)

    # Simulation parameters
    num_simulations = 100
    pulley_angles = np.linspace(0.01*3 * np.pi / 4, 3 * np.pi / 4, num_simulations)

    # Run the simulation
    joint_angles, tendon_tensions, motor_torque, errors = kinematics_simulation(finger_2, pulley_angles)

    # Plot the results
    plot_results(finger_2, pulley_angles, joint_angles, tendon_tensions, motor_torque, errors,"saved_media")

    # Plot the finger video
    make_animation(finger_2,joint_angles)

def main():

    f_1 = 0.5
    f_2 = 0.5*0.5
    p_r = 0.0015*1.5

    r_min = 0.003
    r_max = 0.01

    L_min_phalanx = 0.02
    L_min_palm = 0.08
    L_tot = 0.18

    l_spring = 0.1
    l_0_spring = 0
    k_spring = 21
    pulley_radius_function = lambda x: 0.01
    pulley_rotation = 3 * pi / 4
    max_force = 0.1

    n_pop = 100
    n_gen = 50
    cx_pb = 1
    mu_pb = 1

    pop, logbook, pop_hist = NSGA2_analysis(f_1,f_2,p_r,r_min,r_max,L_min_phalanx,L_min_palm,L_tot,l_spring,l_0_spring,k_spring,pulley_radius_function,pulley_rotation,max_force,n_pop,n_gen,cx_pb,mu_pb,seed=None)
    
    #now we want to plot the hand and foot metrics for each design
    # "b" for 1 phalanx, "g" for 2 phalanxes, "r" for 3 phalanxes
    color_scheme = ["b", "g", "r"]

    #we plot the results of the pareto front
    fig, ax = plt.subplots()
    for i in range(n_pop):
        hand_metric = pop[i].fitness.values[0]
        foot_metric = pop[i].fitness.values[1]
        ax.scatter(hand_metric,foot_metric,color="r")

    ax.set_xlabel("Hand metric")
    ax.set_ylabel("Foot metric")
    plt.show()







if __name__ == "__main__":
    main()