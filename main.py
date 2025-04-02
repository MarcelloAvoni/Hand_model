import numpy as np
from math import atan2, sin, cos, pi
from finger_class.finger_class_definition import Finger
from utility_functions.plot_functions.plot_results import plot_kinematics_results, plot_statics_results
from utility_functions.kinematics_functions.kinematics_simulation import kinematics_simulation
from utility_functions.statics_functions.statics_simulation import statics_simulation
from utility_functions.design_functions.design_analysis import NSGA2_analysis
import matplotlib.pyplot as plt
from matplotlib.ticker import ScalarFormatter


def main():
    # FINGER WITH 2 PHALANGES TEST
    # we initialize the finger parameters
    name_kin = "Kinematics_Simulation"
    name_stat = "Statics_Simulation"
    f_1 = 0.5
    f_2 = 0.5*0.5
    p_r = 0.0015*1.5


    r_joints = [0.006, 0.005]  # Three joints
    r_tip = 0.004
    L_phalanxes = [0.04, 0.03]
    b_a_metacarpal = 0.006
    type_phalanxes = [1,3]
    L_metacarpal = 0.08


    inf_stiff_tendons = [1, 1, 1]
    k_tendons = [0, 0, 0]
    l_springs = [5e-4, 5e-4]
    l_0_springs = [0, 0]
    k_springs = [320, 320]
    pulley_radius_functions = [lambda x: 0.01]
    tendon_joint_interface = [["e", "n"], ["t", "e"], ["f", "f"]]
    tendon_spring_interface = [[1, 0], [0, 1],[0, 0]]
    tendon_pulley_interface = [[0], [0], [1]]

    finger_kinematics = Finger(name_kin, r_joints, r_tip, L_phalanxes, type_phalanxes, L_metacarpal, b_a_metacarpal, f_1, f_2, p_r, inf_stiff_tendons, k_tendons, l_springs, l_0_springs, k_springs, pulley_radius_functions, tendon_joint_interface, tendon_spring_interface, tendon_pulley_interface)

    # Parameters for the kinematics simulation
    num_simulations = 100
    final_angle = 5* np.pi / 8
    initial_angle = 1e-4 * final_angle
    pulley_angles = np.linspace(initial_angle, final_angle, num_simulations)

    # Run the simulation
    joint_angles, tendon_tensions, motor_torque, errors = kinematics_simulation(finger_kinematics, pulley_angles)

    # Plot the results
    plot_kinematics_results(finger_kinematics, pulley_angles, joint_angles, tendon_tensions, motor_torque, errors,"saved_media")

    # Here we perform statics simulation instead
    finger_statics = Finger(name_stat, r_joints, r_tip, L_phalanxes, type_phalanxes, L_metacarpal, b_a_metacarpal, f_1, f_2, p_r, inf_stiff_tendons, k_tendons, l_springs, l_0_springs, k_springs, pulley_radius_functions, tendon_joint_interface, tendon_spring_interface, tendon_pulley_interface)
    for i in range(num_simulations):
        finger_statics.update_given_pulley_angle(pulley_angles[i])

    Fx = np.zeros((num_simulations, 2)) 
    Fy = np.zeros((num_simulations, 2))
    M = np.zeros((num_simulations, 2))

    Fmax = 20 #maximum force in [N]
    Fx[:,0] = - np.linspace(0,Fmax,num_simulations)
    Fx[:,1] = - np.linspace(0,0.8*Fmax,num_simulations)
    force = np.linspace(0,Fmax,num_simulations)

    joint_angles, tendon_tensions, motor_torque, errors = statics_simulation(finger_statics,Fx,Fy,M)

    # plot the results
    plot_statics_results(finger_statics, force, joint_angles, tendon_tensions, motor_torque, errors, "saved_media")



def NSGA_simulation():

    f_1 = 0.5
    f_2 = 0.5*0.5
    p_r = 0.0015*1.5

    r_min = 0.003
    r_max = 0.01

    L_min_phalanx = 0.02
    L_min_palm = 0.08
    L_tot = 0.18

    l_spring = 0.01
    l_0_spring = 0
    k_spring = 110
    pulley_radius_function = lambda x: 0.0125
    pulley_rotation = 3 * pi / 4
    max_force = 0.1

    n_pop = 5
    n_gen = 50
    cx_pb = 1
    mu_pb = 1


    hand_metric = np.zeros((n_pop,3))
    foot_metric = np.zeros((n_pop,3))

    for i in range(3):
        n_joints = i + 1

        pop, logbook, pop_hist = NSGA2_analysis(n_joints, f_1, f_2, p_r, r_min, r_max, L_min_phalanx, L_min_palm, L_tot, l_spring, l_0_spring, k_spring, pulley_radius_function, pulley_rotation, max_force, n_pop, n_gen, cx_pb, mu_pb, seed=None)
        

        # we stoer the fitness values of the pareto front
 
        for j in range(n_pop):
            hand_metric[j][i] = pop[j].fitness.values[0]
            foot_metric[j][i] = pop[j].fitness.values[1]

    # Set global plot parameters
    plt.rcParams['font.family'] = 'Times New Roman'
    plt.rcParams['figure.autolayout'] = True  # Automatically adjust subplot parameters to give specified padding
    plt.rcParams['axes.grid'] = True  # Enable grid for all plots
    plt.rcParams['grid.alpha'] = 0.75  # Set grid transparency
    plt.rcParams['grid.linestyle'] = '-'  # Set grid line style

    font_size = 18  # Define the font size for labels

    # Create a ScalarFormatter for scientific notation
    formatter = ScalarFormatter(useMathText=True)
    formatter.set_scientific(True)
    formatter.set_powerlimits((-1, 1))
        
    fig, ax = plt.subplots()
    # now we want to plot the hand and foot metrics for each design
    # "b" for 1 phalanx, "g" for 2 phalanxes, "r" for 3 phalanxes
    color_scheme = ["b", "g", "r"]
    for i in range(3):
        if i == 0:
            plot_label = "1 phalanx"
        else:
            plot_label = f"{i+1} phalanges"

        ax.scatter(hand_metric[:,i], foot_metric[:,i], color=color_scheme[i],label=plot_label)
    #add a legend
    ax.set_xlabel("Hand metric [-]")
    ax.set_ylabel("Foot metric [-]")
    ax.legend()
    plt.show()



if __name__ == "__main__":
    NSGA_simulation()