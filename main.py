import numpy as np
from math import atan2, sin, cos, pi
from finger_class.finger_class_definition import Finger
from utility_functions.plot_functions.plot_results import plot_kinematics_results, plot_statics_results
from utility_functions.kinematics_functions.kinematics_simulation import kinematics_simulation
from utility_functions.statics_functions.statics_simulation import statics_simulation
from utility_functions.design_functions.design_analysis import NSGA2_analysis
import matplotlib.pyplot as plt


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

    l_spring = 0.001
    l_0_spring = 0
    k_spring = 110
    pulley_radius_function = lambda x: 0.0125
    pulley_rotation = 3 * pi / 4
    max_force = 0.1

    n_pop = 30
    n_gen = 200
    cx_pb = 1
    mu_pb = 1

    pop, logbook, pop_hist = NSGA2_analysis(f_1,f_2,p_r,r_min,r_max,L_min_phalanx,L_min_palm,L_tot,l_spring,l_0_spring,k_spring,pulley_radius_function,pulley_rotation,max_force,n_pop,n_gen,cx_pb,mu_pb,seed=None)
    
    #now we want to plot the hand and foot metrics for each design
    # "b" for 1 phalanx, "g" for 2 phalanxes, "r" for 3 phalanxes
    color_scheme = ["b", "g", "r"]

    #we plot the results of the pareto front
    fig, ax = plt.subplots()
    hand_metric = np.zeros(n_pop)
    foot_metric = np.zeros(n_pop)
    for i in range(n_pop):
        hand_metric[i] = pop[i].fitness.values[0]
        foot_metric[i] = pop[i].fitness.values[1]
        
    ax.scatter(hand_metric,foot_metric,color="r")

    ax.set_xlabel("Hand metric")
    ax.set_ylabel("Foot metric")
    plt.show()

    #we plot the history of the population values
    ind_average = np.zeros((n_gen,5))
    ind_std = np.zeros((n_gen,5))
    ind_values = np.zeros((n_pop,1))
    for i in range(n_gen):
        for j in range(5):
            for k in range(n_pop):
                if j == 4:
                    ind_values[k] = pop_hist[i][k][-1] + pop_hist[i][k][-2]
                else:
                    ind_values[k] = pop_hist[i][k][j]
            ind_average[i][j] = np.mean(ind_values)
            ind_std[i][j] = np.std(ind_values)


    # Plot each metric in a separate subplot
    fig, axes = plt.subplots(5,1, figsize=(12, 10))  # Create a 2x2 grid of subplots
    axes = axes.flatten()  # Flatten the 2D array of axes for easier indexing

    # Define the x-axis values (generations)
    generations = np.arange(n_gen)

    # Plot each metric in its own subplot
    for j in range(5):  # Assuming 4 metrics
        ax = axes[j]
        ax.errorbar(
            generations, 
            ind_average[:, j], 
            yerr=ind_std[:, j], 
            label=f'Metric {j+1}', 
            fmt='-o',  # Line with circular markers
            capsize=5  # Add caps to the error bars
        )
        ax.set_title(f"Metric {j+1}", fontsize=14)
        ax.set_xlabel("Generation", fontsize=12)
        ax.set_ylabel("Metric Value", fontsize=12)
        ax.legend()
        ax.grid(True)

    # Adjust layout to prevent overlap
    plt.tight_layout()

    # Show the plot
    plt.show()

if __name__ == "__main__":
    NSGA_simulation()