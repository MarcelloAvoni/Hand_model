# in this file we define the functions that analyzes a design given the metrics
from finger_class.finger_class_definition import Finger
from utility_functions.metrics_functions.metrics_calculation import compute_hand_metric, compute_foot_metric
import numpy as np
import random
from deap import base, creator, tools, algorithms



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

    #we return the parameters as a dictionary

    return r_joints, r_tip, L_phalanxes, L_metacarpal, b_a_metacarpal, f_1, f_2, p_r


# function that computes the metrics for a given design
def evaluate_design(r_joints, r_tip, L_phalanxes, L_metacarpal, b_a_metacarpal, f_1, f_2, p_r, l_spring, l_0_spring, k_spring, pulley_radius_function, pulley_rotation, max_force):
  

    # we create the finger objects
    n_joints = len(r_joints)

    if n_joints == 1:
        
        type_phalanxes = [3]
        
        inf_stiff_tendons = [1, 1]
        k_tendons = [0, 0]

        l_springs_hand = [l_spring] * 1
        l_springs_foot = [l_spring] * 2


        l_0_springs_hand = [l_0_spring] * 1
        l_0_springs_foot = [l_0_spring] * 2

        k_spring_hand = [k_spring] * 1
        k_spring_foot = [k_spring] * 2

        pulley_radius_function_hand = [pulley_radius_function] * 1
        pulley_radius_function_foot = []

        tendon_joint_interface = [["e"], ["f"]]

        tendon_spring_interface_hand = [[1], [0]]

        tendon_pulley_interface_hand = [[0], [1]]

        tendon_spring_interface_foot = [[1, 0], [0, 1]]

        tendon_pulley_interface_foot = [[], []]





    elif n_joints == 2:

        type_phalanxes = [1, 3]

        inf_stiff_tendons = [1, 1, 1]
        k_tendons = [0, 0, 0]

        l_springs_hand = [l_spring] * 2
        l_springs_foot = [l_spring] * 3


        l_0_springs_hand = [l_0_spring] * 2
        l_0_springs_foot = [l_0_spring] * 3

        k_spring_hand = [k_spring] * 2
        k_spring_foot = [k_spring] * 3

        pulley_radius_function_hand = [pulley_radius_function] * 1
        pulley_radius_function_foot = []

        tendon_joint_interface = [["e", "n"], ["t", "e"], ["f", "f"]]

        tendon_spring_interface_hand = [[1, 0], [0, 1], [0, 0]]

        tendon_pulley_interface_hand = [[0], [0], [1]]

        tendon_spring_interface_foot = [[1, 0, 0], [0, 1, 0], [0, 0, 1]]

        tendon_pulley_interface_foot = [[], [], []]

    elif n_joints == 3:

        type_phalanxes = [1, 2, 3]

        inf_stiff_tendons = [1, 1, 1]
        k_tendons = [0, 0, 0]

        l_springs_hand = [l_spring] * 2
        l_springs_foot = [l_spring] *3


        l_0_springs_hand = [l_0_spring] * 2
        l_0_springs_foot = [l_0_spring] * 3

        k_spring_hand = [k_spring] * 2
        k_spring_foot = [k_spring] * 3

        pulley_radius_function_hand = [pulley_radius_function] * 1
        pulley_radius_function_foot = []

        tendon_joint_interface = [["e", "t", "e"], ["t", "e", "n"], ["f", "f", "f"]]

        tendon_spring_interface_hand = [[1, 0], [0, 1], [0, 0]]

        tendon_pulley_interface_hand = [[0], [0], [1]]

        tendon_spring_interface_foot = [[1, 0, 0], [0, 1, 0], [0, 0, 1]]

        tendon_pulley_interface_foot = [[], [], []]


    hand = Finger("hand", r_joints, r_tip, L_phalanxes, type_phalanxes, L_metacarpal, b_a_metacarpal, f_1, f_2, p_r, inf_stiff_tendons, k_tendons, l_springs_hand, l_0_springs_hand, k_spring_hand, pulley_radius_function_hand, tendon_joint_interface, tendon_spring_interface_hand, tendon_pulley_interface_hand)

    foot = Finger("foot", r_joints, r_tip, L_phalanxes, type_phalanxes, L_metacarpal, b_a_metacarpal, f_1, f_2, p_r, inf_stiff_tendons, k_tendons, l_springs_foot, l_0_springs_foot, k_spring_foot, pulley_radius_function_foot, tendon_joint_interface, tendon_spring_interface_foot, tendon_pulley_interface_foot)


    # we compute the metrics
    pulley_angles = np.linspace(0.01*pulley_rotation, pulley_rotation, 100)
    hand_metric = compute_hand_metric(hand, pulley_angles)

    force = np.linspace(-0.01*max_force, -max_force, 100)
    foot_metric = compute_foot_metric(foot, force)


    return hand_metric, foot_metric


#we define a function that creates a database of designs. Each design is stored as a dictionary
def create_design_database(n_design,f_1,f_2,p_r,r_min,r_max,L_min_phalanx,L_min_palm,L_tot, l_spring, l_0_spring, k_spring, pulley_radius_function, pulley_rotation, max_force):

    #we create the database
    database = []

    for i in range(n_design):
        r_joints, r_tip, L_phalanxes, L_metacarpal, b_a_metacarpal, f_1, f_2, p_r = generate_designs(f_1,f_2,p_r,r_min,r_max,L_min_phalanx,L_min_palm,L_tot)
        hand_metric, foot_metric = evaluate_design(r_joints, r_tip, L_phalanxes, L_metacarpal, b_a_metacarpal, f_1, f_2, p_r, l_spring, l_0_spring, k_spring, pulley_radius_function, pulley_rotation, max_force)
        database.append({
            "n_joints":len(r_joints),
            "r_joints":r_joints,
            "r_tip":r_tip,
            "L_phalanxes":L_phalanxes,
            "L_metacarpal":L_metacarpal,
            "b_a_metacarpal":b_a_metacarpal,
            "f_1":f_1,
            "f_2":f_2,
            "p_r":p_r,
            "hand_metric":hand_metric,
            "foot_metric":foot_metric})

    return database


#we define a NSGA-II algorithm for the evaluation of the designs, we assume to have 2 phalanges
def NSGA_evaluation():

    def evaluate(individual):

        r_joints = [individual[0], individual[1]]
        r_tip = individual[2]
        L_phalanxes = [individual[3], individual[4]]
        L_metacarpal = individual[5]

        b_a_metacarpal = r_joints[0]

        f_1 = 0.5
        f_2 = 0.5*0.5
        p_r = 0.0015*1.5

        l_spring = 0.1
        l_0_spring = 0
        k_spring = 21
        pulley_radius_function = lambda x: 0.01
        pulley_rotation = 3 * np.pi / 4
        max_force = 0.1

        hand_metric, foot_metric = evaluate_design(r_joints, r_tip, L_phalanxes, L_metacarpal, b_a_metacarpal, f_1, f_2, p_r, l_spring, l_0_spring, k_spring, pulley_radius_function, pulley_rotation, max_force)

        return (hand_metric, foot_metric)
    
    # we define the optimization problem as a maximization problem
    creator.create("FitnessMulti", base.Fitness, weights=(1.0, 1.0))
    creator.create("Individual", list, fitness=creator.FitnessMax)

    # we define the toolbox
    toolbox = base.Toolbox()
    toolbox.register("attr_float", random.uniform, 0.003, 0.01)
    


    


    
