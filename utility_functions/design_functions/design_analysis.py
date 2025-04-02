# in this file we define the functions that analyzes a design given the metrics
from finger_class.finger_class_definition import Finger
from utility_functions.metrics_functions.metrics_calculation import compute_hand_metric, compute_foot_metric
import numpy as np
import random
from deap import base, creator, tools
import matplotlib.pyplot as plt

def code_individual(r_joints,r_tip,L_phalanxes,L_metacarpal):

    individual = r_joints + [r_tip] + L_phalanxes + [L_metacarpal]

    return individual

def decode_individual(n_joints,individual):

    r_joints = individual[0:n_joints]
    r_tip = individual[n_joints]
    L_phalanxes = individual[n_joints+1:2*n_joints+1]
    L_metacarpal = individual[2*n_joints+1]

    return r_joints, r_tip, L_phalanxes, L_metacarpal




#function that creates different geometries of fingers
def generate_designs(n_joints,r_min,r_max,L_min_phalanx,L_min_palm,L_tot):

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
            r_joints[i] = random.uniform(r_min, r_joints[i-1])

    r_tip = random.uniform(r_min,r_joints[-1])

    #we return the parameters as a list
    individual = code_individual(r_joints,r_tip,L_phalanxes,L_metacarpal)

    return individual


#We define a function that checks if a design is admissible
def admissible_design(n_joints,r_min,r_max,L_min_phalanx,L_min_palm,L_tot,individual):

    r_joints, r_tip, L_phalanxes, L_metacarpal = decode_individual(n_joints,individual)

    #check if design is whithin boundaries
    if any([r < r_min for r in r_joints]) or any([r > r_max for r in r_joints]):
        return False
    
    if r_tip < r_min or r_tip > r_max:
        return False
    
    if any([l < L_min_phalanx for l in L_phalanxes]):
        return False
    
    if (L_metacarpal < L_min_palm):
        return False

    #check if the radiuses are decreasing
    for i in range(1,n_joints):
        if r_joints[i] > r_joints[i-1]:
            return False
        
    if r_tip > r_joints[-1]:
        return False
    
    #check if the total length is correct
    if abs(sum(L_phalanxes) + L_metacarpal - L_tot) > 1e-4:
        return False

    return True

#we define a function that directs towards the nearest admissible design
def penalty_function(n_joints,r_min,r_max,L_min_phalanx,L_min_palm,L_tot,individual):

    r_joints, r_tip, L_phalanxes, L_metacarpal = decode_individual(n_joints,individual)

    #we compute the distance from the boundaries
    distance = 0

    for i in range(n_joints):
        distance += ((r_min + r_max)/2 - r_joints[i])**2

    distance += ((r_min + r_max)/2 - r_tip)**2

    for i in range(n_joints):
        distance += (L_min_phalanx - L_phalanxes[i])**2

    if L_metacarpal < L_min_palm:
        distance += (L_tot -n_joints*L_min_phalanx - L_metacarpal)**2

    for i in range(1,n_joints):
        if r_joints[i] > r_joints[i-1]:
            distance += (r_joints[i] - r_joints[i-1])**2
    
    if r_tip > r_joints[-1]:
        distance += (r_tip - r_joints[-1])**2

    #we compute the distance between the total length and the sum of the phalanges
    if abs(sum(L_phalanxes) + L_metacarpal - L_tot) > 1e-4:
        distance += 10000*(sum(L_phalanxes) + L_metacarpal - L_tot)**2

    penalty = - 100*distance - 100

    return penalty





# function that computes the metrics for a given design
def evaluate_design(n_joints,r_min,r_max,L_min_phalanx,L_min_palm,L_tot,f_1, f_2, p_r, l_spring, l_0_spring, k_spring, pulley_radius_function, pulley_rotation, max_force, individual):
    
    #first we check if the design is admissible
    if not admissible_design(n_joints,r_min,r_max,L_min_phalanx,L_min_palm,L_tot,individual):
        #id not admissible we return a very high penalty
        penalty = penalty_function(n_joints,r_min,r_max,L_min_phalanx,L_min_palm,L_tot,individual)
        return (penalty, penalty)

    r_joints, r_tip, L_phalanxes, L_metacarpal = decode_individual(n_joints,individual)

    b_a_metacarpal = r_joints[0]

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

    foot_metric = compute_foot_metric(foot)


    return (hand_metric, foot_metric)

#we define the functions that allows to introduce constraints



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
    

def NSGA2_analysis(f_1,f_2,p_r,r_min,r_max,L_min_phalanx,L_min_palm,L_tot,l_spring,l_0_spring,k_spring,pulley_radius_function,pulley_rotation,max_force,n_pop,n_gen,cx_pb,mu_pb,seed=None):
    
    # first we select the seed
    random.seed(seed)

    #secondly we calculate the number of joints
    n_joints = 1

    # we create the boundaries for the independent variables
    LOW_BOUND = [r_min] * n_joints + [r_min] + [L_min_phalanx] * n_joints + [L_min_palm]
    UP_BOUND = [r_max] * n_joints + [r_max] + [L_tot - L_min_palm] * n_joints + [L_tot - L_min_phalanx*n_joints]


    # we define an individual class with a fitness multi-objective function, so to maximize the two metrics
    creator.create("FitnessMulti", base.Fitness, weights=(1.0,1.0))
    creator.create("Individual", list, fitness=creator.FitnessMulti)


    # we define the toolbox
    toolbox = base.Toolbox()

    # we define the attributes of the individual
    toolbox.register("attr_float", generate_designs,n_joints,r_min,r_max,L_min_phalanx,L_min_palm,L_tot)

    # structure initializers
    toolbox.register("individual", tools.initIterate, creator.Individual, toolbox.attr_float)
    toolbox.register("population", tools.initRepeat, list, toolbox.individual)


    toolbox.register("evaluate", evaluate_design, n_joints,r_min,r_max,L_min_phalanx,L_min_palm,L_tot,f_1, f_2, p_r, l_spring, l_0_spring, k_spring, pulley_radius_function, pulley_rotation, max_force)
    toolbox.register("mate", tools.cxSimulatedBinaryBounded, eta=30, low=LOW_BOUND, up=UP_BOUND)
    toolbox.register("mutate", tools.mutPolynomialBounded, eta=20, low=LOW_BOUND, up=UP_BOUND, indpb=1/len(UP_BOUND))
    toolbox.register("select", tools.selNSGA2)

    # initialize statistics object
    stats = tools.Statistics(lambda ind: ind.fitness.values)
    stats.register("avg", np.mean, axis=0)
    stats.register("std", np.std, axis=0)
    stats.register("min", np.min, axis=0)
    stats.register("max", np.max, axis=0)

    # initialize logbook object
    logbook = tools.Logbook()
    logbook.header = "gen", "evals", "std", "min", "avg", "max"

    # create population
    pop = toolbox.population(n=n_pop)

    # create history of population
    pop_hist = [[] for _ in range(n_gen)]

    # evaluate population
    invalid_ind = [ind for ind in pop if not ind.fitness.valid]
    fitnesses = toolbox.map(toolbox.evaluate, invalid_ind)
    for ind, fit in zip(invalid_ind, fitnesses):
        ind.fitness.values = fit

    # Compile statistics about the population
    record = stats.compile(pop)
    logbook.record(gen=0, evals=len(invalid_ind), **record)

    # save the population in the history
    pop_hist[0] = list(map(toolbox.clone, pop))

    # Begin the generational process
    for gen in range(1, n_gen):
        # Select the next generation individuals
        offspring = toolbox.select(pop, len(pop))

        # clone the selected individuals
        offspring = list(map(toolbox.clone, offspring))

        # Apply crossover and mutation on the offspring
        for child1, child2 in zip(offspring[::2], offspring[1::2]):
            if random.random() < cx_pb:
                toolbox.mate(child1, child2)
                del child1.fitness.values
                del child2.fitness.values

        for mutant in offspring:
            if random.random() < mu_pb:
                toolbox.mutate(mutant)
                del mutant.fitness.values

        # Evaluate the individuals with an invalid fitness
        invalid_ind = [ind for ind in offspring if not ind.fitness.valid]
        fitnesses = toolbox.map(toolbox.evaluate, invalid_ind)
        for ind, fit in zip(invalid_ind, fitnesses):
            ind.fitness.values = fit

        # Select the next generation population from parents and offspring
        pop = toolbox.select(pop + offspring, n_pop)

        # save the population in the history
        pop_hist[gen] = list(map(toolbox.clone, pop))

        # Compile statistics about the new population
        record = stats.compile(pop)
        logbook.record(gen=gen, evals=len(invalid_ind), **record)
        print(gen)

    # now we make the plots
    return pop, logbook, pop_hist



    


    
