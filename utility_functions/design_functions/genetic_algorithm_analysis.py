# We generate designs using a genetic algorithm to identify the Pareto front as fast as possible.

import matplotlib.pyplot as plt
import numpy as np
import random
from deap import base, creator, tools

def NSGA2_analysis(seed=None):
    random.seed(seed)

    # we create the boundaries for the independent variables
    LOW_BOUND = 0
    UP_BOUND = 3

    # important algorithm parameters
    N_POP = 100
    N_GEN = 100
    CXPB = 1
    MUPB = 1
    N_DIM = 2
    N_OBJ = 2

    # we define an individual class with a fitness multi-objective function
    creator.create("FitnessMulti", base.Fitness, weights=(1.0,) * N_OBJ)
    creator.create("Individual", list, fitness=creator.FitnessMulti)

    # we define a uniform function for initialization
    def attribute_generation(low, up, size=None):
        try:
            return [random.uniform(a, b) for a, b in zip(low, up)]
        except TypeError:
            return [random.uniform(a, b) for a, b in zip([low] * size, [up] * size)] 

    # we define the toolbox
    toolbox = base.Toolbox()

    # we define the attributes of the individual
    toolbox.register("attr_float", attribute_generation, LOW_BOUND, UP_BOUND, N_DIM)

    # structure initializers
    toolbox.register("individual", tools.initIterate, creator.Individual, toolbox.attr_float)
    toolbox.register("population", tools.initRepeat, list, toolbox.individual)

    # we define the evaluation function
    def evaluate_function(individual):
        x1 = individual[0]
        x2 = individual[1]

        f1 = x1**3 - x2**2
        f2 = 1 + x2 - x1**2

        return (f1, f2)
    
    #we define the functions that allows to introduce constraints
    def feasible(individual):
        statement1 = (individual[0] + individual[1] < 3)
        statement2 = (individual[0] - individual[1] < 0)
        if statement1 and statement2:
            return True
        return False
    
    def distance(individual):

        return -1*( (individual[0])**2 + (individual[1] - 3)**2)

    toolbox.register("evaluate", evaluate_function)
    toolbox.decorate("evaluate", tools.DeltaPenalty(feasible, -1000, distance))
    toolbox.register("mate", tools.cxSimulatedBinaryBounded, eta=1, low=LOW_BOUND, up=UP_BOUND)
    toolbox.register("mutate", tools.mutPolynomialBounded, eta=1, low=LOW_BOUND, up=UP_BOUND, indpb=1/N_DIM)
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
    pop = toolbox.population(n=N_POP)

    # create history of population
    pop_hist = [[] for _ in range(N_GEN)]

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
    for gen in range(1, N_GEN):
        # Select the next generation individuals
        offspring = toolbox.select(pop, len(pop))

        # clone the selected individuals
        offspring = list(map(toolbox.clone, offspring))

        # Apply crossover and mutation on the offspring
        for child1, child2 in zip(offspring[::2], offspring[1::2]):
            if random.random() < CXPB:
                toolbox.mate(child1, child2)
                del child1.fitness.values
                del child2.fitness.values

        for mutant in offspring:
            if random.random() < MUPB:
                toolbox.mutate(mutant)
                del mutant.fitness.values

        # Evaluate the individuals with an invalid fitness
        invalid_ind = [ind for ind in offspring if not ind.fitness.valid]
        fitnesses = toolbox.map(toolbox.evaluate, invalid_ind)
        for ind, fit in zip(invalid_ind, fitnesses):
            ind.fitness.values = fit

        # Select the next generation population from parents and offspring
        pop = toolbox.select(pop + offspring, N_POP)

        # save the population in the history
        pop_hist[gen] = list(map(toolbox.clone, pop))

        # Compile statistics about the new population
        record = stats.compile(pop)
        logbook.record(gen=gen, evals=len(invalid_ind), **record)

    # now we make the plots
    return pop, logbook, pop_hist














