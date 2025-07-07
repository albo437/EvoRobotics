from solution import SOLUTION
import constants as c
import numpy as np
import os


class GENETIC_ALGORITHM:
    """
    Class that contains the genetic
    algorithm to evolve the weights of the robot's brain.
    """
    def __init__(self):
        """
        Deletes all files from previous runs, setup the genetic algorithm parameters, 
        generate the initial population and evaluate it.

        Parameters:
        None

        Returns:
        None
        """
        os.system("rm fitness*.txt")
        os.system("rm brain*.nndf")
        os.system("rm body*.urdf")
        os.system("rm world*.sdf")
        self.crossoverRate = 1
        self.mutationRate = 0.01
        self.parents = []
        self.nextAvailableID = 0
        for i in range(c.populationSize):
            self.parents.append(SOLUTION(self.nextAvailableID, 0))
            self.nextAvailableID += 1
        self.Evaluate(self.parents)
    
    def Evolve(self):
        """
        Evolve the population for a number of generations given by c.numGenerations,
        each step of the evolution is done by calling the Evolve_For_One_Generation method.

        Parameters:
        None

        Returns:
        None
        """
        for i in range(c.numGenerations):
            self.Evolve_For_One_Generation(i)
        
    
    def Evolve_For_One_Generation(self, i):
        """
        Evolve the population for one generation, this is done by generating offspring from the parents,
        evaluating the offspring, selecting the best individuals and updating the parents list.

        Parameters:
        i (int): The current generation number.

        Returns:
        None
        """
        self.generateOffspring(i)
        self.Evaluate(self.children)
        self.Select()
        #print best fitness of generation
        for j in range(10):
            print("\n")
        print("Generation: ", i)
        print(self.parents[0].fitnessList)
        print(self.parents[0].fitness)
        for j in range(10):
            print("\n")

    
    def generateOffspring(self, i):
        """
        Generate offspring from the parents using Simulated Binary Crossover and Polynomial Mutation.
        2 children are generated from each pair of parents.

        Parameters:
        i (int): The current generation number.

        Returns:
        None
        """
        self.children = []
        for i in range(c.childrenSize):
            # Select two random parents
            parent1Index = np.random.randint(0, len(self.parents))
            parent2Index = np.random.randint(0, len(self.parents))
            
            # Get parents weight matrices
            parent1ToHiddenWeights = self.parents[parent1Index].weightsToHidden
            parent2ToHiddenWeights = self.parents[parent2Index].weightsToHidden

            parent1HiddenToHidden2Weights = self.parents[parent1Index].weightsToHidden2
            parent2HiddenToHidden2Weights = self.parents[parent2Index].weightsToHidden2

            parent1ToMotorWeights = self.parents[parent1Index].weightsToMotor
            parent2ToMotorWeights = self.parents[parent2Index].weightsToMotor

            # Simulated Binary Crossover
            child1ToHiddenWeights, child2ToHiddenWeights = self.SBX(parent1ToHiddenWeights, parent2ToHiddenWeights)
            child1ToMotorWeights, child2ToMotorWeights = self.SBX(parent1ToMotorWeights, parent2ToMotorWeights)
            child1HiddenToHidden2Weights, child2HiddenToHidden2Weights = self.SBX(parent1HiddenToHidden2Weights, parent2HiddenToHidden2Weights)

            # Mutate children
            child1ToHiddenWeights = self.polynomialMutation(child1ToHiddenWeights)
            child2ToHiddenWeights = self.polynomialMutation(child2ToHiddenWeights)

            child1HiddenToHidden2Weights = self.polynomialMutation(child1HiddenToHidden2Weights)
            child2HiddenToHidden2Weights = self.polynomialMutation(child2HiddenToHidden2Weights)

            child1ToMotorWeights = self.polynomialMutation(child1ToMotorWeights)
            child2ToMotorWeights = self.polynomialMutation(child2ToMotorWeights)

            # Create children
            child1 = SOLUTION(self.nextAvailableID, i)
            child1.weightsToHidden = child1ToHiddenWeights
            child1.weightsToHidden2 = child1HiddenToHidden2Weights
            child1.weightsToMotor = child1ToMotorWeights

            self.nextAvailableID += 1
            child2 = SOLUTION(self.nextAvailableID, i)
            child2.weightsToHidden = child2ToHiddenWeights
            child2.weightsToHidden2 = child2HiddenToHidden2Weights
            child2.weightsToMotor = child2ToMotorWeights   
            
            self.nextAvailableID += 1

            self.children.append(child1)
            self.children.append(child2)
            
    def SBX(self, parent1weights, parent2weights):
        """
        Perform Simulated Binary Crossover on two parent weight matrices.

        Parameters:
        parent1weights (numpy array): The weight matrix of the first parent.
        parent2weights (numpy array): The weight matrix of the second parent.

        Returns:
        child1Weights (numpy array): The weight matrix of the first child.
        child2Weights (numpy array): The weight matrix of the second child.
        """
        child1Weights = parent1weights
        child2Weights = parent2weights

        if np.random.rand() < self.crossoverRate:
            for i in range(parent1weights.shape[0]):
                for j in range(parent1weights.shape[1]):
                    eta_c = 20
                    u = np.random.rand()
                    if u <= 0.5:
                        beta = (2 * u)**(1 / (eta_c + 1))
                    else:
                        beta = (1 / (2 * (1 - u)))**(1 / (eta_c + 1))
                    beta = beta * ((-1) ** np.random.randint(0, 1))  
                    child1Weights[i, j] = 0.5 * ((1 + beta) * parent1weights[i, j] + (1 - beta) * parent2weights[i, j])
                    child2Weights[i, j] = 0.5 * ((1 - beta) * parent1weights[i, j] + (1 + beta) * parent2weights[i, j])

        child1Weights = np.clip(child1Weights, c.bounds[0], c.bounds[1])
        child2Weights = np.clip(child2Weights, c.bounds[0], c.bounds[1])       
        return child1Weights, child2Weights
    
    def polynomialMutation(self, childWeights):
        """
        Perform Polynomial Mutation on a weight matrix.

        Parameters:
        childWeights (numpy array): The weight matrix of the child.

        Returns:
        childWeights (numpy array): The weight matrix of the child after mutation.
        """
        eta_m = 20
        for i in range(childWeights.shape[0]):
            for j in range(childWeights.shape[1]):
                u = np.random.rand()
                if u <= 0.5:
                    delta = (2 * u + (1 - 2 * u) * (1 - (childWeights[i, j] - c.bounds[0]) / (c.bounds[1] - c.bounds[0]))**(eta_m + 1))**(1 / (eta_m + 1)) - 1
                else:
                    delta = 1 - (2 * (1 - u) + 2 * (u - 0.5) * ((c.bounds[1] - childWeights[i, j]) / (c.bounds[1] - c.bounds[0]))**(eta_m + 1))**(1 / (eta_m + 1))
                childWeights[i, j] = childWeights[i, j] + delta
        
        childWeights = np.clip(childWeights, c.bounds[0], c.bounds[1])

        return childWeights

    def Evaluate(self, solutions):
        """
        Evaluate the fitness of the solutions by running the simulation for each solution 10 times.
        The simulation is run 10 times so that the final fitness reflects the average performance of the solution.

        Parameters:
        solutions (list): A list of SOLUTION objects to evaluate.

        Returns:
        None
        """
        for i in range(c.simulationRuns):
            for individual in solutions:
                individual.Start_Simulation("DIRECT")
            for individual in solutions:
                individual.Wait_For_Simulation_To_End()
    
    def Select(self):
        """
        Select the c.populationSize best individuals from the combined parent + children population based on their fitness.

        Parameters:
        None

        Returns:
        None
        """
        population = self.parents + self.children
        population.sort(key=lambda x: x.fitness)
        self.parents = population[:c.populationSize]    
    
    def Show_Best(self):
        """
        Show the best solution in a GUI simulation."
        """
        print(self.parents[0].fitnessList)
        print(self.parents[0].fitness)
        self.parents[0].Start_Simulation("GUI")

    def Get_Best(self):
        """
        Get the weights of the best solution.

        Returns:
        weightsToHidden (numpy array): The weights of the best solution from the input layer to the hidden layer.
        weightsToMotor (numpy array): The weights of the best solution from the hidden layer to the motor layer.
        """
        return self.parents[0].weightsToHidden, self.parents[0].weightsToHidden2, self.parents[0].weightsToMotor

