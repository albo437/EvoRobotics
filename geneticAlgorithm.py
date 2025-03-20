import pybullet as p
from solution import SOLUTION
import constants as c
import os
import numpy as np

class GENETIC_ALGORITHM:
    def __init__(self):
        os.system("del fitness*.txt")
        os.system("del brain*.nndf")
        os.system("del body*.urdf")
        os.system("del world*.sdf")
        self.crossoverRate = 1
        self.mutationRate = 1/(c.numSensorNeurons + c.numHiddenNeurons)
        self.parents = []
        self.nextAvailableID = 0
        for i in range(c.populationSize):
            self.parents.append(SOLUTION(self.nextAvailableID, 0))
            self.nextAvailableID += 1
        self.Evaluate(self.parents)
    
    def Evolve(self):
        for i in range(c.numGenerations):
            self.Evolve_For_One_Generation(i)
        
    
    def Evolve_For_One_Generation(self, i):
        self.generateOffspring(i)
        self.Evaluate(self.children)
        self.Select()
        #print best fitness of generation
        print("\n")
        print(self.parents[0].fitnessList)
        print(self.parents[0].fitness)
        print("\n")

    
    def generateOffspring(self, i):
        self.children = []
        for i in range(c.childrenSize):
            # Select two random parents
            parent1Index = np.random.randint(0, len(self.parents))
            parent2Index = np.random.randint(0, len(self.parents))
            
            # Get parents weight matrices
            parent1ToHiddenWeights = self.parents[parent1Index].weightsToHidden
            parent2ToHiddenWeights = self.parents[parent2Index].weightsToHidden

            parent1ToMotorWeights = self.parents[parent1Index].weightsToMotor
            parent2ToMotorWeights = self.parents[parent2Index].weightsToMotor

            # Simulated Binary Crossover
            child1ToHiddenWeights, child2ToHiddenWeights = self.SBX(parent1ToHiddenWeights, parent2ToHiddenWeights)

            child1ToMotorWeights, child2ToMotorWeights = self.SBX(parent1ToMotorWeights, parent2ToMotorWeights)

            # Mutate children
            child1ToHiddenWeights = self.polynomialMutation(child1ToHiddenWeights)
            child2ToHiddenWeights = self.polynomialMutation(child2ToHiddenWeights)

            child1ToMotorWeights = self.polynomialMutation(child1ToMotorWeights)
            child2ToMotorWeights = self.polynomialMutation(child2ToMotorWeights)

            # Create children
            child1 = SOLUTION(self.nextAvailableID, i)
            child1.weightsToHidden = child1ToHiddenWeights
            child1.weightsToMotor = child1ToMotorWeights

            #test make all values 0
            # child1.weightsToHidden = np.zeros((c.numSensorNeurons, c.numHiddenNeurons))
            # child1.weightsToMotor = np.zeros((c.numHiddenNeurons, c.numMotorNeurons))
          

            self.nextAvailableID += 1
            child2 = SOLUTION(self.nextAvailableID, i)
            child2.weightsToHidden = child2ToHiddenWeights
            child2.weightsToMotor = child2ToMotorWeights

            # child2.weightsToHidden = np.zeros((c.numSensorNeurons, c.numHiddenNeurons))
            # child2.weightsToMotor = np.zeros((c.numHiddenNeurons, c.numMotorNeurons))     
            
            self.nextAvailableID += 1

            self.children.append(child1)
            self.children.append(child2)
            
    def SBX(self, parent1weights, parent2weights):
        # Simulated Binary Crossover
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
        # Polynomial Mutation
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
        for i in range(3):
            for individual in solutions:
                individual.Start_Simulation("DIRECT")
            for individual in solutions:
                individual.Wait_For_Simulation_To_End()
    
    def Select(self):
        population = self.parents + self.children
        population.sort(key=lambda x: x.fitness)
        self.parents = population[:c.populationSize]    
    
    def Show_Best(self):
        print(self.parents[0].fitnessList)
        print(self.parents[0].generation)
        self.parents[0].Start_Simulation("GUI")
