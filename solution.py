import pyrosim.pyrosim as pyrosim
import numpy as np
import os
import time

class SOLUTION:
    def __init__(self, nextAvailableID):
        # 3 by 2 matrix of random wieghts
        self.myID = nextAvailableID
        self.weights = np.random.uniform(-1, 1, (3, 2))
    
    def Evaluate(self, directOrGui):
        self.Create_World()
        self.Generate_Body()
        self.Generate_Brain()
        os.system("python3 simulate.py " + directOrGui + " " + str(self.myID) +" &")
        fitnessFileName = f"fitness{str(self.myID)}"
        while not os.path.exists(fitnessFileName):
            time.sleep(0.01)
        fitnessFile = open(fitnessFileName, "r")
        self.fitness = float(fitnessFile.read())
        print(self.fitness)
        fitnessFile.close()
    
    def Start_Simulation(self, directOrGui):
        self.generateWorld()
        self.generateBody()
        self.generateBrain()
        os.system("python3 simulate.py " + directOrGui + " " + str(self.myID) +" &")
    
    def Wait_For_Simulation_To_End(self):
        fitnessFileName = "fitness"+str(self.myID)+".txt"
        while not os.path.exists(fitnessFileName):
            time.sleep(0.01)
        f = open("fitness"+str(self.myID)+".txt", "r")
        self.fitness = float(f.read())
        f.close()
        os.system("rm fitness"+str(self.myID)+".txt")

    def generateWorld(self):
        pyrosim.Start_SDF("world.sdf")

        #Dimensions of the box
        length = 1
        width = 1
        height = 1
        #Position of the box
        x = -4
        y = -4
        z = 0.5

        pyrosim.Send_Cube(name = "box", pos = [x, y, z], size = [length, width, height])
        pyrosim.End()

    def generateBody(self):
        pyrosim.Start_URDF("body.urdf")
        #Dimensions of the box
        length = 1
        width = 1
        height = 1
        #Position of the box
        x = 1.5
        y = 0
        z = 1.5

        pyrosim.Send_Cube(name = "Torso", pos = [x, y, z], size = [length, width, height])
        pyrosim.Send_Joint( name = "Torso_BackLeg" , parent= "Torso" , child = "BackLeg" , type = "revolute", position = [1, 0, 1])
        pyrosim.Send_Cube(name = "BackLeg", pos = [-0.5, 0, -0.5], size = [length, width, height])
        pyrosim.Send_Joint( name = "Torso_FrontLeg" , parent= "Torso" , child = "FrontLeg" , type = "revolute", position = [2, 0, 1])
        pyrosim.Send_Cube(name = "FrontLeg", pos = [0.5, 0, -0.5], size = [length, width, height])
        
        pyrosim.End()

    def generateBrain(self):
        pyrosim.Start_NeuralNetwork("brain"+str(self.myID)+".nndf")

        pyrosim.Send_Sensor_Neuron(name = 0 , linkName = "Torso")
        pyrosim.Send_Sensor_Neuron(name = 1 , linkName = "BackLeg")
        pyrosim.Send_Sensor_Neuron(name = 2 , linkName = "FrontLeg")

        pyrosim.Send_Motor_Neuron( name = 3 , jointName = "Torso_BackLeg")
        pyrosim.Send_Motor_Neuron( name = 4 , jointName = "Torso_FrontLeg")

        for row in range(3):
            for column in range(2):
                pyrosim.Send_Synapse( sourceNeuronName = row , targetNeuronName = column + 3 , weight = self.weights[row, column]) 


        pyrosim.End()
    
    def Mutate(self):
        row = np.random.randint(3)
        column = np.random.randint(2)
        self.weights[row, column] = np.random.uniform(-1, 1)
    
    def Set_ID(self, nextAvailableID):
        self.myID = nextAvailableID