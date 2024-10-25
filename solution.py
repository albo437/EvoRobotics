import pyrosim.pyrosim as pyrosim
import numpy as np
import os
import time 
import constants as c

class SOLUTION:
    def __init__(self, nextAvailableID):
        self.myID = nextAvailableID
        self.weights = np.random.uniform(-1, 1, (c.numSensorNeurons, c.numMotorNeurons))
    
    def Evaluate(self, directOrGui):
        self.Create_World()
        self.Generate_Body()
        self.Generate_Brain()
        os.system("python3 simulate.py " + directOrGui + " " + str(self.myID) +" 2&>1 &")
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
        x = 0
        y = 0
        z = 1

        pyrosim.Send_Cube(name = "Torso", pos = [x, y, z], size = [length, width, height])

        pyrosim.Send_Joint(name = "Torso_BackLeg" , parent= "Torso" , child = "BackLeg" , type = "revolute", position = [0, -0.5, 1], jointAxis = "1 0 0")
        pyrosim.Send_Cube(name = "BackLeg", pos = [0, -0.5, 0], size = [0.2,1,0.2])
        pyrosim.Send_Joint(name = "BackLeg_BackLowerLeg" , parent= "BackLeg" , child = "BackLowerLeg" , type = "revolute", position = [0, -1, 0], jointAxis = "1 0 0")
        pyrosim.Send_Cube(name = "BackLowerLeg", pos = [0, 0, -0.5], size = [0.2,0.2,1])

        pyrosim.Send_Joint(name = "Torso_FrontLeg" , parent= "Torso" , child = "FrontLeg" , type = "revolute", position = [0, 0.5, 1], jointAxis = "1 0 0")
        pyrosim.Send_Cube(name = "FrontLeg", pos = [0, 0.5, 0], size = [0.2,1,0.2])
        pyrosim.Send_Joint(name = "FrontLeg_FrontLowerLeg" , parent= "FrontLeg" , child = "FrontLowerLeg" , type = "revolute", position = [0, 1, 0], jointAxis = "1 0 0")
        pyrosim.Send_Cube(name = "FrontLowerLeg", pos = [0, 0, -0.5], size = [0.2,0.2,1])

        pyrosim.Send_Joint(name = "Torso_LeftLeg" , parent= "Torso" , child = "LeftLeg" , type = "revolute", position = [-0.5, 0, 1], jointAxis = "0 1 0")
        pyrosim.Send_Cube(name = "LeftLeg", pos = [-0.5, 0, 0], size = [1,0.2,0.2])
        pyrosim.Send_Joint(name = "LeftLeg_LeftLowerLeg" , parent= "LeftLeg" , child = "LeftLowerLeg" , type = "revolute", position = [-1, 0, 0], jointAxis = "0 1 0")
        pyrosim.Send_Cube(name = "LeftLowerLeg", pos = [0, 0, -0.5], size = [0.2,0.2,1])

        pyrosim.Send_Joint(name = "Torso_RightLeg" , parent= "Torso" , child = "RightLeg" , type = "revolute", position = [0.5, 0, 1], jointAxis = "0 1 0")
        pyrosim.Send_Cube(name = "RightLeg", pos = [0.5, 0, 0], size = [1,0.2,0.2])
        pyrosim.Send_Joint(name = "RightLeg_RightLowerLeg" , parent= "RightLeg" , child = "RightLowerLeg" , type = "revolute", position = [1, 0, 0], jointAxis = "0 1 0")
        pyrosim.Send_Cube(name = "RightLowerLeg", pos = [0, 0, -0.5], size = [0.2,0.2,1])
        
        pyrosim.End()

    def generateBrain(self):
        pyrosim.Start_NeuralNetwork("brain"+str(self.myID)+".nndf")

        pyrosim.Send_Sensor_Neuron(name = 0 , linkName = "Torso")
        pyrosim.Send_Sensor_Neuron(name = 1 , linkName = "BackLeg")
        pyrosim.Send_Sensor_Neuron(name = 2 , linkName = "FrontLeg")
        pyrosim.Send_Sensor_Neuron(name = 3 , linkName = "LeftLeg")
        pyrosim.Send_Sensor_Neuron(name = 4 , linkName = "RightLeg")
        pyrosim.Send_Sensor_Neuron(name = 5 , linkName = "BackLowerLeg")
        pyrosim.Send_Sensor_Neuron(name = 6 , linkName = "FrontLowerLeg")
        pyrosim.Send_Sensor_Neuron(name = 7 , linkName = "LeftLowerLeg")
        pyrosim.Send_Sensor_Neuron(name = 8 , linkName = "RightLowerLeg")

        pyrosim.Send_Motor_Neuron( name = 9 , jointName = "Torso_BackLeg")
        pyrosim.Send_Motor_Neuron( name = 10 , jointName = "Torso_FrontLeg")
        pyrosim.Send_Motor_Neuron( name = 11 , jointName = "Torso_LeftLeg")
        pyrosim.Send_Motor_Neuron( name = 12 , jointName = "Torso_RightLeg")
        pyrosim.Send_Motor_Neuron( name = 13 , jointName = "BackLeg_BackLowerLeg")
        pyrosim.Send_Motor_Neuron( name = 14 , jointName = "FrontLeg_FrontLowerLeg")
        pyrosim.Send_Motor_Neuron( name = 15 , jointName = "LeftLeg_LeftLowerLeg")
        pyrosim.Send_Motor_Neuron( name = 16 , jointName = "RightLeg_RightLowerLeg")

        for row in range(c.numSensorNeurons):
            for column in range(c.numMotorNeurons):
                pyrosim.Send_Synapse( sourceNeuronName = row , targetNeuronName = column + c.numSensorNeurons , weight = self.weights[row, column]) 


        pyrosim.End()
    
    def Mutate(self):
        row = np.random.randint(c.numSensorNeurons)
        column = np.random.randint(c.numMotorNeurons)
        self.weights[row, column] = np.random.uniform(-1, 1)
    
    def Set_ID(self, nextAvailableID):
        self.myID = nextAvailableID