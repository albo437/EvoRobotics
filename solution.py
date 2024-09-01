import pyrosim.pyrosim as pyrosim
import numpy as np
import os

class SOLUTION:
    def __init__(self):
        # 3 by 2 matrix of random wieghts
        self.weights = np.random.uniform(-1, 1, (3, 2))
    
    def Evaluate(self):
        self.generateWorld()
        self.generateBody()
        self.generateBrain()
        os.system("python3 simulate.py")

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
        pyrosim.Start_NeuralNetwork("brain.nndf")

        pyrosim.Send_Sensor_Neuron(name = 0 , linkName = "Torso")
        pyrosim.Send_Sensor_Neuron(name = 1 , linkName = "BackLeg")
        pyrosim.Send_Sensor_Neuron(name = 2 , linkName = "FrontLeg")

        pyrosim.Send_Motor_Neuron( name = 3 , jointName = "Torso_BackLeg")
        pyrosim.Send_Motor_Neuron( name = 4 , jointName = "Torso_FrontLeg")

        for row in range(3):
            for column in range(2):
                pyrosim.Send_Synapse( sourceNeuronName = row , targetNeuronName = column + 3 , weight = self.weights[row, column]) 


        pyrosim.End()