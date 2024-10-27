import pyrosim.pyrosim as pyrosim
import numpy as np
import os
import time 
import constants as c

class SOLUTION:
    def __init__(self, nextAvailableID):
        self.myID = nextAvailableID
        self.weightsToHidden = np.random.uniform(-1, 1, (c.numSensorNeurons, c.numHiddenNeurons))
        self.weightsToMotor = np.random.uniform(-1, 1, (c.numHiddenNeurons, c.numMotorNeurons))
    
    def Evaluate(self, directOrGui):
        self.generateWorld()
        self.generateBody()
        self.generateBrain()
        os.system(f'start /B py simulate.py {directOrGui} {str(self.myID)}')
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
        if directOrGui == "DIRECT":
            os.system("start /B python simulate.py " + directOrGui + " " + str(self.myID))
        else:
            os.system("python simulate.py " + directOrGui + " " + str(self.myID))
 
    def Wait_For_Simulation_To_End(self):
        fitnessFileName = f'fitness{str(self.myID)}.txt'
        while not os.path.exists(fitnessFileName):
            time.sleep(0.01)
        f = open("fitness"+str(self.myID)+".txt", "r")
        self.fitness = float(f.read())
        f.close()
        os.system("del fitness"+str(self.myID)+".txt")

    def generateWorld(self):
        pyrosim.Start_SDF("world"+str(self.myID)+".sdf")

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
        pyrosim.Start_URDF("body"+str(self.myID)+".urdf")
        #Dimensions of the box
        length = 1
        width = 1
        height = 1
        #Position of the box
        x = 0
        y = 0
        z = 1

        pyrosim.Send_Cube(name = "Torso", pos = [x, y, z], size = [length, width, height])

        pyrosim.Send_Joint(name = "Torso_cap", parent = "Torso", child = "cap", type = "revolute", position = [0, 0, 1.5], jointAxis = "0 0 1")
        pyrosim.Send_Cube(name = "cap", pos = [0, 0, 0], size = [1, 1, 0.2])

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

        pyrosim.Send_Sensor_Neuron(name = 0 , linkName = "BackLeg")
        pyrosim.Send_Sensor_Neuron(name = 1 , linkName = "FrontLeg")
        pyrosim.Send_Sensor_Neuron(name = 2 , linkName = "LeftLeg")
        pyrosim.Send_Sensor_Neuron(name = 3 , linkName = "RightLeg")
        pyrosim.Send_Sensor_Neuron(name = 4 , linkName = "BackLowerLeg")
        pyrosim.Send_Sensor_Neuron(name = 5 , linkName = "FrontLowerLeg")
        pyrosim.Send_Sensor_Neuron(name = 6 , linkName = "LeftLowerLeg")
        pyrosim.Send_Sensor_Neuron(name = 7 , linkName = "RightLowerLeg")

        pyrosim.Send_Hidden_Neuron( name = 8 )
        pyrosim.Send_Hidden_Neuron( name = 9 )
        pyrosim.Send_Hidden_Neuron( name = 10 )
        pyrosim.Send_Hidden_Neuron( name = 11 )
        pyrosim.Send_Hidden_Neuron( name = 12 )
        pyrosim.Send_Hidden_Neuron( name = 13 )

        pyrosim.Send_Motor_Neuron( name = 14, jointName = "Torso_BackLeg")
        pyrosim.Send_Motor_Neuron( name = 15 , jointName = "Torso_FrontLeg")
        pyrosim.Send_Motor_Neuron( name = 16 , jointName = "Torso_LeftLeg")
        pyrosim.Send_Motor_Neuron( name = 17 , jointName = "Torso_RightLeg")
        pyrosim.Send_Motor_Neuron( name = 18 , jointName = "BackLeg_BackLowerLeg")
        pyrosim.Send_Motor_Neuron( name = 19 , jointName = "FrontLeg_FrontLowerLeg")
        pyrosim.Send_Motor_Neuron( name = 20 , jointName = "LeftLeg_LeftLowerLeg")
        pyrosim.Send_Motor_Neuron( name = 21 , jointName = "RightLeg_RightLowerLeg")

        pyrosim.Send_Sensor_Neuron(name = 22 , linkName = "cap")

        neuronCount = c.numSensorNeurons
        for i in range(c.numSensorNeurons):
            for j in range(c.numHiddenNeurons):
                pyrosim.Send_Synapse( sourceNeuronName = i , targetNeuronName = j + neuronCount, weight = self.weightsToHidden[i, j] )
        
        neuronCount += c.numHiddenNeurons
        for i in range(c.numHiddenNeurons):
            for j in range(c.numMotorNeurons):
                pyrosim.Send_Synapse( sourceNeuronName = i + c.numSensorNeurons , targetNeuronName = j + neuronCount , weight = self.weightsToMotor[i, j] )

        pyrosim.End()
    def Mutate(self):
        row = np.random.randint(c.numSensorNeurons)
        column = np.random.randint(c.numMotorNeurons)
        self.weights[row, column] = np.random.uniform(-1, 1)
    
    def Set_ID(self, nextAvailableID):
        self.myID = nextAvailableID