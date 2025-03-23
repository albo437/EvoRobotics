import pyrosim.pyrosim as pyrosim
import numpy as np
import os
import time
import constants as c

class SOLUTION:
    def __init__(self, nextAvailableID, generation):
        self.myID = nextAvailableID
        self.weightsToHidden = np.random.uniform(-1, 1, (c.numSensorNeurons, c.numHiddenNeurons))
        self.weightsToMotor = np.random.uniform(-1, 1, (c.numHiddenNeurons, c.numMotorNeurons))
        self.fitnessList = []
        self.generation = generation
        self.fitness = 0
        self.flattenedWeights = np.concatenate((self.weightsToHidden.flatten(), self.weightsToMotor.flatten()))
    
    def Start_Simulation(self, directOrGui):
        self.generateWorld()
        self.generateBody()
        self.generateBrain()
        if directOrGui == "DIRECT":
            os.system(f"python3 simulate.py {directOrGui} {self.myID} &")
        else:
            os.system(f"python3 simulate.py {directOrGui} {self.myID}")
    
    def Wait_For_Simulation_To_End(self):
        fitnessFileName = f'fitness{self.myID}.txt'

        while not os.path.exists(fitnessFileName):
            time.sleep(0.01)

        content = None
        for _ in range(10):  
            try:
                with open(fitnessFileName, "r") as f:
                    content = f.read().strip()
                break  
            except PermissionError:
                time.sleep(0.05)  

        if content is None:
            raise PermissionError(f"Could not read {fitnessFileName} after multiple attempts.")

        try:
            fitness_value = float(content)
        except ValueError:
            raise ValueError(f"Invalid number in {fitnessFileName}: {content}")

        self.fitnessList.append(fitness_value)
        self.fitness = np.mean(self.fitnessList)

        time.sleep(0.01)  
        try:
            os.remove(fitnessFileName)  
        except PermissionError:
            time.sleep(0.1)  
            os.remove(fitnessFileName)

    def generateWorld(self):
        pyrosim.Start_SDF(f"world{self.myID}.sdf")

        length, width, height = 5, 10, 10
        x, y, z = 10, np.random.uniform(-20, 20), 5

        pyrosim.Send_Cube(name="box", pos=[x, y, z], size=[length, width, height])
        pyrosim.End()

    def generateBody(self):
        pyrosim.Start_URDF(f"body{self.myID}.urdf")
        pyrosim.Send_Cube(name="Torso", pos=[0, 0, 1], size=[2, 1, 0.2])

        joints = [
            ("Torso_cap", "Torso", "cap", "revolute", [0, 0, 1.1], "0 0 1"),
            ("Torso_BackLeg", "Torso", "BackLeg", "revolute", [0.7, -0.5, 1], "1 0 0"),
            ("BackLeg_BackLowerLeg", "BackLeg", "BackLowerLeg", "revolute", [0, -0.3, 0], "0 1 0"),
            ("Torso_FrontLeg", "Torso", "FrontLeg", "revolute", [0.7, 0.5, 1], "1 0 0"),
            ("FrontLeg_FrontLowerLeg", "FrontLeg", "FrontLowerLeg", "revolute", [0, 0.3, 0], "0 1 0"),
            ("Torso_LeftLeg", "Torso", "LeftLeg", "revolute", [-0.7, -0.5, 1], "1 0 0"),
            ("LeftLeg_LeftLowerLeg", "LeftLeg", "LeftLowerLeg", "revolute", [0, -0.3, 0], "0 1 0"),
            ("Torso_RightLeg", "Torso", "RightLeg", "revolute", [-0.7, 0.5, 1], "0 1 0"),
            ("RightLeg_RightLowerLeg", "RightLeg", "RightLowerLeg", "revolute", [0, 0.3, 0], "0 1 0"),
        ]
        
        for name, parent, child, jtype, position, axis in joints:
            pyrosim.Send_Joint(name=name, parent=parent, child=child, type=jtype, position=position, jointAxis=axis)
        
        cubes = [
            ("cap", [0, 0, 0], [2, 1, 0.1]),
            ("BackLeg", [0, -0.1, 0], [0.4, 0.2, 0.2]),
            ("BackLowerLeg", [0, 0, -0.5], [0.2, 0.2, 1]),
            ("FrontLeg", [0, 0.1, 0], [0.4, 0.2, 0.2]),
            ("FrontLowerLeg", [0, 0, -0.5], [0.2, 0.2, 1]),
            ("LeftLeg", [0, -0.1, 0], [0.4, 0.2, 0.2]),
            ("LeftLowerLeg", [0, 0, -0.5], [0.2, 0.2, 1]),
            ("RightLeg", [0, 0.1, 0], [0.4, 0.2, 0.2]),
            ("RightLowerLeg", [0, 0, -0.5], [0.2, 0.2, 1]),
        ]

        for name, pos, size in cubes:
            pyrosim.Send_Cube(name=name, pos=pos, size=size)
        
        pyrosim.End()

    def generateBrain(self):
        pyrosim.Start_NeuralNetwork(f"brain{self.myID}.nndf")

        for i, linkName in enumerate(["BackLeg", "FrontLeg", "LeftLeg", "RightLeg", "BackLowerLeg", "FrontLowerLeg", "LeftLowerLeg", "RightLowerLeg", "cap"]):
            pyrosim.Send_Sensor_Neuron(name=i, linkName=linkName)

        for i in range(9, 15):
            pyrosim.Send_Hidden_Neuron(name=i)

        for i, jointName in enumerate(["Torso_BackLeg", "Torso_FrontLeg", "Torso_LeftLeg", "Torso_RightLeg", "BackLeg_BackLowerLeg", "FrontLeg_FrontLowerLeg", "LeftLeg_LeftLowerLeg", "RightLeg_RightLowerLeg"], start=15):
            pyrosim.Send_Motor_Neuron(name=i, jointName=jointName)

        neuronCount = c.numSensorNeurons
        for i in range(c.numSensorNeurons):
            for j in range(c.numHiddenNeurons):
                pyrosim.Send_Synapse(sourceNeuronName=i, targetNeuronName=j + neuronCount, weight=self.weightsToHidden[i, j])

        neuronCount += c.numHiddenNeurons
        for i in range(c.numHiddenNeurons):
            for j in range(c.numMotorNeurons):
                pyrosim.Send_Synapse(sourceNeuronName=i + c.numSensorNeurons, targetNeuronName=j + neuronCount, weight=self.weightsToMotor[i, j])

        pyrosim.End()

    def Mutate(self):
        row = np.random.randint(c.numSensorNeurons)
        column = np.random.randint(c.numMotorNeurons)
        self.weightsToHidden[row, column] = np.random.uniform(-1, 1)
    
    def Set_ID(self, nextAvailableID):
        self.myID = nextAvailableID
