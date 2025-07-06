import pyrosim.pyrosim as pyrosim
import numpy as np
import os
import time
import constants as c

class SOLUTION:
    """
    Class that contains a robot's brain weights and its fitness.
    It also contains the methods to generate the world, body and brain of the robot.
    """
    def __init__(self, nextAvailableID, generation):
        """
        Initialize the solution with random weights for the brain.

        Parameters:
        nextAvailableID (int): The next available ID for the solution.
        generation (int): The generation of the solution.

        Returns:
        None
        """
        self.myID = nextAvailableID
        self.weightsToHidden = np.random.uniform(-1, 1, (c.numSensorNeurons, c.numHiddenNeurons))
        self.weightsToHidden2 = np.random.uniform(-1, 1, (c.numHiddenNeurons, c.numHiddenNeurons2))
        self.weightsToMotor = np.random.uniform(-1, 1, (c.numHiddenNeurons, c.numMotorNeurons))
        self.fitnessList = []
        self.generation = generation
        self.fitness = 0
        self.distanceList = []
        self.distance = 0
    
    def Start_Simulation(self, directOrGui):
        """
        Start the simulation of the robot by generating the world, body and brain of the robot.
        The simulation is started in a new process that runs the simulate.py script.

        Parameters:
        directOrGui (str): The mode of the simulation, it can be "DIRECT" or "GUI".

        Returns:
        None
        """
        self.generateWorld()
        self.generateBody()
        self.generateBrain()
        if directOrGui == "DIRECT":
            os.system(f"python3 simulate.py {directOrGui} {self.myID} &")
        else:
            os.system(f"python3 simulate.py {directOrGui} {self.myID}")
    
    def Wait_For_Simulation_To_End(self):
        """
        Wait for the simulation to end and read the fitness value from the fitness file.

        Parameters:
        None

        Returns:
        None
        """
        fitnessFileName = f'fitness{self.myID}.txt'

        while not os.path.exists(fitnessFileName):
            time.sleep(0.01)

        content = None
        # Read the fitness file, it contains a single line with n values separeted by commas.
        # We will read it multiple times in case the file is not ready yet.
        for _ in range(10):  
            try:
                with open(fitnessFileName, "r") as f:
                    content = f.readline().strip()
                if content:
                    content = content.split(",")
                break  
            except PermissionError:
                time.sleep(0.05)  

        if content is None:
            raise PermissionError(f"Could not read {fitnessFileName} after multiple attempts.")

        try:
            for i in range(len(content)):
                content[i] = float(content[i])
        except ValueError:
            raise ValueError(f"Invalid number in {fitnessFileName}: {content}")

        for i in range(len(content)):
            try:
                content[i] = float(content[i])
            except ValueError:
                raise ValueError(f"Invalid number in {fitnessFileName}: {content[i]}")
        
        fitness_value = np.sum(content)

        self.fitnessList.append(fitness_value)
        self.fitness = np.mean(self.fitnessList)

        self.distanceList.append(content[0])
        self.distance = np.mean(self.distanceList)

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

        for i, linkName in enumerate(["BackLeg", "FrontLeg", "LeftLeg", "RightLeg","BackLowerLeg", 
                                      "FrontLowerLeg", "LeftLowerLeg", "RightLowerLeg", "cap"]):
            pyrosim.Send_Sensor_Neuron(name=i, linkName=linkName)

        for i in range(c.numSensorNeurons, c.numSensorNeurons + c.numHiddenNeurons + c.numHiddenNeurons2):
            pyrosim.Send_Hidden_Neuron(name=i)

        for i, jointName in enumerate(["Torso_BackLeg", "Torso_FrontLeg", "Torso_LeftLeg", 
                                       "Torso_RightLeg", "BackLeg_BackLowerLeg", "FrontLeg_FrontLowerLeg", 
                                       "LeftLeg_LeftLowerLeg", "RightLeg_RightLowerLeg"], 
                                       start=c.numSensorNeurons + c.numHiddenNeurons + c.numHiddenNeurons2):
            pyrosim.Send_Motor_Neuron(name=i, jointName=jointName)

        # Connect sensor neurons to layer 1 of hidden neurons
        neuronCount = c.numSensorNeurons
        for i in range(c.numSensorNeurons):
            for j in range(c.numHiddenNeurons):
                pyrosim.Send_Synapse(sourceNeuronName=i, targetNeuronName=j + neuronCount, weight=self.weightsToHidden[i, j])

        # Connect layer 1 of hidden neurons to layer 2 of hidden neurons
        neuronCount += c.numHiddenNeurons
        for i in range(c.numHiddenNeurons):
            for j in range(c.numHiddenNeurons2):
                pyrosim.Send_Synapse(sourceNeuronName=i + c.numSensorNeurons, targetNeuronName=j + neuronCount, weight=self.weightsToHidden2[i, j])

        # Connect layer 2 of hidden neurons to motor neurons
        neuronCount += c.numHiddenNeurons2
        for i in range(c.numHiddenNeurons2):
            for j in range(c.numMotorNeurons):
                pyrosim.Send_Synapse(sourceNeuronName=i + c.numSensorNeurons + c.numHiddenNeurons, 
                                     targetNeuronName=j + neuronCount, 
                                     weight=self.weightsToMotor[i, j])

        pyrosim.End()
    
    def Set_ID(self, nextAvailableID):
        self.myID = nextAvailableID
