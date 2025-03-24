import pybullet as p
import pyrosim.pyrosim as pyrosim
from pyrosim.neuralNetwork import NEURAL_NETWORK
import constants as c
from sensor import SENSOR
from raySensor import RAY_SENSOR
from motor import MOTOR
import os

class ROBOT:
    """
    Class setups the robot's body, brain and sensors to be used by pyrosim, prepares pyrosim to simulate the robot and the robot to run.
    """
    def __init__(self, steps, solutionID):
        """
        Initialize the robot's body, brain and sensors to be used by pyrosim.
        Prepare pyrosim to simulate the robot and the robot to run.

        Parameters:
        steps (int): The number of steps the simulation will run.
        solutionID (int): The ID of the solution.

        Returns:
        None
        """
        self.steps = steps
        self.myID = solutionID
        self.motors = {}
        self.sensors = {}
        self.robotId = p.loadURDF("body"+str(solutionID)+".urdf")
        self.nn = NEURAL_NETWORK("brain"+str(solutionID)+".nndf")
        os.system("rm brain"+str(solutionID)+".nndf")  
        os.system("rm body"+str(solutionID)+".urdf")  
        pyrosim.Prepare_To_Simulate(self.robotId)

        self.Prepare_To_Sense()
        self.Prepare_To_Act()
    
    def Prepare_To_Sense(self):
        """
        Prepare the robot to sense the environment by creating the sensors to be used by pyrosim.
        """
        for linkName in pyrosim.linkNamesToIndices:
            if linkName != "cap":
                self.sensors[linkName] = SENSOR(linkName, self.steps)
            else:
                self.sensors["cap"] = RAY_SENSOR(linkName, self.steps)
    
    def Sense(self, t):
        """
        Sense the environment by getting the values of the sensors.

        Parameters:
        t (int): The current time step of the simulation.

        Returns:
        None
        """
        for sensor in self.sensors:
            if sensor != "cap":
                self.sensors[sensor].Get_Value(t)
            else:
                self.sensors["cap"].Get_Value(t)

    def Prepare_To_Act(self):
        """
        Prepare the robot to act by creating the motors to be used by pyrosim.
        """
        for jointName in pyrosim.jointNamesToIndices:
            self.motors[jointName] = MOTOR(jointName, self.steps)
    
    def Act(self):
        """
        Act by setting the desired angles of the motors based on the values of the motor neurons."

        Parameters:
        None

        Returns:
        None
        """
        for neuronName in self.nn.Get_Neuron_Names():
            if self.nn.Is_Motor_Neuron(neuronName):
                jointName = self.nn.Get_Motor_Neurons_Joint(neuronName).encode("utf-8")
                desiredAngle = self.nn.Get_Value_Of(neuronName) * c.motorJointRange
                self.motors[jointName].Set_Value(desiredAngle, self.robotId)
    
    def Think(self):
        """
        Think by updating the neural network."
        """
        self.nn.Update()
    
    def Get_Fitness(self):
        """
        Get the fitness of the robot based on its distance to the box.
        The fitness is written to a file.
        """
        basePositionAndOrientation = p.getBasePositionAndOrientation(self.robotId)
        basePosition = basePositionAndOrientation[0]
        zCoordinateOfLinkZero = -basePosition[2]
        f = open("tmp"+self.myID+".txt", "w")
        f.write(str(zCoordinateOfLinkZero))
        f.close()
        os.system("mv tmp"+self.myID+".txt fitness"+self.myID+".txt")  