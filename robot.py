import pybullet as p
import pyrosim.pyrosim as pyrosim
from pyrosim.neuralNetwork import NEURAL_NETWORK
import constants as c
from sensor import SENSOR
from motor import MOTOR
import os

class ROBOT:
    def __init__(self, steps, solutionID):
        self.steps = steps
        self.myID = solutionID
        self.motors = {}
        self.sensors = {}
        self.robotId = p.loadURDF("body"+str(solutionID)+".urdf")
        self.nn = NEURAL_NETWORK("brain"+str(solutionID)+".nndf")
        os.system("del brain"+str(solutionID)+".nndf")
        os.system("del body"+str(solutionID)+".urdf")
        pyrosim.Prepare_To_Simulate(self.robotId)

        self.Prepare_To_Sense()
        self.Prepare_To_Act()
    
    def Prepare_To_Sense(self):
        for linkName in pyrosim.linkNamesToIndices:
            self.sensors[linkName] = SENSOR(linkName, self.steps)
    
    def Sense(self, t):
        for sensor in self.sensors:
            self.sensors[sensor].Get_Value(t)

    def Prepare_To_Act(self):
        for jointName in pyrosim.jointNamesToIndices:
            self.motors[jointName] = MOTOR(jointName, self.steps)
    
    def Act(self):
        for neuronName in self.nn.Get_Neuron_Names():
            if self.nn.Is_Motor_Neuron(neuronName):
                jointName = self.nn.Get_Motor_Neurons_Joint(neuronName).encode("utf-8")
                desiredAngle = self.nn.Get_Value_Of(neuronName) * c.motorJointRange
                self.motors[jointName].Set_Value(desiredAngle, self.robotId)
    
    def Think(self):
        self.nn.Update()
    
    def Get_Fitness(self):
        basePositionAndOrientation = p.getBasePositionAndOrientation(self.robotId)
        basePosition = basePositionAndOrientation[0]
        zCoordinateOfLinkZero = -basePosition[2]
        f = open("tmp"+self.myID+".txt", "w")
        f.write(str(zCoordinateOfLinkZero))
        f.close()
        os.system("move tmp"+self.myID+".txt fitness"+self.myID+".txt")


        