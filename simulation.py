import time
import pybullet as p
import pybullet_data
import pyrosim.pyrosim as pyrosim
import constants as c
from world import WORLD
from robot import ROBOT
import os
import numpy as np

class SIMULATION:
    def __init__(self, steps, directOrGui, solutionID):
        self.myID = solutionID
        self.steps = steps
        self.directOrGui = directOrGui
        if (directOrGui == "DIRECT"):
            self.physicsClient = p.connect(p.DIRECT)
        else:
            self.physicsClient = p.connect(p.GUI)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
        p.setGravity(0,0,-c.gravity)
        self.world = WORLD(solutionID)
        self.robot = ROBOT(self.steps, solutionID)
        pyrosim.Prepare_To_Simulate(self.robot.robotId)
        self.Run()
        os.system("del world"+str(solutionID)+".sdf")
    
    def Run(self):
        self.heights = np.zeros(self.steps)
        self.capSensor = np.zeros(self.steps)
        for t in range(self.steps):
            if self.directOrGui != "DIRECT":
                time.sleep(1/60)
            p.stepSimulation()
            self.robot.Sense(t)
            self.robot.Think()
            self.robot.Act()
            #collect height of robot
            basePositionAndOrientation = p.getBasePositionAndOrientation(self.robot.robotId)
            basePosition = basePositionAndOrientation[0]
            zCoordinateOfLinkZero = basePosition[2]
            self.heights[t] = zCoordinateOfLinkZero

            #collect touch sensor value
            self.capSensor[t] = self.robot.sensors["cap"].values[t]



    def getFitness(self):
        #max height of robot
        maxHeight = -np.max(self.heights)
        #values of capsensor equal to -1
        Noflipped = 1
        if 1 in self.capSensor:
            Noflipped = -1

        #distance from origin
        # basePositionAndOrientation = p.getBasePositionAndOrientation(self.robot.robotId)
        # basePosition = basePositionAndOrientation[0]
        # distance = (basePosition[0]**2 + basePosition[1]**2)

        # #fitness function
        fitness = maxHeight*Noflipped
    

        #write fitness to file
        f = open("tmp"+self.myID+".txt", "w")
        f.write(str(fitness))
        f.close()
        os.system("move tmp"+self.myID+".txt fitness"+self.myID+".txt")

        #final height of robot
        # finalHeight = -self.heights[-1]
        # f = open("tmp"+self.myID+".txt", "w")
        # f.write(str(finalHeight))
        # f.close()
        # os.system("move tmp"+self.myID+".txt fitness"+self.myID+".txt")

        #average height of robot
        # averageHeight = -np.mean(self.heights)
        # f = open("tmp"+self.myID+".txt", "w")
        # f.write(str(averageHeight))
        # f.close()
        # os.system("move tmp"+self.myID+".txt fitness"+self.myID+".txt")


    def __del__(self):
        p.disconnect


