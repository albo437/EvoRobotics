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
        self.tilt = np.zeros(self.steps)
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

            #calculate how much the object is tilted
            
            _, orientation_quaternion = p.getBasePositionAndOrientation(self.robot.robotId)

            # Convert quaternion to Euler angles
            roll, pitch, _ = p.getEulerFromQuaternion(orientation_quaternion)

            # Calculate "flippedness" using the cosines of roll and pitch
            flippedness = (np.cos(roll) + np.cos(pitch)) / 2.0
            self.tilt[t] = flippedness

            # print cap sensor value every 10 steps
            if t % 100 == 0 and self.directOrGui != "DIRECT":
                print(self.robot.sensors["cap"].values[t])

    def getFitness(self):
        #max height of robot
        maxHeight = -np.max(self.heights)
        
        #average tilt of robot
        averageTilt = np.mean(self.tilt)

        #distance to box
        robotPosition = p.getBasePositionAndOrientation(self.robot.robotId)[0][0:2]
        boxPosition = p.getBasePositionAndOrientation(1)[0][0:2]
        
        distance = np.sqrt((robotPosition[0] - boxPosition[0])**2 + (robotPosition[1] - boxPosition[1])**2)

        # #fitness function
        fitness = distance
    
        #write fitness to file
        with open("tmp" + str(self.myID) + ".txt", "a") as f:
            f.write(str(fitness) + "\n")
        os.system("move tmp"+self.myID+".txt fitness"+self.myID+".txt")

    def __del__(self):
        p.disconnect


