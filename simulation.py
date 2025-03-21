import time
import pybullet as p
import pybullet_data
import pyrosim.pyrosim as pyrosim
import constants as c
from world import WORLD
from robot import ROBOT
import os
import numpy as np
import shutil  # Import shutil for moving files

class SIMULATION:
    def __init__(self, steps, directOrGui, solutionID):
        self.myID = solutionID
        self.steps = steps
        self.directOrGui = directOrGui
        if directOrGui == "DIRECT":
            self.physicsClient = p.connect(p.DIRECT)
        else:
            self.physicsClient = p.connect(p.GUI)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
        p.setGravity(0, 0, -c.gravity)
        self.world = WORLD(solutionID)
        self.robot = ROBOT(self.steps, solutionID)
        pyrosim.Prepare_To_Simulate(self.robot.robotId)
        self.Run()
        
        world_file = f"world{solutionID}.sdf"
        if os.path.exists(world_file):
            os.remove(world_file)
    
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

            # Print cap sensor value every 100 steps
            if t % 100 == 0 and self.directOrGui != "DIRECT":
                print(self.robot.sensors["cap"].values[t])

    def getFitness(self):
        # Distance to box
        robotPosition = p.getBasePositionAndOrientation(self.robot.robotId)[0][0:2]
        boxPosition = p.getBasePositionAndOrientation(1)[0][0:2]
        
        distance = np.sqrt((robotPosition[0] - boxPosition[0])**2 + (robotPosition[1] - boxPosition[1])**2)

        # Write fitness to file
        tmp_filename = f"tmp{self.myID}.txt"
        fitness_filename = f"fitness{self.myID}.txt"

        fitness = distance
        
        with open(tmp_filename, "a") as f:
            f.write(str(fitness) + "\n")
        
        if os.path.exists(tmp_filename):
            shutil.move(tmp_filename, fitness_filename)

    def __del__(self):
        p.disconnect()
