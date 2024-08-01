import numpy as np
import time
import pybullet as p
import pybullet_data
import pyrosim.pyrosim as pyrosim
import constants as c
from world import WORLD
from robot import ROBOT

class SIMULATION:
    def __init__(self, steps):
        self.steps = steps
        self.physicsClient = p.connect(p.GUI)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
        p.setGravity(0,0,-c.gravity)
        self.world = WORLD()
        self.robot = ROBOT(self.steps)
        pyrosim.Prepare_To_Simulate(self.robot.robotId)
        self.Run()
    
    def Run(self):
        for t in range(self.steps):
            time.sleep(1/60)
            p.stepSimulation()
            self.robot.Sense(t)
            self.robot.Act(t)
            
    def __del__(self):
        p.disconnect


