import time
import pybullet as p
import pybullet_data
import pyrosim.pyrosim as pyrosim
import constants as c
from world import WORLD
from robot import ROBOT
import os
import numpy as np
import shutil
from sensor import SENSOR

class SIMULATION:
    """
    Class that setups the simulation environment and runs the simulation.
    """
    def __init__(self, steps, directOrGui, solutionID):
        """
        Initialize the simulation environment and run the simulation.

        Parameters:
        steps (int): The number of steps the simulation will run.
        directOrGui (str): The mode of the simulation, it can be "DIRECT" or "GUI".
        solutionID (int): The ID of the solution.

        Returns:
        None
        """
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
        """
        Run the simulation for a number of steps given by self.steps.
        """
        self.heights = np.zeros(self.steps)
        self.tilt = np.zeros(self.steps)
        for t in range(self.steps):
            if self.directOrGui != "DIRECT":
                time.sleep(1/120)
            p.stepSimulation()
            self.robot.Sense(t)
            self.robot.Think()
            self.robot.Act()

            # # Print cap sensor value every 100 steps
            # if t % 100 == 0 and self.directOrGui != "DIRECT":
            #     print(self.robot.sensors["cap"].values[t])
            # # Print sensors with None values
            # if t % 1000 == 0:
            #     none_count = 0
            #     for sensor in self.robot.sensors.values():
            #         if type(sensor) is SENSOR:
            #             none_count += sensor.noneCount
            #     print("None in sensor values",none_count)

    def getFitness(self):
        """
        Get the fitness of the robot based on its distance to the box.
        The fitness is written to a file.

        Parameters:
        None

        Returns:
        None
        """
        # Distance to box
        robotPosition = p.getBasePositionAndOrientation(self.robot.robotId)[0][0:2]
        boxPosition = p.getBasePositionAndOrientation(1)[0][0:2]
        
        distance = np.sqrt((robotPosition[0] - boxPosition[0])**2 + (robotPosition[1] - boxPosition[1])**2)

        # Write fitness to file
        tmp_filename = f"tmp{self.myID}.txt"
        fitness_filename = f"fitness{self.myID}.txt"

        fitness = str(distance) #+","+ str(- 100 * np.sum(self.robot.sensors["cap"].values) / c.steps)
        
        with open(tmp_filename, "a") as f:
            f.write(fitness + "\n")
        
        if os.path.exists(tmp_filename):
            shutil.move(tmp_filename, fitness_filename)

    def __del__(self):
        p.disconnect()
