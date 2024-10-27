import pybullet as p
import pyrosim.pyrosim as pyrosim
import constants as c

class WORLD:
    def __init__(self, solutionID):
        self.planeId = p.loadURDF("plane.urdf")
        p.loadSDF(f'world{solutionID}.sdf')
