import pyrosim.pyrosim as pyrosim
import pybullet as p
import numpy as np
import constants as c

class MOTOR:
    def __init__(self, jointName, steps):
        self.steps = steps
        self.jointName = jointName
    
    def Set_Value(self, desiredAngle, robotId):
        pyrosim.Set_Motor_For_Joint(
            bodyIndex = robotId,
            jointName = self.jointName,
            controlMode = p.POSITION_CONTROL,
            targetPosition = desiredAngle,
            maxForce = c.maxForce)


