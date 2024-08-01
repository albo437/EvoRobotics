import pyrosim.pyrosim as pyrosim
import pybullet as p
import numpy as np
import constants as c

class MOTOR:
    def __init__(self, jointName, steps):
        self.steps = steps
        self.jointName = jointName
        print(jointName)
        self.Prepare_To_Act()
    
    def Prepare_To_Act(self):
        self.motorValues = np.linspace(-np.pi, np.pi, self.steps)
        self.amplitude = np.pi/4
        self.frequency = 20

        if self.jointName == b'Torso_FrontLeg':
            self.frequency *= 0.5

        self.phaseOffset = 0
        for i in range(self.steps):
            self.motorValues[i]= self.amplitude * np.sin(self.frequency * self.motorValues[i] + self.phaseOffset)
    
    def Set_Value(self, t, robotId):
        pyrosim.Set_Motor_For_Joint(
            bodyIndex = robotId,
            jointName = self.jointName,
            controlMode = p.POSITION_CONTROL,
            targetPosition = self.motorValues[t],
            maxForce = 500)
    
    def Save_Values(self):
        np.save('data/' + self.jointName + 'MotorValues.npy', self.motorValues)

