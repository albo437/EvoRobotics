import pybullet as p
import pybullet_data
import pyrosim.pyrosim as pyrosim
import time
import numpy as np
import random
import matplotlib.pyplot as plt
import constants as c

physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)

p.setGravity(0,0,-9.8)
planeId = p.loadURDF("plane.urdf")
robotId = p.loadURDF("body.urdf")
p.loadSDF("world.sdf")
pyrosim.Prepare_To_Simulate(robotId)

BackLegAmplitude = c.BackLegAmplitude
BackLegFrequency = c.BackLegFrequency
BackLegPhaseOffset = c.BackLegPhaseOffset
FrontLegAmplitude = c.FrontLegAmplitude
FrontLegFrequency = c.FrontLegFrequency
FrontLegPhaseOffset = c.FrontLegPhaseOffset

backLegSensorValues = np.zeros(1000)
frontLegSensorValues = np.zeros(1000)

BackLegTargetAngles = np.zeros(1000)
FrontLegTargetAngles = np.zeros(1000)
for i in range(1000):
    BackLegTargetAngles[i] = BackLegAmplitude * np.sin(BackLegFrequency * i * np.pi/60 + BackLegPhaseOffset)
    FrontLegTargetAngles[i] = FrontLegAmplitude * np.sin(FrontLegFrequency * i * np.pi/60 + FrontLegPhaseOffset)

for i in range(1000):
    time.sleep(1/60)
    p.stepSimulation()
    pyrosim.Get_Touch_Sensor_Value_For_Link("BackLeg")
    backLegSensorValues[i] = pyrosim.Get_Touch_Sensor_Value_For_Link("BackLeg")
    pyrosim.Get_Touch_Sensor_Value_For_Link("FrontLeg")
    frontLegSensorValues[i] = pyrosim.Get_Touch_Sensor_Value_For_Link("FrontLeg")
    pyrosim.Set_Motor_For_Joint(robotId, b'Torso_BackLeg', p.POSITION_CONTROL, BackLegTargetAngles[i], 500)
    pyrosim.Set_Motor_For_Joint(robotId, b'Torso_FrontLeg', p.POSITION_CONTROL, FrontLegTargetAngles[i], 500)

np.save('data/backLegSensorValues.npy', backLegSensorValues)
np.save('data/frontLegSensorValues.npy', frontLegSensorValues)
p.disconnect()