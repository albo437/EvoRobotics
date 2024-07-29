import pybullet as p
import pybullet_data
import pyrosim.pyrosim as pyrosim
import time
import numpy as np

physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)

p.setGravity(0,0,-9.8)
planeId = p.loadURDF("plane.urdf")
robotId = p.loadURDF("body.urdf")
p.loadSDF("world.sdf")
pyrosim.Prepare_To_Simulate(robotId)

backLegSensorValues = np.zeros(1000)

for i in range(1000):
    time.sleep(1/60)
    p.stepSimulation()
    pyrosim.Get_Touch_Sensor_Value_For_Link("BackLeg")
    backLegSensorValues[i] = pyrosim.Get_Touch_Sensor_Value_For_Link("BackLeg")

print(backLegSensorValues)

p.disconnect()