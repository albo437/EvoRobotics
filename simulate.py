import pybullet as p
import time

physicsClient = p.connect(p.GUI)
p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)

for i in range(100):
    time.sleep(1/60)
    print(i)
    p.stepSimulation()

p.disconnect()