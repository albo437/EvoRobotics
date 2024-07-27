import pybullet as p
import time

physicsClient = p.connect(p.GUI)
# p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)ß

for i in range(1000):
    time.sleep(1/60)
    print(i)
    p.stepSimulation()

p.disconnect()