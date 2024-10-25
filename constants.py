import numpy as np
gravity = 9.8
amplitude = np.pi/4
frequency = 20
phaseOffset = 0
maxForce = 500
steps = 1000

numSensorNeurons = 9
numMotorNeurons = 8
motorJointRange = 0.3

numGenerations = 100
populationSize = 10
childrenSize = 10
bounds = [-1, 1]
crossoverRate = 1
