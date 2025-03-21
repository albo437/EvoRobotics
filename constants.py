import numpy as np
gravity = 9.8
amplitude = np.pi/4
frequency = 20
phaseOffset = 0
maxForce = 500
steps = 3000

numSensorNeurons = 9
numMotorNeurons = 8
numHiddenNeurons = 6
motorJointRange = 0.5

numGenerations = 2
populationSize = 2
childrenSize = 2
bounds = [-1, 1]
crossoverRate = 1
