import numpy as np
import pyrosim.pyrosim as pyrosim
class SENSOR:
    def __init__(self, linkName, steps):
        self.linkName = linkName
        self.steps = steps
        self.values = np.zeros(steps)
        self.noneCount = 0

    def Get_Value(self, t):

        value = pyrosim.Get_Touch_Sensor_Value_For_Link(self.linkName)
        if value == None:
            self.values[t] = self.values[t-1] if t > 0 else -1
            self.noneCount += 1

        else:
            self.values[t] = value
    
    def Save_Values(self):
        np.save('data/' + self.linkName + 'SensorValues.npy', self.values)