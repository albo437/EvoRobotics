import numpy as np
import pyrosim.pyrosim as pyrosim

class RAY_SENSOR:
    def __init__(self, linkName, steps):
        self.linkName = linkName
        self.steps = steps
        self.values = np.zeros(steps)
    
    def Get_Value(self, t):
        self.values[t] = pyrosim.Get_Ray_Sensor_Value_For_Link(2, self.linkName)
    