#simulate.py
from simulation import SIMULATION
import constants as c
import sys

directOrGui = sys.argv[1]
solutionID = sys.argv[2]
simulation = SIMULATION(c.steps, directOrGui, solutionID)
simulation.getFitness()
