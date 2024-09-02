from simulation import SIMULATION
import constants as c
import sys

directOrGui = sys.argv[1]
simulation = SIMULATION(c.steps, directOrGui)
simulation.getFitness()
