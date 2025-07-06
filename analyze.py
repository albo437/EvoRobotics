from geneticAlgorithm import GENETIC_ALGORITHM

# Create 10 instances of the GENETIC_ALGORITHM class and store them in a list
ga_instances = [GENETIC_ALGORITHM() for _ in range(10)]
# Evolve each instance
for ga in ga_instances:
    ga.Evolve()
# Collect the fitness values from each instance
distance_values = [ga.parents[0].distance for ga in ga_instances]

# Append the fitness values to a text file
with open("stats.txt", "a") as f:
    f.write(distance_values.__str__() + "\n")
