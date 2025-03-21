from geneticAlgorithm import GENETIC_ALGORITHM

ga = GENETIC_ALGORITHM()
ga.Evolve()
for i in range(3):
    print("show? y/n")
    if input() == "y":
        ga.Show_Best()
    else:
        break    


