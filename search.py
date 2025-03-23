from geneticAlgorithm import GENETIC_ALGORITHM
from solution import SOLUTION

ga = GENETIC_ALGORITHM()
ga.Evolve()
for i in range(10):
    print("show? y/n")
    if input() == "y":
        ga.Show_Best()
    else:
        break    


weights_to_hidden = ga.Get_Best()
weights_to_motor = ga.Get_Best()

best_solution = SOLUTION(0, 0)
best_solution.weightsToHidden = weights_to_hidden
best_solution.weightsToMotor = weights_to_motor
best_solution.Start_Simulation("GUI")

