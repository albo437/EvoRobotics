from geneticAlgorithm import GENETIC_ALGORITHM
from solution import SOLUTION

def main():
    """
    Main function to run the genetic algorithm
    It will evolve the population for a number of generations and then show the best solution,
    fitness is based on final proximity to the target wall.

    The position of the target wall is 10 units in the x direction and a random position in the y direction in the range [-20, 20]. 
    The dimensions of the wall are 5 units in the x direction, 10 units in the y direction, and 10 units in the z direction.

    The robot is placed at the origin and has 9 sensor neurons, 6 hidden neurons, and 8 motor neurons. 8 of the sensor neurons are
    touch sensors that detect contact with any object in the simulation. The other sensor neuron is a ray sensor that detects the
    if the target wall is in the rays path, the ray is placed on top of the robot facing the positive x direction at start of simulation.

    The porpuse of the simulation is to evolve the weights of the robot's brain to make it move towards the target wall, it should
    reach the wall no matter the position of the wall in the y direction.
    """

    ga = GENETIC_ALGORITHM()
    ga.Evolve()

    for i in range(10): # Show best solution in different test cases.
        print("show? y/n")
        if input() == "y":
            ga.Show_Best()
        else:
            break

    weights_to_hidden, weights_to_motor = ga.Get_Best()

    solution = SOLUTION(0, 0)
    solution.weightsToHidden = weights_to_hidden
    solution.weightsToMotor = weights_to_motor
