# Evolutionary Robotics

This project applies **Evolutionary Robotics (ER)** to train a robot, via a genetic algorithm, to autonomously find and approach a wall in a simulated environment. The system leverages **neuroevolution** to optimize the control policy of a robot in the physics simulator **PyBullet**.

## What is Evolutionary Robotics?

Evolutionary Robotics is an interdisciplinary field where techniques inspired by **biological evolution** (such as Genetic Algorithms and Evolutionary Strategies) are used to automatically design and improve robotic systems. These methods allow robots to **learn and adapt** in complex environments without human intervention, often producing novel and efficient behaviors.

---

## Experimental Setup

The goal of this project is to evolve a neural controller that enables a robot to:

> **Find a wall and move as close as possible to it**, regardless of the wall's initial location.

Simulations are carried out using [PyBullet](https://pybullet.org/), and behaviors are evolved over generations using a genetic algorithm.

### Robot Design

The robot is equipped with:

* **9 Sensor Neurons**:

  * 8 Touch sensors (one per limb).
  * 1 Ray sensor that detects how many rays (from a grid of rays) hit the target wall.
* **2 Hidden Layers** of 6 neurons each.
* **8 Motor Neurons** controlling the robot's joints.

The neural network receives sensor inputs and produces motor commands at every time step.

---

## Genetic Algorithm Details

* **Representation**: Each solution is a real-valued vector of 138 dimensions (the weights of the neural network).
* **Fitness Function**: Measures average distance to the wall over 10 randomized simulations (lower is better).
* **Selection Strategy**:

  * Parent population size (**μ**) = 20
  * Offspring population size (**λ**) = 20
  * **Generations**: 20
* **Variation Operators**:

  * Simulated Binary Crossover (SBX)
  * Polynomial Mutation
* **Survivor Selection**: (μ + λ)-selection — the top μ individuals from the combined population survive to the next generation.

All parameters can be modified in `constants.py`.

---

## Results

After training, the evolved robot typically displays the following behavior:

* Rotates until its ray sensor detects the wall.
* Moves directly toward the wall.
* Tilts to counteract turning momentum, enhancing stability.

This emergent behavior demonstrates successful adaptation via artificial evolution.

---

## Project Summary Video

[![Watch the summary video](https://youtu.be/ZAfwnkC0e-c.jpg)](https://youtu.be/ZAfwnkC0e-c)


---

## Saving Best Weights

You can save or reuse the best-performing neural network by calling:

```python
weights_to_hidden, weights_to_hidden2, weights_to_motor = ga.Get_Best()
```

These can be loaded into a new solution like so:

```python
best_solution = SOLUTION(0, 0)
best_solution.weightsToHidden = weights_to_hidden
best_solution.weightsToHidden2 = weights_to_hidden2
best_solution.weightsToMotor = weights_to_motor
best_solution.Start_Simulation("GUI")
```

---

## Getting Started

### Install Dependencies

```bash
pip install -r requirements.txt
```

### Run Simulation

You can run training directly from the notebook:

```python
from geneticAlgorithm import GENETIC_ALGORITHM
ga = GENETIC_ALGORITHM()
ga.Evolve()
ga.Show_Best()
```

---

## Repository Structure

```
EvoRobotics/
│
├── geneticAlgorithm.py   # Main GA implementation
├── solution.py           # Simulation and evaluation logic
├── constants.py          # Tunable hyperparameters
├── main.ipynb            # Jupyter notebook for training & analysis
├── requirements.txt      # Python dependencies
└── README.md             # Project overview
```

---

## 📜 License

MIT License — feel free to use and modify this project.