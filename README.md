# Volocopter Case Study
The provided scripts are named based on their function and includes a simulation (**sim.py**) for the dynamics (**dynamics.py**) of a octorotor (**robot.py**), facilitated by controllers (**control.py**) and a planner (**planner.py**). The simulation is run via **main.py** by running ```python main.py``` from the root of this repository. The simulation will produce three plots: 1) 3D position, 2) position tracking and 3) attitude (i.e. roll, pitch and yaw) of the octorotor. 

The provided scripts were developed using Python 3.8.10. 

## Task 1
See **dynamics.py** for the Euler Lagrange formulation of the octorotors dynamics

## Task 2
See the included derivation of my control allocation matrix in **control_allocation_matrix.jpg**.

## Task 3
See the suggestion for the inverse of my control allocation matrix in **control_allocation_matrix.jpg**. Alternatively, in the "px4_generate_mixer" directory I've included a script to produce the desired "mixer" matrix. From the root of the "px4_generate_mixer directory", simply run ```python px_generate_mixers.py -f octa_plus.toml --sixdof``` in terminal.

## Task 4
See the computeReferenceAttitude() (line 70) function in the **control.py** script for the second order attitude reference dynamics. 

## Task 5
See the **control.py** script for the translational and attitude controller (I chose to design a cascade control architecture that takes translational commands and decomposes a desired thrust vector into its respective attitude commands. A low-level attitude controller then controls to the attitude commands. 

## Task 6
See **sim.py** for the gains of each controller.

## Task 7
As mentioned earlier in **Task 5**, I designed a high-level translational controller. As such, my simulation is characterized by three phases of 5, 10 and 15 second durations, each with associated desired positions. A smooth polynomial is followed to traverse from initial position to goal position. See **sim.py** for details on the simulation parameters. 

## Task 8
Not attempted 

## Task 9
Not attempted

Thank you for spending time with my scripts and I look forward to chatting with you through my solutions! 
