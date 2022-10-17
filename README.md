# Micro-AG-for-mobile-robot

The maze.world file contains all the necessary elements to simulate the best trajectory resulting from the algorithm on the robot in a maze with two obstacles
in the gazebo simulator.

Prueba2.word is the base file that contains the maze scenario with the robot ready to be simulated in the gazebo.
The algorithm takes that file, adds a trajectory and saves it in the prueba4.world file to simulate it, this process is repeated n times until the best trajectory is found.
 
 
Implement other test scenarios
The world files can be modified to add more obstacles in future work, you can take the current obstacles as an example just change the pose (x,y,z)
and size in x, y, z of the new obstacle respectively.

You must also modify the collision function in the Micro AG.c file, you can take the current obstacles as an example, just change the x,y coordinates
for the initial and final coordinates of the new obstacle respectively.
