# MOTION-PLANNING-USING-Potential_field--Voronoi_diagram--trapezoidal_decomposition

In this project, 3 Algorithms for motion planning were implemented:

 1. Artificial Potential Field

 2. Generalized Voronoi Diagram

 3. Trapezoidal Decomposition

Simulation for all the 3 were performed in ROS_GAZEBO using turtlebot burger.

# 1. Artificial Potential Field

APF assumes that object in environment are associted with a replulsive potential and **goal**  is associated with an attarctive potential. The potential fucntions are chosen such that the attractive potential is minimum when robot reaches GOAL and repulsive potential is minimum when the robot is as far as away as possible from all the obstacles. Hence the  negative gradient of these functions, as everyone knwos, will point to the direction of it's minima. So combining both functions and giving the combined gradient of both these functions as a **control input** to robot, ensures it reaches goal point while avoiding the obstacles.
The potential functions chosen for this case are:

![equation](https://latex.codecogs.com/gif.latex?U_%7Batt%7D%3D%5Cfrac%7B1%7D%7B2%7D%5Cepsilon%28q-q_%7Bgoal%7D%29%5E2)

![equation](https://latex.codecogs.com/gif.latex?U_%7Brep%7D%3D%5Cleft%5C%7B%5Cbegin%7Bmatrix%7D%20%5Cfrac%7B1%7D%7B2%7D%5Ceta%5B%5Cfrac%7B1%7D%7BD_i%28q%29%7D-%5Cfrac%7B1%7D%7BQ_i%5E*%7D%5D%5E2%20%5C%3A%5C%3A%2CD_i%28q%29%3CQ%5E*%5C%5C%200%5C%3A%5C%3A%5C%3A%5C%3A%5C%3A%5C%3A%5C%3A%2Celse%20%5Cend%7Bmatrix%7D%5Cright.)

Here, **q** is the current configuration/posiotion of robot, **D_i(q)** is distance to obstacle-i, and **Q** is a threshold on when to use the repulsive threshold.

Below is the environment in which APF was implemenetd, and the resulting Potential field generated is also shown.

<p align="center">
  <img src="https://github.com/adarsh2798/MOTION-PLANNING-USING-Potential_field--Voronoi_diagram--trapezoidal_decomposition/blob/main/assignment2/simulation_results/gazebo_world.png" />
</p>



<p align="center">
  <img src="https://github.com/adarsh2798/MOTION-PLANNING-USING-Potential_field--Voronoi_diagram--trapezoidal_decomposition/blob/main/quiver.png" />
</p>
