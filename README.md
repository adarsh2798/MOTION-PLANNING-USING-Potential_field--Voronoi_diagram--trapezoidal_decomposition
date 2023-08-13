# MOTION-PLANNING-USING-Potential_field--Voronoi_diagram--trapezoidal_decomposition

In this project, 3 Algorithms for motion planning were implemented:

 1. Artificial Potential Field

 2. Generalized Voronoi Diagram

 3. Trapezoidal Decomposition

Simulation for all the 3 were performed in ROS_GAZEBO using turtlebot burger.

# 1. Artificial Potential Field

APF assumes that object in environment are associted with a replulsive potential and **goal**  is associated with an attarctive potential. The potential fucntions are chosen such that the attractive potential is minimum when robot reaches GOAL and repulsive potential is minimum when the robot is as far as away as possible from all the obstacles. Hence the  negative gradient of these functions, as everyone knwos, will point to the direction of it's minima. So combining both functions and giving the combined gradient of both these functions as a **control input** to robot, ensures it reaches goal point while avoiding the obstacles.

