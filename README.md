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

Below is the environment in which APF was implemenetd, and the resulting Potential field generated is also shown. **(GOAL=(0,4))**

<p align="center">
  <img src="https://github.com/adarsh2798/MOTION-PLANNING-USING-Potential_field--Voronoi_diagram--trapezoidal_decomposition/blob/main/assignment2/simulation_results/gazebo_world.png" />
</p>



<p align="center">
  <img src="https://github.com/adarsh2798/MOTION-PLANNING-USING-Potential_field--Voronoi_diagram--trapezoidal_decomposition/blob/main/quiver.png" />
</p>

## SIMULATION OF APF
![Alt Text](https://github.com/adarsh2798/MOTION-PLANNING-USING-Potential_field--Voronoi_diagram--trapezoidal_decomposition/blob/main/assignment2/simulation_results/succecssful_APF_RUN_with_NO_smoth_diff_STARTPOINT.gif)

# 2. Generalized Voronoi Diagram (GVD)

GVD is used to decompose the work space with respect to a bunch of objetcs such that each associated voronoi region consists of points that are closest to that object that any other objetcs. This means intersection of multiple voronoi regions forms the vorornoi edges such that points on that edge are as far as away as possible from all nearby objects, meaning EQUIDITANT from all objects. Hence the voronoi edgeds provide a path for robot to navigate through work space such that it is as far away as possible from nearby objects.

In this project, to get GVD, the workspace was dicretized into grid-cells with some resolution and **BRUSHFIRE** algorithm was used to generate voronoi diagram. Brushfire algorithm basically is just similar to an actual brishfire spreading through surroundings. If we start from the boundary cells of all the obstacles, being intialized to 1's, and at each iteration the neighbouring cells(4-way neighbors) are updated to 1 then we can create an effect of brushfire spreading through surrounding workspace and starting at obstacles. The cells where 2 wavefronts collide then gives the gridcells that are equiditant to their 2 nearest obstacles, meaning as far away as possible from those 2 obstacles. Doing this we can get grid cells that correspond to voronoi edges.

After the GVD craeted, one can decompose those grid cells into a Graph/Roadmap with relevenat edges and vertices. After that, give start and goal point, **Dijkstra's** algorithm was used to find shortest path. 

See below image for path generated by GVD:

<p align="center">
  <img src="https://github.com/adarsh2798/MOTION-PLANNING-USING-Potential_field--Voronoi_diagram--trapezoidal_decomposition/blob/main/assignment2/simulation_results/GVD_path.png" />
</p>

## SIMULATION OF GVD
![Alt Text](https://github.com/adarsh2798/MOTION-PLANNING-USING-Potential_field--Voronoi_diagram--trapezoidal_decomposition/blob/main/assignment2/simulation_results/GVD_RUN.gif)


# 2. Trapezoidal Decomposition

Here, trapezoidal decompositon of the workspace was used. In this the free workspace is deocmposed to **trapezoidal** regions. Trapezoid is concex region which ensures any 2 points inside it canbe joined by a simple staright line and hence making it a useful decompostion for motion planning algorithms.

To generate the trapezoidal decomposition of the workspace, **SWEEP LINE** algorithm was used. Once deompcotion was obtained, a graph/Roadmap was generated by joining the trapezoidal region centers and edge centers of teh regions. 

Below image shows decompositon obtained in the GAZEBO WORLD I used for simulation. Further the path taken by robot in simulation is also shown.

<p align="center">
  <img src="https://github.com/adarsh2798/MOTION-PLANNING-USING-Potential_field--Voronoi_diagram--trapezoidal_decomposition/blob/main/assignment2/simulation_results/trapez_run_latest.png" />
</p>
