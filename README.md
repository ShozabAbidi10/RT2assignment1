# First Project of the Research Track 2 course (Robotics Engineering, Unige)

The package contains the nodes and the simulation environment for controlling a mobile robot in the Gazebo and vrep simulation environments.

## Project Installation:

This project requires both ROS1 and ROS2 to be install in the system. Please make sure you have both of them installed before following the instructions. There are three branches in this respository each one of them should be install in the following fashion.

1. Code available in **Main** branch is a ROS1 package should be install in ROS1 workspace (ws). 
2. Code available in **ros2** branch is a ROS2 package should be install in ROS2 workspace (ws).
3. Code available in **sourcefiles** branch is a set of source files that will be required to run the simulation. Please install them in your /root directory. 
4. This project requires both Gazeebo and Vrep simulation environments, if they are not already installed please install them by followuing instructions available on the respository.
  **For Gazeebo :** 
  http://gazebosim.org/tutorials?tut=install_ubuntu
  
  **For Vrep/Coppeliasim :**
  


## Part 1: Using Action Server

```
roslaunch rt2_assignment1 sim.launch
```

## Part 2: Using ROS/ROS2 Brigde

```
roslaunch rt2_assignment1 sim.launch
```

## Part 3: Using Vrep/CoppeliaSim Simulation

```
roslaunch rt2_assignment1 sim.launch
```



