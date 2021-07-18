# First Project of the Research Track 2 course (Robotics Engineering, Unige)

The package contains the nodes and the simulation environment for controlling a mobile robot in the Gazebo and vrep simulation environments.

## Project Installation:

This project requires both ROS1 and ROS2 to be install in the system. Please make sure you have both of them installed before following the instructions. There are three branches in this respository each one of them should be install in the following fashion.

1. Code available in **Main** branch is a ROS1 package should be install in ROS1 workspace (ws). 
2. Code available in **ros2** branch is a ROS2 package should be install in ROS2 workspace (ws).
3. Code available in **sourcefiles** branch is a set of source files that will be required to run the simulation. Please install them in your /root directory. 
4. This project requires both Gazeebo and Vrep simulation environments, if they are not already installed please install them by followuing instructions: 


  **For Gazeebo :** 
  
  * Please follow the instructions available on the respository.http://gazebosim.org/tutorials?tut=install_ubuntu
  
  **For Vrep/Coppeliasim :**
  
  * Download the PRO-EDU version from: http://www.coppeliarobotics.com/downloads.html
  * Vrep should be already integrated with ROS. You just need to launch the ROS master before running the V-REP (CoppeliaSim) software.
  * If there is any problem in building the plugin, you will need to recompile it by yourself: you can download it from here: CoppeliaRobotics/simExtROS (github.com).
  * Please first install xsltproc [(sudo) apt-get install xsltproc] and xmlschema [pip3 install xmlschema]
  * If specific messages/services/etc. need to be supported, make sure to edit files located in simExtROS/meta/, prior to recompilation.
  * In order to build the packages, navigate to the catkin_ws folder and type:
    ```
    export COPPELIASIM_ROOT_DIR=~/path/to/coppeliaSim/folder
    ```
    
    ```
    catkin_make --cmake-args -DCMAKE_BUILD_TYPE=Release
    ```
    The packages should be generated and compiled to a library now. Copy the **devel/lib/libsimExtROS.so** file in the CoppeliaSim installation folder. The plugin is now ready     to be use.
  *  Now to launch Vep/CoppeliaSim by run following command. Make sure you are in the Vep/CoppeliaSim installation folder.

     ```
     ./coppeliasim.sh 
     
     ```
 5. Once this is done. Next dependency that need to install is ROS1/ROS2 bridge package. Please install the package available on this repository: https://github.com/ros2/ros1_bridge 
 6. Once setup 5 is done we need to modify our .bashrc, to have the possibility of using both ROS1 and ROS2 frameworks at the same time (in different
terminal). For that please comment all bash files lines. 

 ```
 #source /opt/ros/noetic/setup.bash
 #source /root/my_ros/devel/setup.bash
 #source /opt/ros/foxy/setup.bash
 source /usr/share/colcon_cd/function/colcon_cd.sh
 #source /root/my_ros2/install/local_setup.bash
 
 ```
 7. If you are successfully able to complete all the previous steps then all the dependencies have been completed in order to run the project. 

## Part 1: Running Mobile Robot Simulation in Gazeboo using action server.

In order to run the code for part 1 of this project please make sure you are in /root folder where you have already downloaded **rt2_assignment_1a.sh** bash file. Open the terminal and run the following command.

```
./rt2_assignment_1a.sh

```
You will not that multiple terinal windows will start appearing on the screen. 

## Part 2: Using ROS/ROS2 Brigde

```
roslaunch rt2_assignment1 sim.launch
```

## Part 3: Using Vrep/CoppeliaSim Simulation

```
roslaunch rt2_assignment1 sim.launch
```



