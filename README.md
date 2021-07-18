# First Project of the Research Track 2 course (Robotics Engineering, Unige)

The following repository contains the packages and the simulation environment for controlling a mobile robot in the Gazebo and vrep simulation environments.

## Project Installation:

This project requires both ROS1 and ROS2 to be install in the system. Please make sure you have both of them installed before following the instructions. There are three branches in this respository each one of them should be install in the following fashion.

1. Code available in **Main** branch is a ROS1 package should be install in ROS1 workspace {ros1_ws}/src. 
2. Code available in **ros2** branch is a ROS2 package should be install in ROS2 workspace {ros2_ws}/src.
3. Code available in **sourcefiles** branch is a set of source files that will be required to run the project. Please install them in your /root directory. 
4. This project requires both Gazebo and Vrep simulation environments, if they are not already installed in your system please install them by following given instructions: 

  **For Gazebo :** 
  
  * Please follow the instructions available on the respository.http://gazebosim.org/tutorials?tut=install_ubuntu
  
  **For Vrep/Coppeliasim :**
  
  * Download the PRO-EDU version from: http://www.coppeliarobotics.com/downloads.html
  * Vrep should be already integrated with ROS. You just need to launch the ROS master before running the V-REP (CoppeliaSim) software.
  * If there is any problem in building the plugin, you will need to recompile it by yourself: you can download it from here: CoppeliaRobotics/simExtROS (github.com).
  * Please first install xsltproc [(sudo) apt-get install xsltproc] and xmlschema [pip3 install xmlschema]
  * If specific messages/services/etc need to be supported, we have to make sure to edit files located in simExtROS/meta/, prior to recompilation. So please check if the following to message type is already included or not in the simExtROS/meta/message.txt 'geometry_msgs/Twist' 'nav_msgs/Odometry'
  * In order to build the packages, navigate to the catkin_ws folder and type:
  
    ```
    export COPPELIASIM_ROOT_DIR=~/path/to/coppeliaSim/folder
    ```
    
    ```
    catkin_make --cmake-args -DCMAKE_BUILD_TYPE=Release
    ```
    The packages should be generated and compiled to a library now. Copy the **devel/lib/libsimExtROS.so** file in the CoppeliaSim installation folder. The plugin is now ready     to be use.
    
  *  Now to launch Vep/CoppeliaSim you can run the following command. Make sure you are in the Vep/CoppeliaSim installation folder.

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
 #source /root/my_ros2/install/local_setup.bash
 ```
 7. If you are successfully able to complete all the previous steps then all the dependencies have been completed in order to run the project. 

The project in total three parts, they are not depended on each other so you individually run them by following given instructions. 

## Part 1: Running Mobile Robot Simulation in Gazeboo using action server.

In order to run this part please make sure you are in /root folder where you have already downloaded **rt2_assignment_1a.sh** bash file. Open the terminal and run the following command.

```
./rt2_assignment_1a.sh
```
After doing this you will see three terminal windows start appearing on the screen, including a Gazebo simulation with a mobile robot in it. Wait for the system load all the files. Find the terminal window with title **user_interface** which will be asking to press 1. Upon pressing 1, the mobile robot will start moving towards the randomly generated goal target. And if during the execution of this goal if you press 0 the robot will stop immediately. 

## Part 2: Running Mobile Robot Simulation in Gazeboo using ROS1/ROS2 Brigde

Part 2 of this project is very similar to part 1. One of the two differences is that half of this is build in ROS1 and the other half is build in ROS2. So we are using ROS1/ROS2 brigde to make these two sub-parts communicate with each other. The second difference is that in this part we are using simple server to assign the randomly generated target to the node which handling motion of the robot instead of using action server like in part 1.

In order to run this part please make sure you are in /root folder where you have already downloaded **rt2_assignment_1b.sh** bash file. Open the terminal and run the following command.

```
./rt2_assignment_1a.sh
```
After doing this you will see four terminal windows start appearing on the screen, including a Gazeebo simulation with a mobile robot in it. Wait for the system load all the files. Find the terminal window with title **user_interface** which will be asking to press 1. Upon pressing 1, the mobile robot will start moving towards the randomly generated goal target. And if during the execution of this goal if you press 0 the robot will not stop immediately since we are not using action server in this part. The robot will complete the last assigned target first before stopping.

## Part 3: Running Mobile Robot Simulation in Vrep/CoppeliaSim simulation

Part 3 of the project is also very similar with the part 1. The only difference is that we are using VRep/CoppeliaSim simulation environement instead of Gazebo. To run this part please following steps. 


1. Now make sure you are in /root folder where **'ros.sh'** is downloaded. open the terminal and run the following commands.
 
```
source ros.sh
```
2. Now go to the ROS1 workspace and run the master by using following command:
```
roscore &
```
3. Then go to the Vrep/CoppeliaSim installation folder and to launch Vrep/CoppeliaSim run the following command.

 ```
 ./coppeliasim.sh 
 ```
3. Wait for vrep simulation to get started. Then load the **'pioneerROS.ttt'** scene in the environment by clicking the **file/open scene..** button in the main menu. 
4. Once the scene is loaded start the simulation by clicking on the play button in the top center. Unlike Gazebo the simulation does not automatically start in the vrep simulator. 
5. To run the program open another terminal window, go to the root folder again and run the following commands.

```
source ros.sh
roslaunch rt2_assignment1 sim_vrep.launch

```
6. Once program is loaded it will be asking user to press 1. Upon pressing 1, the mobile robot will start moving towards the randomly generated goal target. And if during the execution of this goal if you press 0 the robot will not stop immediately. It will complete the last assigned target first and then stop.

 


