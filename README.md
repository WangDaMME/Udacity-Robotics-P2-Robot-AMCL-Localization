# Udacity-Robotics-P3-Robot-AMCL-Localization
Robot Localization using Adaptive MCL Method

## Overview
In this project, I learned to utilize ROS AMCL package to accurately localize a 2-wheel mobile robot inside a map with the PoseArray in the Gazebo simulation environments. This project is carried out with several aspects of robotic software engineering with a focus on ROS:

  * Create a ROS package that launches a custom robot model in a custom Gazebo world.
  * Utilize the ROS AMCL package and the Tele-Operation / Navigation Stack to localize the robot.
  * Explore, add, and tune specific parameters corresponding to each package to achieve the best possible localization results.

## Result
![whereamii (1)-min](https://user-images.githubusercontent.com/48291391/62151823-3ba2a500-b2cf-11e9-9505-59c8be1a1db1.gif)

## Directory Structure
```
   catkin_ws                           # Catkin Workspace
    ├── my_robot                       # my_robot package                   
    │   ├── launch                     # launch folder for launch files   
    │   │   ├── robot_description.launch
    │   │   ├── world.launch
    │   │   ├── amcl.launch            # Adaptive MCL Algorithm parameters
    │   ├── config     
    │   ├── maps                       # map files for navigation
    │   │   ├── pgm_map.pgm            # portable gray map
    │   │   ├── pgm_map.yaml
    │   ├── meshes                     # meshes folder for sensors
    │   │   ├── hokuyo.dae
    │   ├── urdf                       # urdf folder for xarco files
    │   │   ├── twowheel_robot.gazebo
    │   │   ├── twowheel_robot.xacro
    │   ├── world                      # world folder for world files
    │   │   ├── world.world
    │   ├── rviz                      # rviz folder for rviz files
    │   │   ├── my_robot_rviz.rviz
    │   ├── CMakeLists.txt             # compiler instructions
    │   ├── package.xml                # package info
```

## Launch The Project

  1. Clone/Download this project.
  2. Build the project
```
  $ cd /home/workspace/catkin_ws
  $ catkin_make
```
  3. Launch the robot inside the world
```
  $ cd /home/workspace/catkin_ws/
  $ source devel/setup.bash
  $ roslaunch my_robot world.launch
```
  4. Launch amcl to locate and show the robot
```
  $ cd /home/workspace/catkin_ws/
  $ source devel/setup.bash
  $ roslaunch my_robot amcl.launch
```
  5. Run Teleop Package to move the robot and observe the location
```
  $ rosrun teleop_twist_keyboard teleop_twist_keyboard.py  
```
