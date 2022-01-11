# Research Track 1: third assignment 
Author: Monica Fossati s4697871, Robotics Engineering Unige

In this project, a robot moves in an environment initially unknown to it. Thanks to laser scanners mounted on board the robot, it is able to build a map of the environment and move without encountering obstacles.
It is required that the robot can move in 3 different modes:
*  setting the coordinates of a goal to be achieved and the robot would have to dynamically calculate a path to reach it 
*  controlling it via keyboard 
*  controlling it via keyboard with assistance to avoid obstacles

## Installing and running
Some packages are required so, if they aren't already installed, you have to run the following commands.

Install xterm:
```bash
$ sudo apt install xterm
```
Download package:
```bash
$ git clone https://github.com/CarmineD8/slam_gmapping.git
```
Install ros navigation stack:
```bash
$ sudo apt-get install ros-<your_ros_distro>-navigation
```
Install teleop twist keyboard package:
```bash
$ sudo apt-get install ros-hydro-teleop-twist-keyboard
```
In order to run multiple nodes at the same time, a launch script is provided:
```final.launch```:
```
<launch>
    <include file="$(find final_assignment)/launch/simulation_gmapping.launch" />
    <include file="$(find final_assignment)/launch/move_base.launch" />
    <node pkg="final_assignment" type="final_UI" name="final_UI" output="screen" required="true" launch-prefix="xterm -e"/>
    <node pkg="teleop_twist_keyboard" type="teleop_twist_keyboard.py" name="teleop" output="screen" launch-prefix="xterm -e" respawn="true">
    <remap from="cmd_vel" to="prov_cmd_vel"/>
  </node>
</launch>
```
The user can easily run the project with the command:
```bash
$ roslaunch final_assignment final.launch 
```
## Environment and mapping
The robot moves in the environment in the figure (Gazebo view):

![true_map](https://user-images.githubusercontent.com/62377263/148927975-c272cc18-bd40-4af4-822e-f6440199b8a9.JPG)

Initially the robot does not have a map of the surrounding environment, but can build it thanks to the laser scanners it is equipped with and thanks to the ```gmapping``` package.
The final map, visible on Rviz is as follows:

![map](https://user-images.githubusercontent.com/62377263/148928409-d8d45436-5e83-4284-8f01-b21346316c74.JPG)

## Project structure
Ater running ```final.launch```, seven nodes are active:

![graph3](https://user-images.githubusercontent.com/62377263/148929443-39082c28-c785-43f7-88e0-f35ff369175c.JPG)

The ```\cmd_vel``` topic of ```\teleop``` node is remapped on ```\prov_cmd_vel```. In this way the velocity imposed by the user via keyboard isn't immediately imposed to the robot, but it's controlled by ```\final_UI``` node.




