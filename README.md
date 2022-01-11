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
After running ```final.launch```, seven nodes are active:

![graph3](https://user-images.githubusercontent.com/62377263/148929443-39082c28-c785-43f7-88e0-f35ff369175c.JPG)

The ```\cmd_vel``` topic of ```\teleop``` node is remapped on ```\prov_cmd_vel```. In this way the velocity imposed by the user via keyboard isn't immediately imposed to the robot, but it's controlled by ```\final_UI``` node.

```\final_UI``` node is also connected to ```\gazebo``` and ```\move_base``` nodes.
It receives the robot's status by ```\move_base\feedback``` and publish the goal to reach on ```\move_base\goal```. This node also subscribe on ```\move_base\goal``` topic to have a goal feedback. It can cancel current goal using ```\move_base\cancel``` topic.

```\final_UI``` node also receives the laser scanner output on ```\scan``` topic by ```\gazebo``` node and sends to this node the robot velocity on ```\cmd_vel``` topic.

## Pseudocode
```\final_UI``` node behavior can be explained by this pseudocode:

```pseudocode
initialize node
initialize necessary publishers and subscribers

while(k != 0)
    print actual goal
    print menu
    k = command read from the user
    switch(k)
	    case '0':
	    break;
	    case '1':
            print a message to receive goal coordinates
	    	read goal coordinates from the user
            publish them on \move_base\goal topic
            take initial time
	    	break;
	    case '2':
	    	if(there is a goal set)
                cancel actual goal 
                print success message
            else
                print warning
	    case '3':
	    	print manual driving menu
            enable teleop twist keyboard
            while(input = 'x')
                read user input
                if(input = 'a')
                    enable/disable driving assistence
                if(input = 'x')
                    stop the robot
                    disable driving assistence
                    disable manual driving
	    default:
	    	printf("***INVALID COMMAND***");
	    	break;
	    	
	    }
```
When something is published on ```\move_base\feedback```:
```pseudocode
	if(current goal id from \move_base\feedback != id)
		update id
	if(there is a goal)
		if(the goal is reached)
			cancel goal
			print success message
		else
			if(timeout is reached)
				cancel the goal
				print a fail message
```
When something is published on ```\move_base\goal```, actual goal coordinates are saved.
When something is published on ```\prov_cmd_vel```:
```pseudocode
	if(manual driving mode is disabled)
		do nothing
	else if(driving assistence is disabled)
		publish the received velocity on ```\cmd_vel``` topic
	else 
		save velocity to be checked	
```
When something is published on ```\scan```:
```pseudocode
	if(driving assistence enabled)
		if(there is an obstacle in robot direction)
			stop the robot
			print a warning
		else
			publish the stored velocity on ```\cmd_vel``` topic
```
