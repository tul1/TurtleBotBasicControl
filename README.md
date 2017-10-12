# TurtleBotBasicControl
A simple ROS node that makes Gazebo's Turtlebot follow a polygon path.

	     _________     __
	    /_|__|__|_\   / -\
	   /__|__|__|__\  \__< 
	o_/___|__|__|___\_|
	   ())      ())

Version 1.0

## Dependencies

* ROS: kinect version 
* catkin_tools: version 0.4.4 

## How to build the project
The followings are de commands to build turtlebot_driver's project. 

	#Change directory to the catkin workspace	
	$cd catkin_ws_turtleBotBasicControl
	
	#Building the project
	$catkin_make
	
	#sourcing the project's environment 
	$source ./devel/setup.bash


## How to build the project
The followings are de commands to run turtlebot_driver's node.

	#launch Gazebo's turtlebot simulator
	$roslaunch turtlebot_gazebo turtlebot_world.launch


