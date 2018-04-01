# turtlebot_visual_navigation
Version 2.0 for TurtleBot Project

-VERSION LINUX: Ubuntu 14.04 

#Installation:

	$ cd [link catkin_workspace]/src
	$ git clone https://github.com/Projet-M2-TurtleBot-UPS/turtlebot_visual_navigation.git


#Start:

	$ cd [link catkin_workspace]
	$ catkin_make

	#terminal 1
	$ roslaunch turtlebot_visual_navigation navigation.launch 	

	#terminal 2
	$ rosrun turtlebot_visual_navigation map_node
