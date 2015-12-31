#depthimage_to_laserscan
Converts a depth image to a laser scan for use with navigation and localization.
This is a fork from https://github.com/ros-perception/depthimage_to_laserscan

ROS Wiki Page:
http://www.ros.org/wiki/depthimage_to_laserscan

##Environment
* Ubuntu 14.04
* ROS Indigo

##Build
* Copy the whole depthimage_to_laserscan folder to <CATKIN_WS>/src
* `cd <CATKIN_WS> && catkin_make`

##Usage
###Launch the node

`source <CATKIN_WS>/devel/setup.bash

rosrun depthimage_to_laserscan depthimage_to_laserscan`

###Output topics:
* /scan

###Input topics:
* /camera/depth/compressed
* /camera/camera_info
* Topics above will be subscribed only if /scan is subscribed.

###Configurations
* Configurations are described in cfg/Depth.cfg

##Visualization
`rosrun tf static_transform_publisher 0 0 0 0 0 0 map <laser_frame> 50`

`rosrun rviz rviz`
