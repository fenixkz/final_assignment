# Research Track Final Assignment
## Problem statement
This project presents a user interface to control the robot inside the Gazebo simulation engine. Several options are given to the user:

1. Autonomously reach a x,y coordinate inserted by the user

2. Let the user drive the robot with the keyboard

3. Let the user drive the robot assisting them to avoid collisions 

The interface is implemented within the terminal, so user have to input the desired choice to the terminal. 

## Requirements
The project is done on Ubuntu 16.04 and ROS Kinetic. 
This project is dependent on several libraries that need to be installed on your local machine. This can be done by executing the following commands:

'
$ git clone https://github.com/CarmineD8/slam_gmapping.git
$ suo apt-get install ros-<your_ros_distro>-navigation
'


## Installing 
In order to be able to use that project, first you need to have a catkin workspace, then git clone this repository into your workspsace, and execute the following commands:

`
$ catkin_make
`
