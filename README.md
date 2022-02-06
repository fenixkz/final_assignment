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

`
$ git clone https://github.com/CarmineD8/slam_gmapping.git
`

`
$ sudo apt-get install ros-<your_ros_distro>-navigation
`


## Installing 
In order to be able to use that project, first you need to have a catkin workspace, then git clone this repository into your workspsace, and execute the following commands:

`
$ catkin_make
`

## Running
There are two main processes that need to be launched, each in different terminals:
```
$ roslaunch final_assignment combined.launch 
$ rosrun final_assignment controller.py 
```

## Implementation details
The first option is done by getting the user x,y desired coordinates and publishing this coordinated to /move_base/goal topic. Then, the distance is calculated between the current position of the robot and the desired position. If the distance is less than 0.3, then we can conclude that the robot achieved the goal position.

There are cases, when the robot cannot achieve the desired position. In that cases, we constantly read the status of the goal from /move_base/status topic. If the status is 4, it means that the trajectory cannot be found, and the UI notifies the user. Giving an option to cancel the current goal and inputting the new one.

The second option is done trivially, the UI constantly reads the input from the user and then sends the corresponding commands to /cmd_vel topic.

The third option is similar to the second. The only difference is when the user tries to go near the obstacle, the algorithm does not allow it. This is done by calculating the minimal distance to the obstacle from three sides: left, front, and right. The data comes from laser scan. The robot is equipped with laser scanner that gives a measurement of the distance to the closest obstacle in the range of approximately [-90°; 90°]. This interval is sampled with approximately 0.25° step. The algorithm is constantly reads the data from /scan topic and calculates the minimal distance and compares it with the safeDistance.

The frontal side is considered the value of the angles in the range: [-45°; 45°]

The left side is considered the value of the angles in the range: [-90°; -45°]

The right side is considered the value of the angles in the range: [45°; 90°]

![text](https://github.com/fenixkz/rt_assigment2/blob/master/images/RT_3.png)
