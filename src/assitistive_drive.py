#! /usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
import time
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry
def calculateMinDistance(data):
    minDistanceLeft = 30
    minDistanceFront = 30
    minDistanceRight = 30
    for i in range(0, 4*45):
        if data[i] < minDistanceLeft:
            minDistanceLeft = data[i]
    for i in range(4*45, 4*135):
        if data[i] < minDistanceFront:
            minDistanceFront = data[i]
    for i in range(4*135, len(data)):
        if data[i] < minDistanceRight:
            minDistanceRight = data[i]
    return minDistanceLeft, minDistanceFront, minDistanceRight

def assist_cb(msg):
    global assistive
    assistive = msg.data

def tmp_cb():
    

def laser_cb(msg):
    global pub_vel, assistive, msg_vel
    data = msg.ranges
    safeDistance = 0.5
    if assistive:
        [minLeft, minFront, minRight] = calculateMinDistance(data)
        if minLeft < safeDistance:
            # print("Sorry, but you cannot move to the left, because there is an obstacle\n")
            if msg_vel.angular.z > 0:
                msg_vel.angular.z = 0
        if minFront < safeDistance:
            # print("Sorry, but you cannot move to the front, because there is an obstacle\n")
            if msg_vel.linear.x > 0:
                msg_vel.linear.x = 0
        if minRight < safeDistance:
            # print("Sorry, but you cannot move to the right, because there is an obstacle\n")
            if msg_vel.angular.z < 0:
                msg_vel.angular.z = 0
        # print("Left: %2.2f, Front: %2.2f, Right: %2.2f" %(minLeft, minFront, minRight))


rospy.init_node("third_choice")
sub_laser = rospy.Subscriber("/scan", LaserScan, laser_cb)
pub_vel = rospy.Publisher("/cmd_vel", Twist, queue_size = 50)
sub_assist = rospy.Subscriber("/third_choice", Float32, assist_cb)
sub_tmp = rospy.Subsriber("/odom", Odometry, tmp_cb)
assistive = 0
rospy.spin()
