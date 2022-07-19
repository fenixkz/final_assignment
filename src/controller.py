#! /usr/bin/env python3
"""
.. module:: controller_ui
    :platform: Unix
    :synopsis: A user interface of the robot, with implemented controller logic
.. moduleauthor:: Ayan Mazhitov <fenixkz7@gmail.com>

ROS node that implements a user interface in the terminal. The user interface is consisted of four choices:
 * Robot will autonomously reach the desired x and y coordinate
 * Manual control of the robot
 * Manual control of the robot with assistive mode, to avoid crashes
 * Exit the interface

Libraries:
    `NumPy <https://pypi.org/project/numpy/>`_: Array computing for python

    `rospy <http://wiki.ros.org/rospy>`_: ROS bridge for Python

    `time <https://docs.python.org/3/library/time.html>`_: Time-related functions

    `termios <https://docs.python.org/3/library/termios.html>`_: POSIX calls for tty I/O control

    `sys <https://docs.python.org/3/library/sys.html>`_: System-specific parameters and functions

    `tty <https://docs.python.org/3/library/tty.html>`_: Terminal control functions

Publishes to:
    ``/move_base/goal`` the goal position to move the robot

    ``/move_base/cancel`` if the cancel call is given to cancel the previously set goal position

    ``/cmd_vel`` the desired robot velocities

Retrieve from:
    ``/move_base/status`` id and status of the goal position

    ``/move_base/feedback`` x, y position of the robot
"""

# Libraries that we are going to use
import numpy as np
import rospy
import time
import os
from move_base_msgs.msg import MoveBaseActionFeedback
from move_base_msgs.msg import MoveBaseActionGoal
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from actionlib_msgs.msg import GoalID
from actionlib_msgs.msg import GoalStatusArray
import termios
import sys, tty
from std_msgs.msg import Float32

# All the publishers that are used for communication and control purposes
pub_movebase = rospy.Publisher("move_base/goal", MoveBaseActionGoal, queue_size = 50)
pub_cancel = rospy.Publisher("move_base/cancel", GoalID, queue_size = 50)
pub_vel = rospy.Publisher("cmd_vel", Twist, queue_size = 50)


def my_print():
    """ Function to simulate a UI to give the user a set of options

    Args:
        None

    Returns:
        None
    """
    os.system('clear')
    print("Greetings! This is a UI for the mobile robot simulation, there are several options available, please choose one of them: ")
    print("1. Robot will autonomously reach the desired x and y coordinate")
    print("2. Manual control of the robot")
    print("3. Manual control of the robot with assistive mode")
    print("4. Exit the interface")


def first_choice():
    """ Function to autonomously reach the desired x,y coordinate. Corresponds to the first choise of UI.

    Args:
        None

    Returns:
        int: **1** if done

    .. note::
        The completion of the goal position is checked via calculating the Euclidean distance between the goal and the
        actual position. The threshold is set to 0.3, but can be changed to be more accurate.
    """
    # Constructing our message to sent to the /move_base topic
    msg = MoveBaseActionGoal()
    msg.goal.target_pose.header.frame_id = "map"
    msg.goal.target_pose.pose.orientation.w = 1
    done = 0
    dist_thresh = 0.3
    while not done:
        os.system('clear')
        print("The first option has been chosen, now please insert x and y coordinate")
        x_goal = float(input("x coordinate: "))
        y_goal = float(input("y coordinate: "))
        print("The coordinates were given: x = %2.2f and y = %2.2f" % (x_goal, y_goal))
        msg.goal.target_pose.pose.position.x = x_goal
        msg.goal.target_pose.pose.position.y = y_goal
        # Move to the desired x and y
        pub_movebase.publish(msg)

        print("The robot starts moving")
        start = rospy.Time.now().to_sec()
        now = start
        dist = 100
        status = None
        while status is None:
            status = rospy.wait_for_message("move_base/status", GoalStatusArray)
        # Retrieve the id of the goal
        id = status.status_list[-1].goal_id.id
        # Retrieve the current status
        state = status.status_list[-1].status

        while dist > dist_thresh: # We are checking the distance to see if we reached the goal
            # Constantly check the status of the goal position
            status = rospy.wait_for_message("move_base/status", GoalStatusArray)
            state = status.status_list[-1].status
            # 1 means that the goal is active, others are that it is no longer active,
            # so if its no longer active we can exit the loop
            if (state != 1):
                break
            feedback = rospy.wait_for_message("move_base/feedback", MoveBaseActionFeedback)
            # Get the current x position of the robot
            x_now = feedback.feedback.base_position.pose.position.x
            # Get the current y position of the robot
            y_now = feedback.feedback.base_position.pose.position.y
            # Calculate the Euclidean distance
            dist = ((x_goal - x_now)**2 + (y_goal - y_now)**2)**0.5
            now = rospy.Time.now().to_sec()
            # Print some feedback for the user
            print("Robot's current location: x = %2.2f y = %2.2f" % (x_now, y_now))
            print("Time elapsed: %2.2f" %(now - start))
        # So, basically status can be 4 which means that there is no trajectory for the desired goal position
        # (Note) status can also be 2, meaning that it reached the position, but since we are calculating the distance, it does not have time to update it
        # So, the only case is when its 4, and we have to notify the user that the goal position is unreachable
        if state != 1:
            inp = input("The robot cannot move to the position you indicated, would you like to cancel the current goal? yn\n")
            if inp[0] == 'y':
                pass
            else:
                break

        # Successfully reached
        if dist < dist_thresh:
            print("Robot has achieved the goal position!")

        # Cancel the current goal, to set a new one if needed
        msg_cancel = GoalID()
        msg_cancel.id = id
        pub_cancel.publish(msg_cancel)
        inp = input("Do you want to enter new goal coordinates? yn\n")[0]
        if inp == 'y':
            continue
        else:
            done = 1
    return 1


def getch():
    """ Function to get the user input without having to press enter

        Args:
            None

        Returns:
            char: A char that was given by the user.
    """
    def _getch():
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch
    return _getch()


def manual_drive():
    """ Function that get the user input and transform it into appropriate velocities.
        For the second and third choice, to map the input to the appropriate output

        Args:
            None

        Returns:
            (tuple): tuple containing:
                * **x_vel** (double): speed along x-axis

                * **z_ang** (double): angular velocity around z-axis

                * **done** (bool): the user exitted the UI
    """
    key = getch()
    x_vel = 0
    z_ang = 0
    done = 0
    # Forward
    if key == 'w' or key == 'W':
        return 0.5, 0, 0
    # Turn left
    elif key == 'a' or key == 'A':
        return 0, 2, 0
    # Turn right
    elif key == 'd' or key == 'D':
        return 0, -2, 0
    # Backward
    elif key == 's' or key == 'S':
        return -0.5, 0, 0
    # Exit
    elif key == 'e' or key == 'E':
        return 0, 0, 1
    else:
        print("Unknown command\n")
    return x_vel, z_ang, done


def second_choice():
    """ Function to manually control the robot
        Args:
            None

        Returns:
            int: **1** if done

        .. note::
                The function accepts the control of WASD type. Either low or upper key. Also, to constantly move
                the user has to push the corresponding button continuously, not only press one time.
    """
    done = 0
    os.system('clear')
    print("Well choice, Vin Diesel. Now you can manually control the robot. The list of keys are:")
    print("W or w: to move forward")
    print("S or s: to move backward")
    print("A or a: to turn left")
    print("D or d: to turn right")
    print("E or e: to exit the manual control\n\n")
    # Set the message of type Twist() to send to the corresponding topic
    msg = Twist()
    msg.linear.y = 0
    msg.linear.z = 0
    msg.angular.x = 0
    msg.angular.y = 0
    # Infinite loop until the user decides to exit
    while not done:
        # Get the desired velocity from the input
        [msg.linear.x, msg.angular.z, done] = manual_drive()
        # Publish this velocity to the /cmd_vel topic
        pub_vel.publish(msg)
        time.sleep(0.2)
        msg.linear.x = 0
        msg.angular.z = 0
        # Reset the velocity, so its like in real life, the user has to constantly
        # push the button to move forward for example. Not only once
        pub_vel.publish(msg)
        # To keep it clean and accurate. These commands used to erase the last line, where user wrote his input
        CURSOR_UP = '\033[F'
        ERASE_LINE = '\033[K'
        print(CURSOR_UP + ERASE_LINE)
    return 1


def calculateMinDistance(data):
    """ Function to calculate the minimal distance
        from the robot to the obstacle, from the left side, right side and from frontal side

        Args:
            data (array of doubles): array of data from laser scan.

        Returns:
            (tuple): tuple containing:
                * **minDistanceLeft** (double): distance to the closest obstacle on the left

                * **minDistanceFront** (double): distance to the closest obstacle in front
                * **minDistanceRight** (double): distance to the closest obstacle on the right
    """
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

#
def third_choice():
    """ Function for manual control with assistance. It allows the user to drive the robot manually, but
        prevents it from crashing.

        Args:
            None

        Returns:
            int: **1** if done
    """
    done = 0
    # Variable to set the minimal safe distance
    safeDistance = 0.5
    os.system('clear')
    print("The third option has been chosen. Now you can manually control the robot, but the algorithm will help you not to crush into the walls. The list of keys are:")
    print("W or w: to move forward")
    print("S or s: to move backward")
    print("A or a: to turn left")
    print("D or d: to turn right")
    print("E or e: to exit the manual control\n\n")
    # Set the message of type Twist() to send to the corresponding topic
    msg = Twist()
    msg.linear.y = 0
    msg.linear.z = 0
    msg.angular.x = 0
    msg.angular.y = 0

    while not done:
        [msg.linear.x, msg.angular.z, done] = manual_drive()
        laser = rospy.wait_for_message("/scan", LaserScan)
        data = laser.ranges
        # So, now we are calculating the minimal distance and not allowing the user to go crush
        [minLeft, minFront, minRight] = calculateMinDistance(data)
        # If the obstacle is on the left, front, right then it blocks the user to turn/drive
        if minLeft < safeDistance:
            if msg.angular.z < 0:
                print("Sorry, but you cannot move to the right, because there is an obstacle\n")
                msg.angular.z = 0
        if minFront < safeDistance:
            if msg.linear.x > 0:
                print("Sorry, but you cannot move to the front, because there is an obstacle\n")
                msg.linear.x = 0
        if minRight < safeDistance:
            if msg.angular.z > 0:
                print("Sorry, but you cannot move to the left, because there is an obstacle\n")
                msg.angular.z = 0
        pub_vel.publish(msg)
        time.sleep(0.2)
        msg.linear.x = 0
        msg.angular.z = 0
        pub_vel.publish(msg)
        # To keep it clean and accurate. These commands used to erase the last line, where user wrote his input
        CURSOR_UP = '\033[F'
        ERASE_LINE = '\033[K'
        print(CURSOR_UP + ERASE_LINE)
    return 1

# This is the main function with the while loop to give
def main():
    """ Main function. Constantly waits for the user choice and then runs the corresponding
        function.

        Args:
            None

        Returns:
            None

        .. note::
                The choice is has to be an integer from 1 to 4
    """
    done = 0
    rospy.init_node('final_assignment')
    while not rospy.is_shutdown() and not done:
        my_print()
        while True:
            try:
                choice = int(input("Please insert your choice: "))
                break
            except:
                print("Please, only integers")
        if choice == 1:
            while not first_choice():
                pass
        if choice == 2:
            while not second_choice():
                pass
        if choice == 3:
            while not third_choice():
                pass
        elif choice == 4:
            done = 1
        else:
            print("Unknown command")
if __name__ == '__main__':
    main()
