import numpy as np
import rospy
import termios
import sys, tty
from actionlib_msgs.msg import GoalStatusArray
from geometry_msgs.msg import Twist
import time
from nav_msgs.msg import Odometry
rospy.init_node("tst")

pub_vel = rospy.Publisher("cmd_vel", Twist, queue_size = 50)
def main_cb(msg):
    print("Abs")
    i = 0
    while True:
        print("1")
        time.sleep(1)
        i = i +1
        if i == 5:
            break
    time.sleep(0.5)

def getch():
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

sub_main = rospy.Subscriber("/odom", Odometry, main_cb)
rospy.spin()
