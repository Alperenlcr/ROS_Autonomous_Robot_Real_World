#!/usr/bin/python3.8

import rospy
from geometry_msgs.msg import Twist
from time import sleep
import signal
import sys
def publish_cmd_vel(linear_x, angular_z, linear_y):
    twist_msg = Twist()
    twist_msg.linear.x = linear_x
    twist_msg.linear.y = linear_y
    twist_msg.angular.z = angular_z

    pub.publish(twist_msg)
    print(twist_msg)

def signal_handler(sig, frame):
    sys.exit(0)

if __name__ == '__main__':
    rospy.init_node('cmd_vel_publisher')
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    signal.signal(signal.SIGINT, signal_handler)
    try:
        while True:
            # Forward
            publish_cmd_vel(0.135, 0.0, 0.0)
            sleep(1)
            # Backward
            publish_cmd_vel(-0.651, 0.0, 0.0)
            sleep(1)
            # Left
            publish_cmd_vel(0.0, 0.21, 0.0)
            sleep(1)
            # Right
            publish_cmd_vel(0.0, -0.41, 0.0)
            sleep(1)
            # Forward and Left
            publish_cmd_vel(0.51, 0.21, 0.0)
            sleep(1)
            # Forward and Right
            publish_cmd_vel(0.91, -0.1, 0.0)
            sleep(1)
            # Backward and Left
            publish_cmd_vel(-0.81, 0.31, 0.0)
            sleep(1)
            # Backward and Right
            publish_cmd_vel(-0.51, -0.51, 0.0)
            sleep(1)
    except rospy.ROSInterruptException:
        pass
