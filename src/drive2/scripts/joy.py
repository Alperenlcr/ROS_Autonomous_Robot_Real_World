#!/usr/bin/python3.8

import rospy
from sensor_msgs.msg import Joy
import drive


def joy_callback(data):
    # Print the list of buttons
    # print(data.buttons)
    drive.move(100, "forward" if data.buttons[0] == 1 else "backward" if data.buttons[2] == 1 else "no", \
         "right" if data.buttons[1] == 1 else "left" if data.buttons[3] == 1 else "no", 0.6)
    print(f'speed={100}, direction={"forward" if data.buttons[0] == 1 else "backward" if data.buttons[2] == 1 else "no"}, \
 turn={"right" if data.buttons[1] == 1 else "left" if data.buttons[3] == 1 else "no"}, radius={0.6}')


if __name__ == '__main__':
    # Spin until Ctrl+C is pressed
    try:
        # Initialize the ROS node
        rospy.init_node('joy_listener')

        # Subscribe to the joystick topic
        rospy.Subscriber('joy', Joy, joy_callback)
        drive.setup()
        while not rospy.is_shutdown():
            rospy.rostime.wallsleep(0.1)
        drive.destroy()
    except KeyboardInterrupt:
        drive.destroy()
