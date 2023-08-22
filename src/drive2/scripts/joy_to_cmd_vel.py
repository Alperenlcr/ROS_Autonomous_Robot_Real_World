#!/usr/bin/python3.8

import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

def publish_cmd_vel(linear_x, angular_z):
    twist_msg = Twist()
    twist_msg.linear.x = linear_x
    twist_msg.angular.z = angular_z

    pub.publish(twist_msg)
    print(twist_msg)


def joy_callback(data):
    # Print the list of buttons
    global speed
    if data.buttons[5] == 1:
        speed = 0
    if data.buttons[7] == 1:
        if speed <= 0.9:
            speed += 0.1
    if data.buttons[6] == 1:
        if speed >= -0.9:
            speed -= 0.1

    angular = 0
    if data.buttons[1] == 1:
        angular -= 0.8
    elif data.buttons[3] == 1:
        angular += 0.8

    publish_cmd_vel(speed, angular)


if __name__ == '__main__':
    # Spin until Ctrl+C is pressed
    speed = 0
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    # Initialize the ROS node
    rospy.init_node('joy_to_cmd_vel')

    # Subscribe to the joystick topic
    rospy.Subscriber('joy', Joy, joy_callback)
    while not rospy.is_shutdown():
        rospy.sleep(0.1)
