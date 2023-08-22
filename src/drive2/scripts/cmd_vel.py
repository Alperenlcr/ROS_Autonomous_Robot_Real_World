#!/usr/bin/python3.8
import rospy
from geometry_msgs.msg import Twist
#import drive

def cmd_vel_callback(data):
    linear_x = data.linear.x
    angular_z = data.angular.z
    print("Linear X:", linear_x, "Angular Z:", angular_z)

    direction = ''
    turn = ''
    speed = max([abs(linear_x), abs(angular_z)])*100+50
    if linear_x > 0.01:
        direction += 'forward'
    elif linear_x < -0.01:
        direction += 'backward'

    if angular_z > 0.02:
        turn += 'left'
    elif angular_z < -0.02:
        turn += 'right'

    if direction == '':
        direction += 'no'
    #drive.move(speed, direction, turn, 0.6)
    print(f'speed={speed}, direction={direction}, turn={turn}, radius={0.6}')


if __name__ == '__main__':
    # Spin until Ctrl+C is pressed
    try:
        # Initialize the ROS node
        rospy.init_node('cmd_vel_listener')

        # Subscribe to the joystick topic
        rospy.Subscriber('cmd_vel', Twist, cmd_vel_callback)
     #   drive.setup()
        while not rospy.is_shutdown():
            rospy.rostime.wallsleep(0.1)
     #   drive.destroy()
    except KeyboardInterrupt:
      #  drive.destroy()
      pass


