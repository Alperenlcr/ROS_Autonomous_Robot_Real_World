#!/usr/bin/python3.8
import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry

def pose_stamped_callback(pose_stamped_msg):
    # Convert PoseStamped to Odometry
    odom_msg = pose_stamped_to_odom(pose_stamped_msg)

    # Publish the converted Odometry message
    odom_pub.publish(odom_msg)

def pose_stamped_to_odom(pose_stamped_msg):
    odom_msg = Odometry()

    # Copy the header information from the PoseStamped message to the Odometry message
    odom_msg.header = pose_stamped_msg.header

    # Set the child frame ID
    odom_msg.child_frame_id = "base_link"

    # Copy the pose information from the PoseStamped message to the Odometry message
    odom_msg.pose.pose = pose_stamped_msg.pose

    # Set the twist information (optional)
    # odom_msg.twist.twist = ???  # Provide the twist information if available

    return odom_msg

# Example usage
if __name__ == '__main__':
    rospy.init_node('pose_to_odom_converter')

    # Create a subscriber to the PoseStamped topic
    rospy.Subscriber('/slam_out_pose', PoseStamped, pose_stamped_callback)

    # Create a publisher for the Odometry message
    odom_pub = rospy.Publisher('odom', Odometry, queue_size=10)

    rospy.spin()
