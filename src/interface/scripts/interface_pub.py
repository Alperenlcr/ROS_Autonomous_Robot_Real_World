#!/usr/bin/python3.8

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from nav_msgs.msg import OccupancyGrid, Odometry
import numpy as np


def combine_images(image1, image2):
    # Get the dimensions of both images
    height1, width1, _ = image1.shape
    height2, width2, _ = image2.shape
    
    # Resize the first image to match the size of the second image
    resized_image1 = cv2.resize(image1, (width2, height2))
    
    # Combine the two images horizontally
    combined_image = cv2.hconcat([resized_image1, image2])
    
    return combined_image


def image_proccessing(frame):
    hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV) #HSV renk uzayı, renkleri daha kolay algılamamıza olanak tanır.

    lower_purple = np.array([130, 50, 50])
    upper_purple = np.array([160, 255, 255])

    purple_mask = cv2.inRange(hsv_frame, lower_purple, upper_purple)

    contours, _ = cv2.findContours(purple_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    for contour in contours:
        area = cv2.contourArea(contour)
        if area > 1000:
            x, y, w, h = cv2.boundingRect(contour)
            frame = cv2.rectangle(frame, (x, y), (x+w, y+h), (128, 0, 128), 3)
            cv2.putText(frame, 'Enemy', (x, y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (128, 0, 128), 2)
    # Sarı renge göre dost algılama
    lower_yellow = np.array([20, 100, 100])
    upper_yellow = np.array([40, 255, 255])

    yellow_mask = cv2.inRange(hsv_frame, lower_yellow, upper_yellow)

    contours, _ = cv2.findContours(yellow_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    for contour in contours:
        area = cv2.contourArea(contour)
        if area > 1000:
            x, y, w, h = cv2.boundingRect(contour)
            frame = cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 255), 3)
            cv2.putText(frame, 'Friend', (x, y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 255), 2)

    return frame


def process_map(occupancy_map, odom):
    # Extract information from the occupancy map message
    map_width = occupancy_map.info.width
    map_height = occupancy_map.info.height
    map_resolution = occupancy_map.info.resolution
    map_data = occupancy_map.data
    
    # Convert the occupancy map data to a NumPy array
    map_array = np.array(map_data, dtype=np.int8).reshape((map_height, map_width))
    
    map_array_scaled = map_array.astype(dtype=np.uint8)
    
    # Create an empty image with the same shape as the occupancy map array
    map_image = np.zeros((map_height, map_width, 3), dtype=np.uint8)
    
    # Assign colors to the empty spaces and occupied spaces
    map_image[map_array_scaled == 0] = (255, 255, 255)  # White for empty spaces
    map_image[map_array_scaled == 100] = (0, 0, 255)  # Blue for occupied spaces
    
    # Extract the robot's position from the odometry message
    robot_x = odom.pose.pose.position.x
    robot_y = odom.pose.pose.position.y
    
    # Scale the robot's position to match the map resolution
    map_robot_x = int(robot_x / map_resolution) + map_width // 2
    map_robot_y = int(robot_y / map_resolution) + map_height // 2
    
    # Draw a circle to represent the robot's position on the image
    cv2.circle(map_image, (map_robot_x, map_robot_y), radius=5, color=(255, 0, 0), thickness=-1)
    
    return map_image


def main():
    # Initialize the ROS node
    rospy.init_node('interface_publisher', anonymous=True)
    
    # Create a publisher for the image_raw topic
    image_pub = rospy.Publisher('info_combined', Image, queue_size=10)
    
    # Initialize the OpenCV capture object for the camera
    cap = cv2.VideoCapture(0)  # Assuming the camera index is 0
    
    # Initialize the CvBridge object
    bridge = CvBridge()
    
    # Set the publishing rate (e.g., 10 Hz)
    rate = rospy.Rate(10)
    
    while not rospy.is_shutdown():
        # Capture frame from the camera
        ret, frame = cap.read()
        
        if ret:
            detected_image = image_proccessing(frame)

            # Wait for the occupancy map and odom message to be published
            occupancy_map = rospy.wait_for_message('map', OccupancyGrid)
            odom = rospy.wait_for_message('odom', Odometry)
   
            map_image = process_map(occupancy_map, odom)
            
            combined = combine_images(detected_image, map_image)
            # Convert the OpenCV image to ROS image message
            ros_image = bridge.cv2_to_imgmsg(combined, "bgr8")
            # Publish the ROS image message
            image_pub.publish(ros_image)
        
        # Sleep to maintain the desired publishing rate
        rate.sleep()
    
    # Release the camera capture and close the OpenCV windows
    cap.release()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
