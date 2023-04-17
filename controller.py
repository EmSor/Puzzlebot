#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
import numpy as np

def change_speed(linear_speed, angular_speed):
    twist = Twist()
    twist.linear.x = linear_speed
    twist.angular.z = angular_speed
    return twist

# Initialize the node
rospy.init_node('square_controller')

# Set the side length of the square
side_length = 2.0

# Ask the user to choose an option
print("Choose an option:")
print("1. Enter the desired linear speed")
print("2. Enter the desired time to complete the square path")
option = int(input("Enter the option number (1 or 2): "))

if option == 1:
    # Get the desired linear speed from the user
    linear_speed = float(input("Enter the desired linear speed (m/s): "))
    # Calculate the time for each segment based on the desired linear speed
    # Calculate the time for each segment based on the desired linear speed
    if(linear_speed == 1):
        segment_time = (side_length / linear_speed)
    elif(linear_speed < 3):
        segment_time = (side_length / linear_speed) + 1
    elif(linear_speed < 4):
        segment_time = (side_length / linear_speed) + 1.2
    else:
        segment_time = (side_length / linear_speed) + 1.5
    # Calculate the angular speed based on the desired linear speed and side length
    angular_speed = np.pi / 2 / segment_time
elif option == 2:
    # Get the desired time to complete the square path from the user
    total_time = float(input("Enter the desired time to complete the square path (s): "))
    # Calculate the time for each segment based on the total time
    segment_time = (total_time - 9) / 4
    # Calculate the linear speed based on the segment time
    linear_speed = side_length / segment_time
    # Calculate the angular speed based on the desired linear speed and side length
    angular_speed = np.pi / 2
else:
    print("Invalid option. Exiting.")
    exit()



# Set the time for each segment of the square as a rospy.Duration object
segment_time_duration = rospy.Duration(segment_time)

# Create the publisher for the Twist messages
pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

# Wait for the publisher to connect to the topic
rospy.sleep(1)

# Move the robot in a square pattern
for i in range(4):
    # Move forward for the duration of the segment
    move_forward = change_speed(linear_speed, 0)
    pub.publish(move_forward)
    rospy.sleep(segment_time_duration)

    # Stop the robot
    stop = change_speed(0, 0)
    pub.publish(stop)
    rospy.sleep(1.0)

    # Turn left for the duration of the segment
    turn_left = change_speed(0, angular_speed)
    pub.publish(turn_left) 
    if option == 1:
        rospy.sleep(segment_time_duration)
    else:
        rospy.sleep(1)



    # Stop the robot
    pub.publish(stop)
    # Stop the robot
    pub.publish(stop)
    rospy.sleep(1.0)

# Stop the robot before exiting
pub.publish(stop)
rospy.sleep(1)
