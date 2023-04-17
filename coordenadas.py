#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
import numpy as np

def change_speed(linear_speed, angular_speed):
    twist = Twist()
    twist.linear.x = linear_speed
    twist.angular.z = angular_speed
    return twist

def move_to_coordinate(curr_x, curr_y, curr_theta, next_x, next_y, linear_speed, angular_speed):
    # Calculate the distance and angle between the current and next positions
    dx = next_x - curr_x
    dy = next_y - curr_y
    distance = np.sqrt(dx**2 + dy**2)
    target_angle = np.arctan2(dy, dx)
    
    # Calculate the relative angle to turn
    relative_angle = target_angle - curr_theta
    if relative_angle > np.pi:
        relative_angle -= 2 * np.pi
    elif relative_angle < -np.pi:
        relative_angle += 2 * np.pi

    # Calculate time to reach the destination
    time_to_move = distance / linear_speed + 1.2
    time_to_turn = np.abs(relative_angle) / angular_speed

    # Turn towards the destination
    turn = change_speed(0, np.sign(relative_angle) * angular_speed)
    pub.publish(turn)
    rospy.sleep(time_to_turn)

    # Stop before moving forward
    stop = change_speed(0, 0)
    pub.publish(stop)
    rospy.sleep(1.0)

    # Move forward to the destination
    move_forward = change_speed(linear_speed, 0)
    pub.publish(move_forward)
    rospy.sleep(time_to_move)

    # Stop at the destination
    pub.publish(stop)
    rospy.sleep(1.0)

    # Update the current orientation
    curr_theta += relative_angle
    if curr_theta > np.pi:
        curr_theta -= 2 * np.pi
    elif curr_theta < -np.pi:
        curr_theta += 2 * np.pi

    return next_x, next_y, curr_theta

# Initialize the node
rospy.init_node('path_controller')

# Get the desired linear speed from the user
linear_speed = float(input("Enter the desired linear speed (m/s): "))

# Set the angular speed
angular_speed = np.pi / 8

# Create the publisher for the Twist messages
pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

# Wait for the publisher to connect to the topic
rospy.sleep(1)

# Get the coordinates from the user
coordinates = []
for i in range(3):
    x = float(input("Enter the x coordinate of point: "))
    y = float(input("Enter the y coordinate of point: "))
    coordinates.append((x, y))

# Set the reference point (initial position) as (0, 0)
reference_point = (0, 0)

# Move the robot to each coordinate
curr_x, curr_y, curr_theta = reference_point[0], reference_point[1], 0
for next_x, next_y in coordinates:
    curr_x, curr_y, curr_theta = move_to_coordinate(curr_x, curr_y, curr_theta, next_x, next_y, linear_speed, angular_speed)

# Stop the robot before exiting
stop = change_speed(0, 0)
pub.publish(stop)
rospy.sleep(1)
