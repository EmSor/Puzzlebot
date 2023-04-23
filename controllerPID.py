#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
import numpy as np
from std_msgs.msg import Float32

class RobotController:

    def __init__(self):
        self.wr_value = 0.0
        self.wl_value = 0.0

    def wr_callback(self, msg):
        self.wr_value = msg.data

    def wl_callback(self, msg):
        self.wl_value = msg.data

robot_controller = RobotController()

x1 = rospy.get_param('/x1', 2)
y1 = rospy.get_param('/y1', 1)
x2 = rospy.get_param('/x2', 2)
y2 = rospy.get_param('/y2', 2)
x3 = rospy.get_param('/x3', 0)
y3 = rospy.get_param('/y3', 2)
x4 = rospy.get_param('/x4', 0)
y4 = rospy.get_param('/y4', 0)

# Create a NumPy array of the values
points = np.array([[x1, y1], [x2, y2], [x3, y3], [x4, y4]])

#Controllador de v
kp_v = rospy.get_param('/kp_v', 0.25)
ki_v = rospy.get_param('/ki_v', 0.0)
kd_v = rospy.get_param('/kd_v', 0.0)

#Controlador de w
kp_w = rospy.get_param('/kp_w', .37)
ki_w = rospy.get_param('/ki_w', 0.0)
kd_w = rospy.get_param('/kd_w', 0.0)

# Initialize the ROS node
rospy.init_node('Controller_node')

# Subscribe to the /wr & /wl topics with the callback functions
rospy.Subscriber('/wr', Float32, robot_controller.wr_callback)
rospy.Subscriber('/wl', Float32, robot_controller.wl_callback)


rate = rospy.Rate(10)

# Hacemos una variable para poder mandarle los datos al robot
pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

x = 0.0
y = 0.0
theta = 0.0
robot = Twist()
angular_speed = np.pi / 8
target_angle = 0

rospy.sleep(.5)

vel  = 2
while not rospy.is_shutdown():
    # Iterate through the target points
    for i in range(4):
        pub.publish(Twist())
        rospy.sleep(1)
        
        x_ref = points[i][0]
        y_ref = points[i][1]


        e_dist = 0.1
        e_theta = 0.1
        prevTime = rospy.get_time()
        sm_error_v = 0
        sm_error_w = 0
        error_a_v = 0
        error_a_w = 0

        # Turn the robot to face the target point
        dx = x_ref - x
        dy = y_ref - y
        target_angle = np.arctan2(dy, dx)
        if target_angle > np.pi:
            target_angle -= 2 * np.pi
        elif target_angle < -np.pi:
            target_angle += 2 * np.pi

        e_theta = target_angle - theta
        while abs(e_theta) >= 0.05:  # Change this threshold as needed
            wr_value = robot_controller.wr_value
            wl_value = robot_controller.wl_value
            currentTime = rospy.get_time()

            
            v = .05 * ((wr_value + wl_value)/2)
            #Calculamos la velocidad angular
            w = .05 * ((wr_value - wl_value)/.19)

            #Calculamos nuestro dt
            dt = currentTime - prevTime

            #Calculamos distancia en x, y y su angulo
            #Tengo la posicion y angulo del robot actual
            x = x + v * dt * np.cos(theta)
            y = y + v * dt * np.sin(theta)

            #rospy.loginfo('My valor de x es: %f, my valor de y es: %f, el angulo es: %f, mi refrenecia es (%f,%f)', x, y, theta, x_ref, y_ref)
            theta = theta + w * dt
            
            dx = x_ref - x
            dy = y_ref - y
            target_angle = np.arctan2(dy, dx)
            if target_angle > np.pi:
                target_angle -= 2 * np.pi
            elif target_angle < -np.pi:
                target_angle += 2 * np.pi

            # Calculate the relative angle to turn
            e_theta = target_angle - theta

            ang = e_theta * kp_w
            robot.linear.x = 0
            robot.angular.z = ang
            pub.publish(robot)

            prevTime = currentTime

        while abs(e_dist) >= 0.05:
            # Subscribe to the /wr & /wl topic with a callback function to handle incoming messages
            wr_value = robot_controller.wr_value
            wl_value = robot_controller.wl_value
            currentTime = rospy.get_time()

            #Calculamos la velocidad
            v = .05 * ((wr_value + wl_value)/2)

            #Calculamos la velocidad angular
            w = .05 * ((wr_value - wl_value)/.19)

            #Calculamos nuestro dt
            dt = currentTime - prevTime

            #Calculamos distancia en x, y y su angulo
            #Tengo la posicion y angulo del robot actual
            x = x + v * dt * np.cos(theta)
            y = y + v * dt * np.sin(theta)

            #rospy.loginfo('My valor de x es: %f, my valor de y es: %f, el angulo es: %f, mi refrenecia es (%f,%f)', x, y, theta, x_ref, y_ref)
            theta = theta + w * dt
            
            dx = x_ref - x
            dy = y_ref - y
            target_angle = np.arctan2(dy, dx)
            if target_angle > np.pi:
                target_angle -= 2 * np.pi
            elif target_angle < -np.pi:
                target_angle += 2 * np.pi
            
            # Calculate the relative angle to turn

            # Calculate the relative angle to turn
            e_theta = target_angle - theta
            e_dist = np.sqrt((dx)**2+(dy)**2)
            # sm_error_v += e_dist * dt
            # sm_error_w += e_theta * dt

            # df_error_v = (e_dist - error_a_v)/dt 
            # df_error_w = (e_dist - error_a_w)/dt 


            vel = e_dist * kp_v# + sm_error_v * ki_v + df_error_v * kd_v
            ang = e_theta * kp_w# + sm_error_w * ki_w + df_error_w * kd_w
            #rospy.loginfo('My valor de dx es: %f, my valor de dy es: %f, el error_d es: %f, necesito un angulo de: %f', dx, dy, e_dist, target_angle)

            robot.linear.x = vel
            robot.angular.z = ang
            pub.publish(robot)
            prevTime = currentTime
            error_a_v = e_dist
            error_a_w = e_theta
