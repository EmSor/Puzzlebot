#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
import numpy as np

# Inicializa el nodo
rospy.init_node('recorrido_cuadrado')

# Define los comandos de movimiento
move_forward = Twist()
move_forward.linear.x = 0.5
turn_left = Twist()
turn_left.angular.z = np.pi/4

# Calcula el tiempo de recorrido
tiempo_recorrido = 20.0

# Crea el publicador para enviar los comandos de movimiento
pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

# Espera a que el nodo de tiempo simulado este activo
while not rospy.Time.now():
    pass

# Publica los comandos de movimiento para recorrer el cuadrado
rospy.sleep(2)
while not rospy.is_shutdown():
    pub.publish(move_forward)
    rospy.sleep(4.0)
    pub.publish(turn_left)
    rospy.sleep(2.0)
# Detiene el robot antes de salir del nodo
pub.publish(Twist())
rospy.sleep(1.0)

