#!/usr/bin/env python
# Autor> Ruben Claveria Vega
# Curso> EL5206 Laboratorio de Inteligencia Computacional y Robotica

# En lugar de 'testpy', deberia ir el nombre del paquete creado.
import roslib; roslib.load_manifest('clay_robot')
import robot_utilities
import time
import rospy
import math
import numpy as np

from sensor_msgs.msg import LaserScan 
from nav_msgs.msg import Odometry     
from geometry_msgs.msg import Twist
from std_msgs.msg import String 

import numpy.matlib
import matplotlib.pyplot as plt
from matplotlib.path import Path
import matplotlib.patches as patches

import sys

class Controller:

	x = []
	y = []
    	measFile = open('/home/claydergc/catkin_ws/src/clay_robot/scripts/measurement.dat', 'w+')
	deadReckFile = open('/home/claydergc/catkin_ws/src/clay_robot/scripts/deadReckoning.dat', 'w+')

	def __init__(self):

		# inicializa nodo ROS
		rospy.init_node('Simulation');

		# robotID = ID del robot que se desea controlar
		# si hay solo un robot, robotID debe ser ""
		# si hay mas de un robot, robotID debe ser de la forma "/robot_0", "/robot_1", etc.
		robotID = ""

		# Posicion y orientacion iniciales.
		# IMPORTANTE: deben ser consistentes con la del archivo .world
		init_x = 0.0
		init_y = 0.0
		#init_angle = 90.0
		init_angle = 0

		# creacion de un objeto de tipo Robot
		R = robot_utilities.Robot(robotID, init_x, init_y, init_angle)

		# Subscripcion a los topicos de interes: odometria (Odometry) y sensor de distancia (LaserScan)
		rospy.Subscriber(robotID+'/odom', Odometry, R.odom_callback)
		#rospy.Subscriber(robotID+'/base_scan', LaserScan, R.ranger_callback)
		rospy.Subscriber(robotID+'p3dx/laser/scan', LaserScan, R.ranger_callback)
		# Observacion: Las funciones de callback se ejecutan cada vez que el programa recibe informacion 
		# desde un topico. Se utilizan, generalmente, para actualizar datos.
		# Pueden utilizarse funciones de callback programadas por el alumno, 
		# diferentes a las provistas por la clase Robot

		# Se recomienda que este ciclo vaya antes de todas las instrucciones de control
		# Sirve para asegurar que el Robot recibio su primera lectura de sensor
		while not rospy.is_shutdown() and len(R.distances)==0:
			R.rate.sleep()

		self.measFile.write('#segundos nanosegundos Rango theta Radio\n')
		self.deadReckFile.write('#segundos nansegundos X Y Theta\n')

		# Ciclo de ejemplo. 
		# Si se va a controlar el robot con una funcion de robot_utilities en vez de con el ciclo:
		# opcion 1: cambiar True por False en la condicion del while
		# opcion 2: comentar el bloque de codigo
		#while not rospy.is_shutdown() and True:
		#while not rospy.is_shutdown() and R.pose_x < 3:
		while not rospy.is_shutdown() and R.pose_x < 2:
			# Define velocidad lineal [metros/segundo] del robot durante la iteracion
			R.cmd.linear.x = 0.2
			
			# Velocidad angular [radianes/segundo]; rango aceptable: [-pi/2, pi/2 ], aprox. [-1.57, 1.57]
			#R.cmd.angular.z = 0.0
			R.cmd.angular.z = 0.03

			# indica a Stage la velocidad que el robot tendra durante un ciclo
			R.cmd_pub.publish(R.cmd)
			
			# Ejemplo de como rescatar datos captados por los sensores
			# En este caso solo se imprimen, pero la idea es utilizarlos en decisiones de control
			print "_______________________"
			print 'Primera lectura del laser [m]: '+ str(R.distances[0])
			print 'Posicion robot [m]: x = ' + str(R.pose_x)+'; y = ' +str(R.pose_y)
			print 'Orientacion robot [grados]: ' + str(R.angle)
			R.show_distance()

			self.deadReckFile.write( str(int(rospy.Time.now().to_sec())) + ' ' + str(rospy.Time.now().to_nsec()) + ' ' +  str(R.pose_x) + ' ' + str(R.pose_y) + ' ' + str(R.angle*math.pi/180) + '\n' )

			for i in range(0, len(R.distances)):
				if np.isfinite(R.distances[i]):
					#print rospy.Time.now().to_sec(), ' ', rospy.Time.now().to_nsec(), ' ',  R.distances[i], ' ', ( (R.angle*math.pi/180) + R.laser_angles[i] )
					self.measFile.write( str(int(rospy.Time.now().to_sec())) + ' ' + str(rospy.Time.now().to_nsec()) + ' ' +  str(R.distances[i]) + ' ' + str( ((R.angle*math.pi/180) + R.laser_angles[i]) ) + '\n')

			# Mantiene la regularidad temporal (de acuerdo al rate definido en robot_utilities.Robot)
			R.rate.sleep()

		R.stop()
		self.measFile.close()
		self.deadReckFile.close()

		#for i in range(0, len(R.distances)):
			#print R.distances[i]
		#	if np.isfinite(R.distances[i]):
		#		self.x.append(R.pose_x + R.distances[i] * math.cos( (R.angle*math.pi/180) + R.laser_angles[i] ) )
		#		self.y.append(R.pose_y + R.distances[i] * math.sin( (R.angle*math.pi/180) + R.laser_angles[i] ) )

		#print "_______________________"
		#print 'Primera lectura del laser [m]: '+ str(R.distances[0])
		#print 'Numero de puntos: '+ str(len(R.distances))
		
		#plt.plot(self.x, self.y,'r.', R.pose_x, R.pose_y, 'b>')
		#plt.show()

		# Funciones de ejemplo (uncomment para usar): 
		#R.nSteps(500,500)
		#R.nSteps(100,500)
		#R.nSteps(1500,900)
		#R.nSteps(900,1500)
		#R.moveTill(math.pi*5.0/2, 0.5, 0.1)


        
if __name__ == "__main__": Controller()
