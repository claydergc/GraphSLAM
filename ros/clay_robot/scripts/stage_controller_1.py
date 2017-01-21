#!/usr/bin/env python

import roslib; roslib.load_manifest('clay_robot')
import robot_utilities
import time
import rospy
import math
import numpy as np

from sensor_msgs.msg import LaserScan 
from nav_msgs.msg import Odometry
from circle2d_feature_extractor.msg import Circle2DDetections
import sensor_msgs.point_cloud2 as pc2
from geometry_msgs.msg import Twist
from std_msgs.msg import String 

import numpy.matlib
import matplotlib.pyplot as plt
from matplotlib.path import Path
import matplotlib.patches as patches

import sys
import getch

from collections import namedtuple

Point2D = namedtuple("Point2D", "x y")

class Controller:

	measFile = open('/home/claydergc/catkin_ws/src/clay_robot/scripts/measurement.dat', 'w+')
	deadReckFile = open('/home/claydergc/catkin_ws/src/clay_robot/scripts/deadReckoning.dat', 'w+')

	#featureX = []
	#featureY = []
	landMarks = []

	def keyControl(self, robot):

		x = []
		y = []

		KEY_UP = 65
		KEY_DOWN = 66
		KEY_RIGHT = 67
		KEY_LEFT = 68
		USER_QUIT = 100

		#MAX_FORWARD = 1.1
		MAX_FORWARD = 0.5
		MAX_LEFT = 0.3
		#MIN_FORWARD = -1.1
		MIN_FORWARD = -0.5
		MIN_LEFT = -0.3

		forward = 0.0
		left = 0.0
		keyPress = 0

		t0 = int(rospy.Time.now().to_sec())

		while(keyPress != USER_QUIT):
			keyPress = getch.getArrow()

			if((keyPress == KEY_UP) and (forward <= MAX_FORWARD)):
				forward += 0.1
				#forward = 0.5
			elif((keyPress == KEY_DOWN) and (forward >= MIN_FORWARD)):
				forward -= 0.1
				#forward = -0.5
			#elif((keyPress == KEY_LEFT) and (left <= MAX_LEFT)):
			elif((keyPress == KEY_LEFT) and (left >= MIN_LEFT)):				
				#left += 0.1
				left -= 0.1
				#left = 0.2
			#elif((keyPress == KEY_RIGHT) and (left >= MIN_LEFT)):
			elif((keyPress == KEY_RIGHT) and (left <= MAX_LEFT)):				
				#left -= 0.1
				left += 0.1
				#left = -0.2

			robot.cmd.linear.x = forward
			robot.cmd.angular.z = left
			robot.cmd_pub.publish(robot.cmd)
			robot.rate.sleep()

			tActual = int(rospy.Time.now().to_sec())


			if(tActual!=t0):
				self.deadReckFile.write( str(int(rospy.Time.now().to_sec())) + ' ' + str(rospy.Time.now().to_nsec()) + ' ' + 
						 str(robot.pose_x) + ' ' + str(robot.pose_y) + ' ' + str(robot.angle*math.pi/180) + '\n' )

				posxAnt = 99
				posyAnt = 99

				for i in range(0, len(robot.distances)):
					if np.isfinite(robot.distances[i]):
						#self.measFile.write( str(int(rospy.Time.now().to_sec())) + ' ' + str(rospy.Time.now().to_nsec()) + 
						#					 ' ' +  str(robot.distances[i]) + ' ' + str( ((robot.angle*math.pi/180) + robot.laser_angles[i]) ) + '\n')
						ang = ((robot.angle*math.pi/180.0) + robot.laser_angles[i])
						posx = (robot.pose_x + (robot.distances[i]*math.cos(ang)))
						posy = (robot.pose_y + (robot.distances[i]*math.sin(ang)))
						#posx = (robot.distances[i]*math.cos(ang))
						#posy = (robot.distances[i]*math.sin(ang))
						dis = math.sqrt( posx*posx + posy*posy )
						#self.measFile.write( str(int(rospy.Time.now().to_sec())) + ' ' + str(rospy.Time.now().to_nsec()) + 
						#					 ' ' +  str(dis) + ' ' + str(ang) + '\n')
						if abs(posxAnt-posx)>1 and abs(posyAnt-posy)>1:

							self.measFile.write( str(int(rospy.Time.now().to_sec())) + ' ' + str(rospy.Time.now().to_nsec()) +
											 ' ' +  str(posx) + ' ' + str(posy) + '\n')
							x.append(posx)
							y.append(posy)

						posxAnt = posx
						posyAnt = posy

						
						#x.append(dis*math.cos(ang))
						#y.append(dis*math.sin(ang))
						#print 'Ang',ang,'Dis',dis

			t0 = tActual

			print 'Posicion robot [m]: x = ' + str(robot.pose_x)+'; y = ' +str(robot.pose_y)
			print 'Orientacion robot [grados]: ' + str(robot.angle)
		return x,y


	def features_callback(self, data):
		#print data.features.x, ',', data.features.y
		#self.featureX = []
		#self.featureY = []
		self.landMarks = []

		for p in pc2.read_points(data.features, field_names = ("x", "y", "z"), skip_nans=True):
			#print " x : %f  y: %f  z: %f" %(p[0],p[1],p[2])
			#print " x : %f  y: %f " %(p[0],p[1])
			#self.featureX.append(p[0])
			#self.featureY.append(p[1])
			point = Point2D(p[0], p[1])
			self.landMarks.append(point)


	def __init__(self):

		# inicializa nodo ROS
		rospy.init_node('Simulation');
		robotID = ""

		init_x = 0.0
		init_y = 0.0
		init_angle = 0

		robot = robot_utilities.Robot(robotID, init_x, init_y, init_angle)

		# Subscripcion a los topicos de interes: odometria (Odometry) y sensor de distancia (LaserScan)
		rospy.Subscriber(robotID+'/odom', Odometry, robot.odom_callback)
		#rospy.Subscriber(robotID+'/base_scan', LaserScan, R.ranger_callback)
		rospy.Subscriber(robotID+'p3dx/laser/scan', LaserScan, robot.ranger_callback)
		rospy.Subscriber(robotID+'/circleFeatures', Circle2DDetections, self.features_callback)
		
		# Observacion: Las funciones de callback se ejecutan cada vez que el programa recibe informacion 
		# desde un topico. Se utilizan, generalmente, para actualizar datos.
		# Pueden utilizarse funciones de callback programadas por el alumno, 
		# diferentes a las provistas por la clase Robot

		# Se recomienda que este ciclo vaya antes de todas las instrucciones de control
		# Sirve para asegurar que el Robot recibio su primera lectura de sensor
		while not rospy.is_shutdown() and len(robot.distances)==0:
			robot.rate.sleep()

		self.measFile.write('#segundos nanosegundos Rango theta Radio\n')
		self.deadReckFile.write('#segundos nansegundos X Y Theta\n')

		t0 = int(rospy.Time.now().to_nsec())

		try:
			while not rospy.is_shutdown() and True:						
			
				robot.cmd.linear.x = 0.2
				#robot.cmd.linear.x = 0.0
				# Velocidad angular [radianes/segundo]; rango aceptable: [-pi/2, pi/2 ], aprox. [-1.57, 1.57]			
				

				#if(robot.pose_x<2.5):
				#	robot.cmd.angular.z = 0.03
				#else:
				# 	robot.cmd.angular.z = -0.08

				#if(robot.pose_x>1):
				#	robot.cmd.angular.z = -0.08
				#else:
				robot.cmd.angular.z = 0.0

				
				robot.cmd_pub.publish(robot.cmd)
				
				#print "_______________________"
				#print 'Primera lectura del laser [m]: '+ str(robot.distances[0])
				#print 'Posicion robot [m]: x = ' + str(robot.pose_x)+'; y = ' +str(robot.pose_y)
				#print 'Orientacion robot [grados]: ' + str(robot.angle)
				print 'Robot Pose: x=' + str(robot.pose_x)+'; y=' +str(robot.pose_y) + '; theta=' + str(robot.angle)
				#robot.show_distance()

				
				#tActual = int(rospy.Time.now().to_nsec())
				#print abs(tActual-t0)

				#if(abs(tActual-t0)>2e9):

				self.deadReckFile.write( str(int(rospy.Time.now().to_sec())) + ' ' + str(rospy.Time.now().to_nsec()) + ' ' + 
					str(robot.pose_x) + ' ' + str(robot.pose_y) + ' ' + str(robot.angle*math.pi/180) + '\n' )

				#print 'Samples: ', len(robot.distances)
	
				#for i in range(0, len(robot.distances)):
				#	if np.isfinite(robot.distances[i]):
				#		ang = ((robot.angle*math.pi/180.0) + robot.laser_angles[i])
				#		posx = (robot.pose_x + (robot.distances[i]*math.cos(ang)))
				#		posy = (robot.pose_y + (robot.distances[i]*math.sin(ang)))

						#ang = robot.laser_angles[i]
						#posx = robot.distances[i]*math.cos(ang)
						#posy = robot.distances[i]*math.sin(ang)

						#print robot.laser_angles[i], ', ', robot.distances[i]
						#print 'x:', posx, ', y:', posy
						
				#		self.measFile.write( str(int(rospy.Time.now().to_sec())) + ' ' + str(rospy.Time.now().to_nsec()) +
				#			' ' +  str(posx) + ' ' + str(posy) + '\n')


				#for i in range(0, len(self.featureX)):				
				for p in self.landMarks:
					posx = robot.pose_x + p.x
					posy = robot.pose_y + p.y					
					self.measFile.write( str(int(rospy.Time.now().to_sec())) + ' ' + str(rospy.Time.now().to_nsec()) +
							' ' +  str(posx) + ' ' + str(posy) + '\n')					

				robot.rate.sleep()
				rospy.sleep(0.1)
				
				#t0 = tActual

		except KeyboardInterrupt:
			print 'Saliendo'
			robot.stop()
			self.measFile.close()
			self.deadReckFile.close()		

		#x,y = self.keyControl(robot)

		#robot.stop()
		#self.measFile.close()
		#self.deadReckFile.close()

		#plt.plot(x, y, 'r.')
		#plt.ylabel('metros')
		#plt.show()
        
if __name__ == "__main__": Controller()
