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
	
	measFile = open('/home/claydergc/MyPackages/GraphSLAM/src/python-helpers/v4-clay-data/data/Clay/measurement.dat', 'w+')
	deadReckFile = open('/home/claydergc/MyPackages/GraphSLAM/src/python-helpers/v4-clay-data/data/Clay/deadReckoning.dat', 'w+')
	landMarks = []

	def keyControl(self, robot):	

		KEY_UP = 65
		KEY_DOWN = 66
		KEY_RIGHT = 67
		KEY_LEFT = 68
		USER_QUIT = 100
		
		MAX_FORWARD = 0.5
		MAX_LEFT = 0.1		
		MIN_FORWARD = -0.5
		MIN_LEFT = -0.1

		forward = 0.0
		left = 0.0
		keyPress = 0

		angulo = 0		

		while(keyPress != USER_QUIT):
			keyPress = getch.getArrow()		

			if((keyPress == KEY_UP) and (forward <= MAX_FORWARD)):
				forward = 0.5
				left = 0.0				
			elif((keyPress == KEY_DOWN) and (forward >= MIN_FORWARD)):
				forward = -0.5
				left = 0.0			
			elif((keyPress == KEY_LEFT) and (left >= MIN_LEFT)):								
				left = -0.1							
			elif((keyPress == KEY_RIGHT) and (left <= MAX_LEFT)):				
				left = 0.1				
		
			robot.cmd.linear.x = forward
			robot.cmd.angular.z = left
			robot.cmd_pub.publish(robot.cmd)

			self.deadReckFile.write( str(int(rospy.Time.now().to_sec())) + ' ' + str(rospy.Time.now().to_nsec()) + ' ' + 
					str(robot.pose_x) + ' ' + str(robot.pose_y) + ' ' + str(robot.angle*math.pi/180) + '\n' )
								
			for p in self.landMarks:									
				posx = p.x
				posy = p.y				
				self.measFile.write( str(int(rospy.Time.now().to_sec())) + ' ' + str(rospy.Time.now().to_nsec()) +
						' ' +  str(posx) + ' ' + str(posy) + '\n')
			

			robot.rate.sleep()			
			
			print 'Posicion robot [m]: x = ' + str(robot.pose_x)+'; y = ' +str(robot.pose_y)
			print 'Orientacion robot [grados]: ' + str(robot.angle)			


	def features_callback(self, data):		
		self.landMarks = []

		for p in pc2.read_points(data.features, field_names = ("x", "y", "z"), skip_nans=True):
			#print " x : %f  y: %f  z: %f" %(p[0],p[1],p[2])
			#print " x : %f  y: %f " %(p[0],p[1])			
			point = Point2D(p[0], p[1])
			self.landMarks.append(point)


	def __init__(self):
		
		rospy.init_node('stage_controller');
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
				
		while not rospy.is_shutdown() and len(robot.distances)==0:
			robot.rate.sleep()

		self.measFile.write('#segundos nanosegundos Rango theta Radio\n')
		self.deadReckFile.write('#segundos nansegundos X Y Theta\n')
				
		self.keyControl(robot)			
		
		print 'Saliendo'
		robot.cmd.linear.x = 0
		robot.cmd.angular.z = 0
		robot.cmd_pub.publish(robot.cmd)
		self.measFile.close()
		self.deadReckFile.close()			
        
if __name__ == "__main__": Controller()
