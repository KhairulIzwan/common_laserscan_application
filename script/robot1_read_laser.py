#!/usr/bin/env python

################################################################################
## {Description}: Read a Laserscan
################################################################################
## Author: Khairul Izwan Bin Kamsani
## Version: {1}.{0}.{0}
## Email: {wansnap@gmail.com}
################################################################################

# import the necessary Python packages
#from __future__ import print_function
#import sys
#import cv2
#import time

# import the necessary ROS packages
#from std_msgs.msg import String
#from sensor_msgs.msg import Image
#from sensor_msgs.msg import CameraInfo

#from cv_bridge import CvBridge
#from cv_bridge import CvBridgeError

#import math
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

import rospy

import numpy as np

class LaserPreview:
	def __init__(self):

#		self.bridge = CvBridge()
#		self.image_received = False
		self.laser_received = False
		
		self.move = Twist()

		rospy.logwarn("[Robot1] LaserRead Node [ONLINE]...")

		# rospy shutdown
		rospy.on_shutdown(self.cbShutdown)
			
		# Subscribe to LaserScan msg
		self.laser_topic = "/scan_robot1"
		self.laser_sub = rospy.Subscriber(self.laser_topic, LaserScan, self.cbLaser)
		
		# Publish to Twist msg
		twist_topic = "/cmd_vel_robot1"
		self.twist_pub = rospy.Publisher(twist_topic, Twist, queue_size=10)

		# Allow up to one second to connection
		rospy.sleep(1)
			
	# Get LaserScan reading
	def cbLaser(self, msg):

		try:
			self.scanValue = msg.ranges

		except KeyboardInterrupt as e:
			print(e)

		if self.scanValue is not None:
			self.laser_received = True
		else:
			self.laser_received = False		

	# Print info
	def cbLaserInfo(self):
		if self.laser_received:
#			rospy.loginfo([self.scanValue[0], self.scanValue[512]])
			center = len(self.scanValue) // 2
			if np.isnan(self.scanValue[center]):
				pass
			else:
				if self.scanValue[center] > 0.6:
#					rospy.loginfo("[Angle: 0]: %.4f" % (self.scanValue[0]))
#					pass

					self.move.linear.x = 0.10
					self.move.linear.y = 0.00
					self.move.linear.z = 0.00
					
					self.move.angular.x = 0.00
					self.move.angular.y = 0.00
					self.move.angular.z = 0.00
					
					self.twist_pub.publish(self.move)
				else:
					rospy.logwarn("[Robot1] OBSTACLE! [Angle: 0]: %.4f" % (self.scanValue[center]))
					
					self.move.linear.x = 0.00
					self.move.linear.y = 0.00
					self.move.linear.z = 0.00
					
					self.move.angular.x = 0.00
					self.move.angular.y = 0.00
					self.move.angular.z = 0.00
					
					self.twist_pub.publish(self.move)
		else:
			rospy.logerr("No Laser Reading")
			
			self.move.linear.x = 0.00
			self.move.linear.y = 0.00
			self.move.linear.z = 0.00
			
			self.move.angular.x = 0.00
			self.move.angular.y = 0.00
			self.move.angular.z = 0.00
			
			self.twist_pub.publish(self.move)
			
		# Allow up to one second to connection
		rospy.sleep(0.1)

	# rospy shutdown callback
	def cbShutdown(self):

		rospy.logerr("[Robot1] LaserRead Node [OFFLINE]...")
		
		self.move.linear.x = 0.00
		self.move.linear.y = 0.00
		self.move.linear.z = 0.00
		
		self.move.angular.x = 0.00
		self.move.angular.y = 0.00
		self.move.angular.z = 0.00
		
		self.twist_pub.publish(self.move)

if __name__ == '__main__':

	# Initialize
	rospy.init_node('robot1_read_scan', anonymous=False)
	laser = LaserPreview()

	# Camera preview
	while not rospy.is_shutdown():
		laser.cbLaserInfo()
