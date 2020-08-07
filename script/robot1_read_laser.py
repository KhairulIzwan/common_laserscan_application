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
#from geometry_msgs.msg import Twist

import rospy

import numpy as np

class LaserPreview:
	def __init__(self):

#		self.bridge = CvBridge()
#		self.image_received = False
		self.laser_received = False

		rospy.logwarn("[Robot1] LaserRead Node [ONLINE]...")

		# rospy shutdown
		rospy.on_shutdown(self.cbShutdown)
			
		# Subscribe to LaserScan msg
		self.laser_topic = "/scan_robot1"
		self.laser_sub = rospy.Subscriber(self.laser_topic, LaserScan, self.cbLaser)

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
					pass
				else:
					rospy.logwarn("[Robot1] OBSTACLE! [Angle: 0]: %.4f" % (self.scanValue[center]))
		else:
			rospy.logerr("No Laser Reading")

	# rospy shutdown callback
	def cbShutdown(self):

		rospy.logerr("[Robot1] LaserRead Node [OFFLINE]...")

if __name__ == '__main__':

	# Initialize
	rospy.init_node('robot1_read_scan', anonymous=False)
	laser = LaserPreview()

	# Camera preview
	while not rospy.is_shutdown():
		laser.cbLaserInfo()
