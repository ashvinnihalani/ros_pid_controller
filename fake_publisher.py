#! /usr/bin/env python

import rospy
import math
import numpy as np 

from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry

class Fake_Publisher:
	def __init__(self):
		self.publish_command = rospy.Publisher('/goal_position', Twist, queue_size=1)
		self.publish_odom = rospy.Publisher('/odom', Odometry, queue_size=1)
		
		g = Twist()
		g.linear.x = 4
		g.linear.y = 3
		g.angular.z = 1
		self.g = g

		o = Odometry()
		o.pose.pose.orientation.x = 2
		o.pose.pose.orientation.y = 3
		o.pose.pose.orientation.w = 1.2
		self.o = o

		self.rate = rospy.Rate(1)
	
	def run(self):
		while not rospy.is_shutdown():
			self.publish_command.publish(self.g)
			self.publish_odom.publish(self.o)
			self.rate.sleep()

if __name__ == '__main__':
	rospy.init_node('fake_publisher')
	fake_publisher = Fake_Publisher()
	print 'Fake Publisher Running'
	fake_publisher.run()
