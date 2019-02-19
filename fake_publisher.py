#! /usr/bin/env python

import rospy
import math
import numpy as np 

from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry

class Fake_Publisher:
	def __init__(self):
		self.publish_command = rospy.publisher('/goal_position', Twist, queue_size=1)
		self.publish_odom = rospy.publisher('/odom', Odometry, queue_size=1)
		
		g = Twist()
		g.linear.x = 4
		g.linear.y = 3
		g.angular.z = 1

		o = Odometry()
		o.pose.pose.orientation.x = 2
		o.pose.pose.orientation.y = 3
		o.pose.post.orientation.w = 1.2

		rate = rospy.Rate(1)

		while not rospy.is_shutdown():
			self.publish_command.publish(g)
			self.publish_odom(o)
			rate.sleep()

if __name__ == '__main__':
	rospy.init_node('fake_publisher')
	fake_publisher = Fake_Publisher()
	print 'Fake Publisher Running'
	rospy.spin()