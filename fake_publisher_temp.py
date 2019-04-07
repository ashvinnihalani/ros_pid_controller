#! /usr/bin/env python

import rospy
import math
import numpy as np 

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from roborts_msgs.msg import EstimatedState

class Fake_Publisher:
	def __init__(self):
		self.publish_command = rospy.Publisher('/goal_position', Twist, queue_size=1)
		self.publish_state = rospy.Publisher('/estimated_state', EstimatedState, queue_size=1)
		
		g = Twist()
		g.linear.x = 4.
		g.linear.y = 3.
		g.angular.z = 1.
		self.g = g

		state = EstimatedState()
		state.x = 2.
		state.y = 3.
		state.yaw = 1.2
		state.dx = .2
		state.dy = 0.
		state.dyaw = 0.
		self.state = state

		self.rate = rospy.Rate(1)
	
	def run(self):
		while not rospy.is_shutdown():
			self.publish_command.publish(self.g)
			self.publish_state.publish(self.state)
			self.rate.sleep()

if __name__ == '__main__':
	rospy.init_node('fake_publisher')
	fake_publisher = Fake_Publisher()
	print 'Fake Publisher Running'
	fake_publisher.run()
