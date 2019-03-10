#! /usr/bin/env python

import rospy
import math
import numpy as np 

from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry as Odom
from roborts_msgs.msg import TwistAccel

class getState:
	def __init__(self):
		rospy.Subscriber('/odom', Odom, self.update_state)
		self.state = Odom()

	
	def update_state(self, state):
		self.state = state

def quat_to_eul(state):
	return np.arctan2(2*state.pose.pose.orientation.z*state.pose.pose.orientation.w,1-2*state.pose.pose.orientation.w**2)

def go_to(goal_position):
	state = getState().state
	state = np.array([state.pose.pose.position.x, state.pose.pose.position.y, \
			quat_to_eul(state)])
	diffArr = abs(goal_position - state)
	diff = max(diffArr)

	g = Twist()
	g.linear.x = goal_position[0]
	g.linear.y = goal_position[1]
	g.angular.z = goal_position[2]

	while (diff > .02):
		fake.publish(g)
		time.sleep(.5)

		state = np.array([state.pose.pose.position.x, state.pose.pose.position.y, \
			quat_to_eul(state)])
		diffArr = abs(goal_position - state)
		diff = np.max(diffArr)

if __name__ == '__main__':
	rospy.init_node('fake_publisher')
	fake = rospy.Publisher('goal_position', Twist, queue_size = 1)

	state = getState().state
	init_state = np.array([state.pose.pose.position.x, state.pose.pose.position.y, \
			quat_to_eul(state)])
	
	goal_positions = np.array([init_state + np.array([.1,0,0]), \
					  init_state + np.array([.1,.1,0]), \
					  init_state + np.array([0,.1,0]), \
					  init_state + np.array([0,0,0])], \
					  init_state + np.array([0,0,.5]))
	while True:
		for goal in goal_positions:
			go_to(goal)
	print 'Fake Publisher Running'
	fake_publisher.run()
