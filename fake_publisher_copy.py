#! /usr/bin/env python

import rospy
import math
import numpy as np 

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry as Odom

class getState:
	def __init__(self):
		rospy.Subscriber('/odom', Odom, self.update_state)
		self.state = Odom()

	
	def update_state(self, state):
		self.state = state

def quat_to_eul(state):
	return np.arctan2(2*(state.pose.pose.orientation.z*state.pose.pose.orientation.w),1-2*(state.pose.pose.orientation.w**2))

def go_to(goal_position):
	state = getState()
	rospy.sleep(.1)

	state = state.state
	state = np.array([state.pose.pose.position.x, state.pose.pose.position.y, \
			quat_to_eul(state)])
	diffArr = abs(goal_position - state)
	diff = max(diffArr)

	g = Twist()
	g.linear.x = goal_position[0]
	g.linear.y = goal_position[1]
	g.angular.z = goal_position[2]

	while (diff > .1) and not rospy.is_shutdown():
		fake.publish(g)
		state = getState()
		rospy.sleep(.1)
		state = state.state
		state = np.array([state.pose.pose.position.x, state.pose.pose.position.y, \
			quat_to_eul(state)])
		diffArr = goal_position - state
		diffArr[2] %= (2*3.14159)
		if diffArr[2] > 3.14159:
			diffArr[2] -= 2*3.14159
		diff = max(abs(diffArr))
		print(g.angular.z)

if __name__ == '__main__':
	rospy.init_node('fake_publisher')
	fake = rospy.Publisher('/goal_position', Twist, queue_size = 1)

	state = getState()
	rospy.sleep(.1)
	state = state.state
	init_state = np.array([state.pose.pose.position.x, state.pose.pose.position.y, \
			quat_to_eul(state)])
	
	# goal_positions = np.array([init_state + np.array([0,0,-1.]), \
	# 				  init_state + np.array([.6,0,-1.]), \
	# 				  init_state + np.array([.6,.6,-1.]), \
	# 				  init_state + np.array([0,.6,-1.]), \
	# 				  init_state + np.array([0,0,-1.])])

	# goal_positions = np.array([init_state + np.array([0.,0.,0.]), \
	# 				init_state + np.array([.6,0.,0.]), \
	# 				init_state + np.array([.6,.6,0.]), \
	# 				init_state + np.array([0.,.6,0.]), \
	# 				init_state + np.array([0.,0.,0.])])

	goal_positions = np.array([init_state + np.array([.6,0.5,-1.5]), \
					  init_state + np.array([0.,.6,3.14]), \
					  init_state + np.array([0.,0.,0.])])


	for i in range(len(goal_positions)):
		go_to(goal_positions[i])
		print(i)
