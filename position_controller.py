#! /usr/bin/env python

import rospy
import math
import numpy as np 

from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from roborts_msgs.msg import EstimatedState

class Position_Controller:
	def __init__(self):
		self.publish_command = rospy.Publisher('/cmd_vel',Twist,queue_size=1)
		rospy.Subscriber('/estimated_state', EstimatedState, self.update_state)
		rospy.Subscriber('/goal_position',Twist,self.update_goal_postion)

		self.state = EstimatedState()

		goal_position = rospy.get_param('goal_position', [2.7, 2.7, 1.2, 0])

		g = Twist()
		g.linear.x = goal_position[0]
		g.linear.y = goal_position[1]
		g.angular.z = goal_position[2]
		self.goal_position = g

		stop = Twist()
		stop.linear.x = 0
		stop.linear.y = 0
		stop.angular.z = 0
		self.stop_cmd = stop

		self.Kp_x = .3
		self.Ki_x = 0.
		self.Kd_x = .36
		self.Kp_y = .3
		self.Ki_y = 0.
		self.Kd_y = .36
		self.Kp_yaw = rospy.get_param('~Kp_yaw', 1.)
		self.Ki_yaw = 0.
		self.Kd_yaw = rospy.get_param('~Kd_yaw', 0.) 

		self.max_xvel = 3.0
		self.max_yvel = 2.0
		self.max_yaw = rospy.get_param('~max_yaw', .5) #.1

# =========== ROS Update Functions =========================================

	def update_goal_postion(self, wp):
		self.goal_position = wp

	def update_state(self, state):
		self.state = state
		self.run_position_controller()


# ========== Main Position Controller ======================================

	def run_position_controller(self):

		goal = self.goal_position 
		yaw = goal.angular.z*np.pi/180
		goal = np.array([[np.cos(yaw),-np.sin(yaw),goal.linear.x], \
						[np.sin(yaw),np.cos(yaw),goal.linear.y],   \
						[0,0,1.]])

		state = self.state
		erroryaw = yaw*180/np.pi - state.yaw

		if erroryaw > 180:
			erroryaw -= 360
		elif erroryaw < -180:
			erroryaw = yaw%360

		yaw = state.yaw*np.pi/180
		state = np.array([[np.cos(yaw),-np.sin(yaw),state.x], \
						  [np.sin(yaw),np.cos(yaw),state.y], \
						  [0,0,1.]])

		print '=============='
		print "goal",self.goal_position

		###############  INSERT CODE BELOW ######################
		
		# Calculate state error (HINT!! Rotate the errors into body frame)
		error = np.dot(np.linalg.inv(state),goal)

		errorx,errory= error[:2,2]

		print "State Error: x: %.2f y: %.2f yaw: %.2f" %(errorx,errory,erroryaw)


		# Calculate twist commands in each axis
		state = self.state

		dx = state.dx*np.cos(yaw) + state.dy*np.sin(yaw)
		dy = -state.dx*np.sin(yaw) + state.dy*np.cos(yaw)

		cmd_x = self.Kp_x*errorx + -self.Kd_x*dx
		cmd_y = self.Kp_y*errory + -self.Kd_y*dy
		cmd_yaw = self.Kp_yaw*erroryaw


		print "Unbounded Command: x: %.2f y: %.2f yaw: %.2f" %(cmd_x,cmd_y,cmd_yaw)
	  
		# Saturate commands to safe limits
		if(abs(cmd_x) > self.max_xvel):
			rel = self.max_xvel/abs(cmd_x)
			cmd_x *= rel
		if(abs(cmd_y) > self.max_yvel):
			rel = self.max_yvel/abs(cmd_y)
			cmd_y *= rel

		if(cmd_yaw > 0):
			cmd_yaw = min(cmd_yaw,self.max_yaw)
		else:
			cmd_yaw = max(cmd_yaw,-self.max_yaw)

		print "Saturated Command: x: %.2f y: %.2f yaw: %.2f" %(cmd_x,cmd_y,cmd_yaw)

		############### INSERT CODE ABOVE ######################
		cmd_out = Twist()
		cmd_out.linear.x = cmd_x 
		cmd_out.linear.y = cmd_y
		cmd_out.angular.z = cmd_yaw
		
		self.publish_command.publish(cmd_out)


if __name__ == '__main__':
	rospy.init_node('position_controller')
	controller = Position_Controller()
	print 'Controller Running'
	rospy.spin()
