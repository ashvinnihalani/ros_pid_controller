#! /usr/bin/env python

import rospy
import math
import numpy as np 

from geometry_msgs.msg import Twist
# from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry as Odom
from roborts_msgs.msg import TwistAccel

class Position_Controller:
	def __init__(self):
		print('Intialize')
		#First set the inital variables
		self.flag = 0
		self.state = Twist()
		self.last_state = Twist()
		#Set the publish and the subsriber
		self.publish_command = rospy.Publisher('/cmd_vel_acc',TwistAccel,queue_size=1)
		rospy.Subscriber('/odom', Odom, self.update_state)
		rospy.Subscriber('/goal_position',Twist,self.update_goal_postion)
		# Topic goal position needed from path planning
		
#		g.linear.x = self.init_goal[0]
#		g.linear.y = self.init_goal[1]
#		g.angular.z = self.init_goal[2]
		self.goal_position = Twist()

		stop = Twist()
		stop.linear.x = 0
		stop.linear.y = 0
		stop.angular.z = 0
		self.stop_cmd = stop

		self.Kp_x = .8
		self.Ki_x = 0.
		self.Kd_x = 0
		self.Kp_y = .8
		self.Ki_y = 0.
		self.Kd_y = 0
		self.Kp_yaw = .8
		self.Ki_yaw = 0.
		self.Kd_yaw = 0 

		self.max_xvel = 3. # Meters /sec
		self.max_yvel = 2. #Meters /sec 
		self.max_yaw  = 3. #radians sec
 
# =========== ROS Update Functions =========================================

	def update_goal_postion(self, wp):
		self.goal_position = wp

	def update_state(self, state_odom):

		#Make sure there is an inital state first. 
		if self.flag == 0:
			self.func_init_goal()
		self.state = Twist()
		self.state.linear.x = state_odom.pose.pose.position.x
		self.state.linear.y = state_odom.pose.pose.position.y
		self.state.angular.z = np.arctan2(2*(state_odom.pose.pose.orientation.z*state_odom.pose.pose.orientation.w),1-2*(state_odom.pose.pose.orientation.w**2))
		self.run_position_controller()
		self.last_state = self.state 
		self.state = Twist()
		self.state.linear.x = state_odom.pose.pose.position.x
		self.state.linear.y = state_odom.pose.pose.position.y
		self.state.angular.z = np.arctan2(2*(state_odom.pose.pose.orientation.z*state_odom.pose.pose.orientation.w),1-2*(state_odom.pose.pose.orientation.w**2))
		print('Update State', self.state.linear.x,  self.state.linear.y, self.state.angular.z)
	
	def func_init_goal(self):
		pos = self.state
		self.goal_position.linear.x = pos.linear.x 
		self.goal_position.linear.y = pos.linear.y
		self.goal_position.angular.z = pos.angular.z - 5.8
		self.flag = 1

	def check_reached(self):
		threshhold_xy = .01 #Meters
		threshhold_angular = .19634954084 #pi/16
		if (abs(self.goal.position.linear.x - self.state.linear.x) < threshhold_xy 
			and abs(self.goal.position.linear.y - self.state.linear.y) < threshhold_xy
			and abs(self.goal_position.angular.z - self.state.angular.z) < threshhold_angular):
			return True
		else:
			return False

# ========== Main Position Controller ======================================

	def run_position_controller(self):

		goal = self.goal_position #twist
		curr_pos = self.state #Twist as well
		x, y, yaw = curr_pos.linear.x, curr_pos.linear.y, -curr_pos.angular.z
		goal_x, goal_y, goal_yaw = goal.linear.x, goal.linear.y, -goal.angular.z
		tf_mat = np.array([[np.cos(yaw),-np.sin(yaw),x], \
			               [np.sin(yaw),np.cos(yaw),y],  \
						   [0,0,1.0]])
		goal = np.array([[np.cos(goal_yaw),-np.sin(goal_yaw),goal_x], \
			             [np.sin(goal_yaw),np.cos(goal_yaw),goal_y],  \
						 [0,0,1.0]])

		print '=============='
		print "goal",goal_x, goal_y, goal_yaw

		##############  INSERT CODE BELOW ######################
		
		# Calculate state error (HINT!! Rotate the errors into body frame)
		error = np.dot(np.linalg.inv(tf_mat),goal)

		errorx,errory= -error[:2,2]
		
		# errorx,errory,erroryaw = goal.linear.x - curr_pos.linear.x , goal.linear.y - curr_pos.linear.y, goal.angular.z - curr_pos.angular.z
		# erroryaw = -errroryaw

		erroryaw = (goal_yaw - yaw)
		erroryaw %= 2*3.14159
		if erroryaw > 3.14159:
			erroryaw -= 2*3.14159
		print(erroryaw)
		print "State Error: x: %.2f y: %.2f yaw: %.2f" %(errorx,errory,erroryaw)
		


		# Calculate twist commands in each axis
		# state = self.state
		# calculate vel 
		dx = self.state.linear.x - self.last_state.linear.x 
		dy = self.state.linear.y - self.last_state.linear.y 
		dyaw = self.state.angular.y - self.last_state.angular.y 

		#dx_global = dx*np.cos(yaw) + dy*np.sin(yaw)
		#dy_global = -dx*np.sin(yaw) + dy*np.cos(yaw)

		cmd_x = self.Kp_x*errorx + -self.Kd_x*dx
		cmd_y = self.Kp_y*errory + -self.Kd_y*dy
		# cmd_yaw = self.Kp_yaw*erroryaw
		cmd_yaw = self.Kp_yaw*erroryaw + -self.Kd_yaw*dyaw


		#print "Unbounded Command: x: %.2f y: %.2f yaw: %.2f" %(cmd_x,cmd_y,cmd_yaw)
	  
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

		#print "Saturated Command: x: %.2f y: %.2f yaw: %.2f" %(cmd_x,cmd_y,cmd_yaw)

		############### INSERT CODE ABOVE ######################
		cmd_out = TwistAccel()
		cmd_out.twist.linear.x = cmd_x
		cmd_out.twist.linear.y = cmd_y
		cmd_out.twist.angular.z = cmd_yaw
		print("current state", curr_pos.linear.x, curr_pos.linear.y, curr_pos.angular.z)
		self.publish_command.publish(cmd_out)


if __name__ == '__main__':
	rospy.init_node('position_controller')
	controller = Position_Controller()
	print 'Controller Running'
	rospy.spin()
