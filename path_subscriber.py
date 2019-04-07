#! /usr/bin/env python

import rospy
import math
import numpy as np 

from geometry_msgs.msg import Twist, PoseStamped, Pose
from nav_msgs.msg import Odometry as Odom
from nav_msgs.msg import Path
from std_msgs.msg import Bool

from tf.transformations import euler_from_quaternion

class PathSubscriber():
	"""docstring for PathSubscriber"""
	def __init__(self):
		self.wp_pub = rospy.Publisher('/goal_position', Twist, queue_size = 10)
		#rospy.Subscriber("/path_planning/short_term_position", Path, self.path_cb)
		rospy.Subscriber("/global_planner_node/path", Path, self.path_cb)
		rospy.Subscriber("/amcl_pose", PoseStamped, self.amcl_cb)
		rospy.Subscriber("/goal_position_cb", Bool, self.wp_publisher_cb)
		self.path = Path()
		self.odom = Pose()
		self.new_path = False
		#self.wp_publisher() 

	def path_cb(self, msg):
		self.path = msg
		self.new_path = True
		self.wp_publisher()

	def amcl_cb(self, msg):
		self.odom = msg.pose 

	def wp_publisher(self):
		r = rospy.Rate(10)
		if len(self.path.poses) > 0:
			t_min_pose = self.path.poses.pop(0)
			t_min = Twist()
			t_min.linear.x = t_min_pose.pose.position.x
			t_min.linear.y = t_min_pose.pose.position.y
			t_min.angular.z = self.get_euler_yaw(t_min_pose.pose.orientation)
			print "Goal: x: %.2f y: %.2f yaw: %.2f" %(t_min.linear.x,t_min.linear.y,t_min.angular.z)


			self.wp_pub.publish(t_min)

	def wp_publisher_cb(self, msg):
		if msg.data:
			self.wp_publisher()


	def get_euler_yaw(self, orientation):
		euler = euler_from_quaternion([orientation.x,orientation.y,orientation.z,orientation.w])
		return euler[2]

	def get_error(self, goal_twist, curr_pose):
		state = np.array([curr_pose.position.x, curr_pose.position.y, self.get_euler_yaw(curr_pose.orientation)])
		goal_position = np.array([goal_twist.linear.x, goal_twist.linear.y, goal_twist.angular.z])
		diffArr = goal_position - state
		dist_err = math.sqrt(diffArr[0]*diffArr[0] + diffArr[1]*diffArr[1])
		yaw_err = abs(diffArr[2])
		yaw_err %= (2*3.14159)
		if yaw_err > 3.14159:
			yaw_err -= 2*3.14159
		return dist_err, yaw_err

	def go_to(self, goal_twist):
		dist_err, yaw_err = self.get_error(goal_twist,self.odom)

		r = rospy.Rate(10)
		while (dist_err > .1) and yaw_err > np.deg2rad(2.5) and not rospy.is_shutdown():
			self.wp_pub.publish(goal_twist)
			dist_err, yaw_err = self.get_error(goal_twist,self.odom)
			r.sleep()

if __name__ == '__main__':
	rospy.init_node('path_subscriber')
	ps = PathSubscriber()
	rospy.spin()