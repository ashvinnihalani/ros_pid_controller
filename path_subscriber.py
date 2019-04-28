#! /usr/bin/env python

import rospy
import math
import numpy as np 
#import pprint
from geometry_msgs.msg import Twist, PoseStamped, Pose
from nav_msgs.msg import Odometry as Odom
from nav_msgs.msg import Path
from std_msgs.msg import Bool

from tf.transformations import euler_from_quaternion
epsilon = 2.220446049250313e-16 # Episalon for floating point comparasion
pp = None
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
		pp.pprint(self.path.poses)
		print 'Len of Queue of Waypoints' + str(len(self.path.poses))
		if len(self.path.poses) > 0:
			pose = self.path.poses.pop(0)
			while len(self.path.poses) > 3:
				first = self.path.poses.pop(0)
				second = self.path.poses.pop(0)
				third = self.path.poses.pop(0)
				if self.isBetween(first,second,third):
					self.path.poses.insert(0,third)
					self.path.poses.insert(0,first)
					print "Trimming Path"
				else:
					self.path.poses.insert(0,third)
					self.path.poses.insert(0,second)
					self.path.poses.insert(0,first)
					break
				
			if len(self.path.poses) > 0:
				pose = self.path.poses.pop(0)
			wp = Twist()
			wp.linear.x = pose.pose.position.x
			wp.linear.y = pose.pose.position.y
			if len(self.path.poses) > 0:
				wp.angular.z = self.get_euler_yaw(self.path.poses[-1].pose.orientation)
			else:
				wp.angular.z = self.get_euler_yaw(pose.pose.orientation)
			print "Goal: x: %.2f y: %.2f yaw: %.2f" %(wp.linear.x,wp.linear.y,wp.angular.z)


			self.wp_pub.publish(wp)
	def isBetween(self,a, b, c):
			crossproduct = (c.pose.position.y - a.pose.position.y) * (b.pose.position.x - a.pose.position.x) - (c.pose.position.x - a.pose.position.x) * (b.pose.position.y - a.pose.position.y)
			if abs(crossproduct) > epsilon:
				return False
			return True
			dotproduct = (c.pose.position.x - a.pose.position.x) * (b.pose.position.x - a.pose.position.x) + (c.pose.position.y - a.pose.position.y)*(b.pose.position.y - a.pose.position.y)
			if dotproduct < 0:
				return False

			squaredlengthba = (b.pose.position.x - a.pose.position.x)*(b.pose.position.x - a.pose.position.x) + (b.pose.position.y - a.pose.position.y)*(b.pose.position.y - a.pose.position.y)
			if dotproduct > squaredlengthba:
				return False
			return True
	def wp_publisher_cb(self, msg):
		if msg.data:
			self.wp_publisher()


	def get_euler_yaw(self, orientation):
		euler = euler_from_quaternion([orientation.x,orientation.y,orientation.z,orientation.w])
		return euler[2]

if __name__ == '__main__':
	rospy.init_node('path_subscriber')
	ps = PathSubscriber()
#	pp = pprint.PrettyPrinter(indent=4)
	rospy.spin()
