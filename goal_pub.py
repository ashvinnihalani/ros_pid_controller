#! /usr/bin/env python

import rospy
import math
import tf
from geometry_msgs.msg import PoseStamped


class EnemyPos():
	"""docstring for EnemyPos"""
	def __init__(self):
		self.goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size = 10)
		self.p1_ = [0.7845,3.2574,0]
		self.p2_ = [7.4849,4.3242,0]
		self.goal_publish(self.p1_)

	def goal_publish(self, p1_):
		p1 = PoseStamped()
		p1.header.frame_id = "/map"
		#p1.header.stamp = rospy.Time.now()
		p1.pose.position.x = p1_[0]
		p1.pose.position.y = p1_[1]
		p1.pose.position.z = 0
		self.goal_pub.publish(p1)
		rospy.loginfo("goal sent")


if __name__ == '__main__':
	rospy.init_node('enemy_pos_pub')
	ep = EnemyPos()
	rospy.spin()