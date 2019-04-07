#! /usr/bin/env python

import rospy
import math
import tf
from geometry_msgs.msg import PoseStamped, Pose, Vector3, Point


class EnemyPos():
	"""docstring for EnemyPos"""
	def __init__(self):
		self.pos_pub = rospy.Publisher('/enemy_pose', PoseStamped, queue_size = 10)
		rospy.Subscriber("/detection/base_camera", Vector3, self.enemy_cb)
		self.tf = tf.TransformListener()

	def enemy_cb(self, msg):
		dist, theta = msg.x/1000.0, msg.y
		#if self.tf.frameExists("/map") and self.tf.frameExists("/stereoCameraL"):
		t = self.tf.getLatestCommonTime("/map", "/stereoCameraL")
		p1 = PoseStamped()
		p1.header.frame_id = "/stereoCameraL"
		#p1.header.stamp = rospy.Time.now()
		p1.pose.position.x = dist * math.cos(theta)
		p1.pose.position.y = dist * math.sin(theta)
		p1.pose.position.z = 0
		p1.pose.orientation.w = 1.0    # Neutral orientation
		p_in_map = self.tf.transformPose("/map", p1)
		self.pos_pub.publish(p_in_map)


if __name__ == '__main__':
	rospy.init_node('enemy_pos_pub')
	ep = EnemyPos()
	rospy.spin()