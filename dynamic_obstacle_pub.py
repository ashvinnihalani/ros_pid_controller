#! /usr/bin/env python

import rospy
import math
import numpy as np 

from pid_controller.msg import DynObstacleArray, DynObstacle   


if __name__ == '__main__':
	rospy.init_node('dynamic_obstalces_pub')
	dyn_obstacles_pub = rospy.Publisher('/controller/dynamic_obstalces', DynObstacleArray, queue_size = 10)
	obstacle_1 = DynObstacle()
	obstacle_2 = DynObstacle()
	obstacle_1.x = 2.0 / 0.05
	obstacle_1.y = 2.0 / 0.05
	obstacle_1.width = 0.5 / 0.05
	obstacle_1.height = 1.5 / 0.05
	obstacle_2.x = 1.0 / 0.05
	obstacle_2.y = 1.0 / 0.05
	obstacle_2.width = 1.5 / 0.05
	obstacle_2.height = 0.5 / 0.05
	obstacles = [obstacle_1,obstacle_1]
	while not rospy.is_shutdown():
		dyn_obstacles_pub.publish(obstacles)
		rospy.loginfo("msg sent:{}".format(obstacles))
		rospy.sleep(3)