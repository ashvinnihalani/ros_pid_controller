#!/usr/bin/env python


import rospy
from geometry_msgs.msg import Vector3

def txt_reader():
    rospy.init_node('txt_reader')
    base_pub = rospy.Publisher('/detection/base_camera', Vector3, queue_size=10)
    gimbal_pub = rospy.Publisher('/detection/gimbal_camera', Vector3, queue_size = 10)
    v3_base = Vector3()
    v3_gimbal = Vector3()

    r = rospy.Rate(20)
    while not rospy.is_shutdown():
        file = open("/home/rmcal/MYNT-EYE-S-SDK/location.txt", "r") 
        line = file.read().split(" ")
        try:
            dist, theta = float(line[0]), float(line[1])
        except:
            pass
        #rospy.loginfo("[base camera]: dist is {}, theta is {}".format(dist, theta))
        v3_base.x = dist
        v3_base.y = theta
        base_pub.publish(v3_base)
        file2 = open("/home/rmcal/Perception/Cal_Robomaster_ICRA_2019/Perception/exchange.txt", "r") 
        line = file2.read().split("_")
        try:
            x, y= line[0], line[1]
            y.replace(" ","")
            x, y = float(x), float(y)
        except:
            pass
        #rospy.loginfo("[turret camera]: x is {}, y is {}".format(dist, theta, state))
        v3_gimbal.x = x
        v3_gimbal.y = y
        v3_gimbal.z = 0
        print(v3_gimbal)
        gimbal_pub.publish(v3_gimbal)
        r.sleep()


if __name__ == '__main__':
    try:
        txt_reader()
    except rospy.ROSInterruptException:
        pass