import rospy
from roborts_msgs.msg import GimbalAngle
from geometry_msgs.msg import Vector3

K_yaw = 0.001
K_pitch = 0.001
yaw_min = -1.50
yaw_max = 1.50
pitch_min = -0.50
pitch_max = 0.50
error_pixel_max = 10

class automatic_aiming_config():
	def __init__(self):
		# subscribers
		self.yaw_current = 0
		self.pitch_current = 0
		
		self.yaw_rough = 0

		self.gimbal_camera_x = 0;
		self.gimbal_camera_y = 0;
		self.state = 0;

		# publishers
		self.yaw_next = 0;
		self.pitch_next = 0;

		rospy.init_node('automatic_aiming')
		# rospy.Subscriber('/cmd_gimbal_angle', GimbalAngle, self.callback_now)
		rospy.Subscriber('/detection/base_camera', Vector3, self.callback_base_camera)
		rospy.Subscriber('/detection/gimbal_camera', Vector3, self.callback_gimbal_camera)
		pub = rospy.Publisher("/cmd_gimbal_angle", GimbalAngle, queue_size = 10)
		rate = rospy.Rate(10)

		# initialization
		while not rospy.is_shutdown():
			gimbal_angle_base_camera = GimbalAngle()
			gimbal_angle_base_camera.yaw_angle = self.yaw_rough
			gimbal_angle_base_camera.pitch_angle = 0
			pub.publish(gimbal_angle_base_camera)

			while self.gimbal_camera_x != 0 and self.gimbal_camera_y != 0:
				gimbal_angle_gimbal_camera = GimbalAngle()
				# precise aiming
				gimbal_angle_gimbal_camera.yaw_mode = True
				gimbal_angle_gimbal_camera.pitch_mode = True
				gimbal_angle_gimbal_camera.yaw_angle = self.yaw_next
				gimbal_angle_gimbal_camera.pitch_angle = self.pitch_next
				pub.publish(gimbal_angle_gimbal_camera)

				if max(abs(self.gimbal_camera_x),abs(self.gimbal_camera_y)) <= error_pixel_max:
					break
				rate.sleep()

			rate.sleep()

	# def callback_now(self, data):
	# 	self.yaw_current = data.yaw_angle
	# 	self.pitch_current = data.pitch_angle

	def callback_base_camera(self, data):
		self.yaw_rough = -data.y

	def callback_gimbal_camera(self, data):
		self.gimbal_camera_x = data.x
		self.gimbal_camera_y = data.y

		self.yaw_next = - K_yaw * self.gimbal_camera_x
		self.pitch_next = - K_pitch * self.gimbal_camera_y

		# Threshold
		if self.yaw_next <= yaw_min:
			self.yaw_next = yaw_min 
		elif self.yaw_next >= yaw_max:
			self.yaw_next = yaw_max

		if self.pitch_next <= pitch_min:
			self.pitch_next = pitch_min 
		elif self.pitch_next >= pitch_max:
			self.pitch_next = pitch_max

if __name__ == '__main__':
	try:
		config = automatic_aiming_config()
		rospy.spin()
	except rospy.ROSInterruptException:
		pass