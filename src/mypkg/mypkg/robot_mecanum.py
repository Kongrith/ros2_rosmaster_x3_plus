import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Pose2D, Twist, TransformStamped
# from std_msgs.msg import Float32, Int32
from nav_msgs.msg import Odometry	#

import tf2_ros
import math

# from yahboom
from Rosmaster_Lib.Rosmaster import Rosmaster
from sensor_msgs.msg import Imu,MagneticField, JointState

class Robot(Node):
	def __init__(self):
		super().__init__('robot_core_node')
		self.rover = Rosmaster(car_type = 2, com = "/dev/USBserial", delay = 0.002, debug = False)
		self.rover.create_receive_threading()

		# robot parameters
		self.wheel_radius = 0.251327
		self.wheel_separation = 0.20				# Distance between the two wheels on the same axis (meters).
		self.wheel_separation_length = 0.21			# Distance between the front and rear axis (meters).
		self.wheelbase = 0.25
		self.PPR = 2464												# PPR: Pulse per Revolution
		self.DPT = (2 * math.pi * self.wheel_radius) / self.PPR  	# DPT: Distance Per Tick (m/tick)

		# create subcriber
		self.vel_sub = self.create_subscription(Twist, 'cmd_vel', self.vel_callback, 1)

		# create publisher
		self.vel_pub = self.create_publisher(Twist,"vel_raw", 50)
		self.imu_pub = self.create_publisher(Imu, "/imu/data_raw", 100)
		self.mag_pub = self.create_publisher(MagneticField, "/imu/mag", 100)

		# related time
		self.timer = self.create_timer(0.1, self.timer_callback)
		self.last_time = self.get_clock().now()

		# Odometry
		self.robot_pose	= Pose2D()						# x, y, theta
		self.dx = 0.0
		self.dy = 0.0
		self.dtheta = 0.0

		self.last_encoder_m1 = 0
		self.last_encoder_m2 = 0
		self.last_encoder_m3 = 0
		self.last_encoder_m4 = 0
		self.get_delta_encoder()

		self.odom_pub = self.create_publisher(Odometry, "odom", 10)

		# TF
		self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

		#
		self.get_logger().info("robot core is running")

	def vel_callback(self, vel):
		if not isinstance(vel, Twist): return		# motion control, subscriber callback function

		##
		vx = vel.linear.x * 1.0
		vy = vel.linear.y * 1.0						# vy = msg.linear.y/1000.0*180.0/3.1416    #Radian system
		angular = vel.angular.z * 1.0				# wait for chang

		# นำค่ามาสั่งการ
		self.rover.set_car_motion(vx, vy, angular)

	def timer_callback(self):
		self.send_imu_data()
		self.send_odometry_data()

	def send_imu_data(self):
			imu = Imu()
			mag = MagneticField()

			# get data from Rosmaster Lib (USB serial)
			ax, ay, az = self.rover.get_accelerometer_data()
			gx, gy, gz = self.rover.get_gyroscope_data()
			mx, my, mz = self.rover.get_magnetometer_data()
			mx = mx * 1.0
			my = my * 1.0
			mz = mz * 1.0

			# IMU
			imu.header.stamp = self.get_clock().now().to_msg()
			imu.header.frame_id = "imu_link"
			imu.linear_acceleration.x = ax*1.0
			imu.linear_acceleration.y = ay*1.0
			imu.linear_acceleration.z = az*1.0
			imu.angular_velocity.x = gx*1.0
			imu.angular_velocity.y = gy*1.0
			imu.angular_velocity.z = gz*1.0

			# Magnetic
			mag.header.stamp = self.get_clock().now().to_msg()
			mag.header.frame_id = "mag_link"
			mag.magnetic_field.x = mx*1.0
			mag.magnetic_field.y = my*1.0
			mag.magnetic_field.z = mz*1.0

			# Publish IMU & Magnetics
			self.imu_pub.publish(imu)
			self.mag_pub.publish(mag)

	'''
		m1: front Left Wheel
		m2: Rear Left Wheel
		m3: front Right Wheel
		m4: Rear Right Wheel
	'''
	def get_delta_encoder(self):
		m1, m2, m3, m4 = self.rover.get_motor_encoder()
		# print("m1: {}, m2: {}, m3: {}, m4: {}".format(m1, m2, m3, m4))
		if self.last_encoder_m1 == 0:
			self.last_encoder_m1 = m1
		if self.last_encoder_m2 == 0:
			self.last_encoder_m2 = m2
		if self.last_encoder_m3 == 0:
			self.last_encoder_m3 = m3
		if self.last_encoder_m4 == 0:
			self.last_encoder_m4 = m4

		delta_encoder_m1 = m1 - self.last_encoder_m1
		delta_encoder_m2 = m2 - self.last_encoder_m2
		delta_encoder_m3 = m3 - self.last_encoder_m3
		delta_encoder_m4 = m4 - self.last_encoder_m4

		offset_m1 = delta_encoder_m1 if abs(delta_encoder_m1) < 24000 else 0
		offset_m2 = delta_encoder_m2 if abs(delta_encoder_m2) < 24000 else 0
		offset_m3 = delta_encoder_m3 if abs(delta_encoder_m3) < 24000 else 0
		offset_m4 = delta_encoder_m4 if abs(delta_encoder_m4) < 24000 else 0
		# print("m1: {}, m2: {}, m3: {}, m4: {}".format(offset_m1, offset_m2, offset_m3, offset_m4))

		self.last_encoder_m1 = m1
		self.last_encoder_m2 = m2
		self.last_encoder_m3 = m3
		self.last_encoder_m4 = m4

		return offset_m1, offset_m2, offset_m3, offset_m4

	def send_odometry_data(self):
		ts = self.get_clock().now()
		dt = ts - self.last_time		# หน่วยจะเป็น nano second
		dt = dt.nanoseconds*1e-9		# แปลงหน่วยเป็นวินาที

		twist = Twist()

		# get data from Rosmaster Lib (USB serial)
		vx, vy, angular = self.rover.get_motion_data()

		# Vx Vy Vz
		twist.linear.x = vx *1.0
		twist.linear.y = vy *1.0
		twist.angular.z = angular*1.0

		# Publish Velocity (x: forward/backward, y:translate left/rigth, z:ccw/cw)
		self.vel_pub.publish(twist)

		prev_robot_pose = self.robot_pose				# Pose2D
		# 	R = self.wheel_radius
		# 	L = self.wheelbase
		# 	TPR = self.tick_per_rev							# TPR: Tick per Revolution

		# 	DPT = (2 * math.pi * R) / TPR					# Distance Per Tick
		# 	DL = DPT * (tick['l'] - self.left_tick["bf"])
		# 	DR = DPT * (tick['r'] - self.right_tick["bf"])

		# 	DC = (DL + DR) * 0.5							# DC: Distance Center (เทียบตัวหุ่นเอง)
		# 	self.dx = DC*math.cos(prev_robot_pose.theta)	# ตัว theta จะเป็นมุม yaw
		# 	self.dy = DC*math.sin(prev_robot_pose.theta)	#
		# 	self.dtheta = (DR - DL) / L						# มาจากสมการ kinematics ของ diff drive

		## ???
		offset_m1, offset_m2, offset_m3, offset_m4 = self.get_delta_encoder()

		# print("m1: {}, m2: {}, m3: {}, m4: {}".format(offset_m1, offset_m2, offset_m3, offset_m4 ))
		front_left_distance  = self.DPT * offset_m1
		rear_left_distance   = self.DPT * offset_m2
		front_right_distance = self.DPT * offset_m3
		rear_right_distance   = self.DPT * offset_m4
		# print("m1:{}, m2:{}, m3:{}, m4:{}".format(front_left_distance, rear_left_distance, front_right_distance, rear_right_distance ))

		self.dx = (front_left_distance + rear_left_distance + front_right_distance + rear_right_distance) / 4
		self.dy = (- front_left_distance + rear_left_distance + front_right_distance - rear_right_distance) / 4
		self.dtheta = (- front_left_distance - rear_left_distance + front_right_distance + rear_right_distance) / (2 * (self.wheel_separation + self.wheel_separation_length) )
		# print("x: {}, y: {}, theta: {}".format(self.dx, self.dy, self.dtheta))

		new_robot_pose = Pose2D()
		new_robot_pose.x = prev_robot_pose.x + (self.dx * math.cos(prev_robot_pose.theta) - self.dx * math.sin(prev_robot_pose.theta))
		new_robot_pose.y = prev_robot_pose.y + (self.dy * math.cos(prev_robot_pose.theta) + self.dy * math.sin(prev_robot_pose.theta))
		new_robot_pose.theta = (prev_robot_pose.theta + self.dtheta) % (2 * 3.14)
		print("x: {}, y: {}, theta: {}".format(new_robot_pose.x, new_robot_pose.y, new_robot_pose.theta ))

		self.robot_pose.x = new_robot_pose.x
		self.robot_pose.y = new_robot_pose.y
		self.robot_pose.theta = new_robot_pose.theta

		# self.pose.xVel = deltaXTravel / deltaTime if deltaTime > 0 else 0.
        # self.pose.yVel = deltaYTravel / deltaTime if deltaTime > 0 else 0.
        # self.pose.thetaVel = deltaTheta / deltaTime if deltaTime > 0 else 0.

		#	publish odometry data
		odom = Odometry()
		odom.header.stamp = self.get_clock().now().to_msg()
		odom.header.frame_id = "odom"							# frame แม่
		odom.child_frame_id = "base_link"
		odom.pose.pose.position.x = self.robot_pose.x
		odom.pose.pose.position.y = self.robot_pose.y
		odom.pose.pose.position.z = 0.0							# assume ว่าไม่ได้เปลี่ยนระดับ

		quat = self.quaternion_from_euler(0, 0, self.robot_pose.theta)	# เพราะการเคลื่อนที่แบบ 2 มิติ ค่า roll pitch เป็น ศูนย์
		odom.pose.pose.orientation.w = quat[0]
		odom.pose.pose.orientation.x = quat[1]
		odom.pose.pose.orientation.y = quat[2]
		odom.pose.pose.orientation.z = quat[3]

		self.odom_pub.publish(odom)

		t = TransformStamped()
		t.header.stamp = self.get_clock().now().to_msg()
		t.header.frame_id = "odom"
		t.child_frame_id = "base_link"
		t.transform.translation.x = self.robot_pose.x
		t.transform.translation.y = self.robot_pose.y
		t.transform.translation.z = 0.0
		t.transform.rotation.w = quat[0]
		t.transform.rotation.x = quat[1]
		t.transform.rotation.y = quat[2]
		t.transform.rotation.z = quat[3]
		self.tf_broadcaster.sendTransform(t)

		self.last_time = ts

	def quaternion_from_euler(self, roll, pitch, yaw):
		cy = math.cos(yaw*0.5)
		sy = math.sin(yaw*0.5)
		cp = math.cos(pitch*0.5)
		sp = math.sin(pitch*0.5)
		cr = math.cos(roll*0.5)
		sr = math.sin(roll*0.5)
		q = [0]*4
		q[0] = cy * cp * cr + sy * sp * sr
		q[1] = cy * cp * sr - sy * sp * cr
		q[2] = sy * cp * sr + cy * sp * cr
		q[3] = sy * cp * cr - cy * sp * sr
		return q

def main():
	rclpy.init()
	rb = Robot()
	rclpy.spin(rb)

	rclpy.shutdown()
	del rb				# delete to avoid conflicts in other programs

if __name__ == "__main__":
	main()
