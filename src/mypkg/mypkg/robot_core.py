import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Pose2D, Twist, TransformStamped
from std_msgs.msg import Float32, Int32
from nav_msgs.msg import Odometry	#

import tf2_ros


import math

class Robot(Node):
	def __init__(self):
		super().__init__('robot_core_node')
		self.wheel_radius = 0.06096 # 0.06096 m
		self.wheelbase = 0.25 # 0.225m

		self.max_rpm = 60.0 #rpm

		self.vel_sub = self.create_subscription(Twist, 'cmd_vel', self.vel_callback,10)

		self.leftwheel_pub = self.create_publisher(Float32, 'wheel_command_left', 10)
		self.rightwheel_pub = self.create_publisher(Float32, 'wheel_command_right', 10)

		self.v = 0.0
		self.w = 0.0
		self.left_tick = {"bf": None, "now": None}
		self.right_tick = {"bf": None, "now": None}
		self.left_tick_ts = None
		self.right_tick_ts = None
		self.timer = self.create_timer(0.1, self.timer_callback)

		self.cmd_l = Float32()
		self.cmd_r = Float32()

		# queue size
		self.left_tick_sub = self.create_subscription(Int32, 'left_tick', self.left_tick_callback, 10)
		self.right_tick_sub = self.create_subscription(Int32, 'right_tick', self.right_tick_callback, 10)

		self.last_time = self.get_clock().now()

		# odometry
		self.tick_per_rev = 2800
		self.robot_pose	= Pose2D()						# x, y, theta
		self.dx = 0.0
		self.dy = 0.0
		self.dtheta = 0.0

		self.odom_pub = self.create_publisher(Odometry, "odom", 10)

		# TF
		self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

		self.get_logger().info("robot core is running")

	def left_tick_callback(self, tick_in):
		self.left_tick['now'] = tick_in.data
		self.left_tick_ts = self.get_clock().now()		# function สำหรับ ros2

	def right_tick_callback(self, tick_in):
		self.right_tick['now'] = tick_in.data
		self.right_tick_ts = self.get_clock().now()

	def get_tick(self):
		tick = {}
		tick["l"] = self.left_tick["now"]
		tick["r"] = self.right_tick["now"]
		tick["lts"] = self.left_tick_ts					# lst = left timestamp
		tick["rts"] = self.right_tick_ts
		return tick

	def vel_callback(self, vel):
		print("callback function")
		self.v = vel.linear.x
		self.w = vel.angular.z

		L = self.wheelbase
		R = self.wheel_radius

		v_r = ((2.0*self.v)+(self.w*L))/(2.0*R)		# radius/sec
		v_l = ((2.0*self.v)-(self.w*L))/(2.0*R)		# radius/sec

		self.wheel_speed_setpoint(v_r,v_l)
		# self.car.set_car_motion(vx, vy, angular)
	def timer_callback(self):
		self.leftwheel_pub.publish(self.cmd_l)
		self.rightwheel_pub.publish(self.cmd_r)
		self.odometry()

	def wheel_speed_setpoint(self,vr,vl):
		rpm_r = vr*9.549297
		rpm_l = vl*9.549297  #1 rad/s = 9.549297 rpm

		rpm_r = max(min(rpm_r,self.max_rpm), -self.max_rpm)
		rpm_l = max(min(rpm_l,self.max_rpm), -self.max_rpm)

		if rpm_r == 0.0:
			self.cmd_r.data = 0.0
		if rpm_l == 0.0:
			self.cmd_l.data = 0.0
		self.cmd_r.data = rpm_r
		self.cmd_l.data = rpm_l

		self.get_logger().info('out left rpm: %s' %rpm_l)
		self.get_logger().info('out right rpm: %s' %rpm_r)

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

	def odometry(self):
		ts = self.get_clock().now()
		tick = self.get_tick()
		dt = ts - self.last_time		# หน่วยจะเป็น nano second
		dt = dt.nanoseconds*1e-9		# แปลงหน่วยเป็นวินาที

		if tick['l'] == None or tick['r'] == None:
			return

		if self.left_tick["bf"] == None or self.right_tick["bf"]:
			self.left_tick["bf"] = tick["l"]
			self.right_tick["bf"] = tick["r"]

		prev_robot_pose = self.robot_pose				# Pose2D
		R = self.wheel_radius
		L = self.wheelbase
		TPR = self.tick_per_rev							# TPR: Tick per Revolution

		DPT = (2 * math.pi * R) / TPR					# Distance Per Tick
		DL = DPT * (tick['l'] - self.left_tick["bf"])
		DR = DPT * (tick['r'] - self.right_tick["bf"])

		DC = (DL + DR) * 0.5							# DC: Distance Center (เทียบตัวหุ่นเอง)
		self.dx = DC*math.cos(prev_robot_pose.theta)	# ตัว theta จะเป็นมุม yaw
		self.dy = DC*math.sin(prev_robot_pose.theta)	#
		self.dtheta = (DR - DL) / L						# มาจากสมการ kinematics ของ diff drive

		new_robot_pose = Pose2D()
		new_robot_pose.x = prev_robot_pose.x + self.dx
		new_robot_pose.y = prev_robot_pose.y + self.dy
		new_robot_pose.theta = prev_robot_pose.theta + self.dtheta

		self.robot_pose.x = new_robot_pose.x
		self.robot_pose.y = new_robot_pose.y
		self.robot_pose.theta = new_robot_pose.theta

		self.left_tick["bf"] = tick['l']
		self.right_tick["bf"] = tick['r']

		# publish odometry data
		odom = Odometry()
		odom.header.stamp = self.get_clock().now().to_msg()
		odom.header.frame_id = "odom"							# frame แม่
		odom.child_frame_id = "base_link"
		odom.pose.pose.position.x = self.robot_pose.x
		odom.pose.pose.position.y = self.robot_pose.y
		odom.pose.pose.position.z = 0.0							# assume ว่าไม่ได้เปลี่ยนระดับ

		quat = self.quaternion_from_euler(0, 0, self.robot_pose.theta)	# เพราะการเคลื่อนที่แบบ 2 มิติ ค่า roll pitแ้ เป็น ศูนย์
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

def main():
	rclpy.init()

	rb = Robot()
	rclpy.spin(rb)

	rclpy.shutdown()

if __name__ == "__main__":
	main()
