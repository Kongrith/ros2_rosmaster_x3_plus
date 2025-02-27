#! /usr/bin/env python3
import rclpy
from rclpy.node import Node
import math

from .robot_navigator import BasicNavigator, NavigationResult
from geometry_msgs.msg import PoseStamped

class navRobot(Node):
	def __init__(self):
		super().__init__('nav2_cmd_node')
		self.get_logger().info('starting navigator')
		self.nav = BasicNavigator()

		self.nav.waitUntilNav2Active()
		self.get_logger().info('Nav2 is activated')

		# set initial pose
		self.ip = PoseStamped()
		self.set_initial_pose()
		self.nav.waitUntilNav2Active()
		self.wp = []

	def set_initial_pose(self):
		self.ip.header.frame_id = "map"
		self.ip.header.stamp = self.get_clock().now().to_msg()
		self.ip.pose.position.x = 0.0
		self.ip.pose.position.y = 0.0

		q = self.quaternion_from_euler(0.0, 0.0, 0.0)

		self.ip.pose.orientation.w = q[0]
		self.ip.pose.orientation.x = q[1]
		self.ip.pose.orientation.y = q[2]
		self.ip.pose.orientation.z = q[3]

		self.nav.setInitialPose(self.ip)


	def set_point(self, x, y, theta):
		goal_pose = PoseStamped()
		goal_pose.header.frame_id = "map"
		goal_pose.header.stamp = self.get_clock().now().to_msg()
		goal_pose.pose.position.x = x
		goal_pose.pose.position.y = y

		q = self.quaternion_from_euler(x, y, theta)

		goal_pose.pose.orientation.w = q[0]
		goal_pose.pose.orientation.x = q[1]
		goal_pose.pose.orientation.y = q[2]
		goal_pose.pose.orientation.z = q[3]

		return goal_pose

	def goto(self, x, y, theta):
		target = self.set_point(x, y, theta)
		self.nav.goToPose(target)

	def waypoint(self):
		p = self.set_point(-0.5, 0.0, 0.0)		# ตามแนวแกน x, y, theta ของ map frame
		self.wp.append(p)
		p = self.set_point(0.5, 0.0, 0.0)		# ตามแนวแกน x, y, theta ของ map frame
		self.wp.append(p)
		p = self.set_point(1.0, 0.0, 0.0)		# ตามแนวแกน x, y, theta ของ map frame
		self.wp.append(p)
		self.nav.followWaypoints(self.wp)

		# while not self.nav.isNavComplete():
		# 	print(self.nav.isNavComplete())
		# 	feedback = self.nav.getFeedback()
		# 	print(feedback)

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
	nr = navRobot()

	# x, y, theta
	# nr.goto( 0.5, 0.0, 0.0)
	nr.waypoint()

	# เราจะไม่ spin เพราะจะให้ทำงานครั้งเดียว
	nr.destroy_node()


if __name__ == "__main__":
	main()
