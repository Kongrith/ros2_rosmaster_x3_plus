#!/usr/bin/env python3

import rclpy
from rclpy.node import Node 	# ดึง class ของ Node เข้ามา
from std_msgs.msg import Int64
from rclpy import qos
import random

class NumberNode(Node):
	def __init__(self):														# initialize claas นี้
		super().__init__("number_publisher")								# การสืบทอด Node มาที่ NumberNode
		self.num_pub = self.create_publisher(Int64, "number_topic", qos_profile = qos.qos_profile_sensor_data)
		self.num_timer = self.create_timer(0.5, self.num_timer_callback)	# ทุกๆวินาทีจะทำการส่งข้อมูล

	def num_timer_callback(self):
		random1 = random.randint(1, 100)									# random ค่าตั้งแต่ 1-100
		msg = Int64()
		msg.data = random1

		self.num_pub.publish(msg)	# ส่งแบบ Int64 ที่ number_topic

def main():
	rclpy.init()
	node = NumberNode()

	rclpy.spin(node)				# โปรแกรมทำงานเรื่อยๆจนกว่าจะปิด node
	rclpy.shutdown()

if __name__ == "__main__":
	main()
