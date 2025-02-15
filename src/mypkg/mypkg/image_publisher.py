import rclpy						# Python Client Library for ROS 2
from rclpy.node import Node			# Handles the creation of nodes
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge		# Package to convert between ROS and OpenCV Images
import cv2							# OpenCV library

from rclpy.time import Time

import base64
from std_msgs.msg import String

class ImagePublisher(Node):
	def __init__(self):
		super().__init__('image_publisher')
		self.pub_raw_img = self.create_publisher(Image, '/rgb/image', 10)
		self.pub_compressed_img = self.create_publisher(CompressedImage, '/rgb/compressed', 10)

		period = 0.1
		self.timer = self.create_timer(period, self.timer_callback)		# Create the timer

		self.cap = cv2.VideoCapture(0)
		self.cv_bridge = CvBridge()										# Used to convert between ROS and OpenCV images

	def timer_callback(self):
		"""
		Callback function.
		This function gets called every 0.1 seconds.
		"""

		ret, frame = self.cap.read()
		if ret == True:
			# Opencv processing image
			frame = cv2.resize(frame, (640, 480))
			frame = cv2.flip(frame, 0)										# 0 flip vertically
			self.pub_raw_img.publish(self.cv_bridge.cv2_to_imgmsg(frame, "bgr8"))
			self.pub_compressed_img.publish(self.cv_bridge.cv2_to_compressed_imgmsg(frame))

def main():
	rclpy.init()
	image_publisher = ImagePublisher()		# Create the node
	rclpy.spin(image_publisher)

	image_publisher.destroy_node()
	rclpy.shutdown()

if __name__ == "__main__":
	main()
