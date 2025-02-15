import rclpy						# Python Client Library for ROS 2
from rclpy.node import Node			# Handles the creation of nodes
from sensor_msgs.msg import Image	# Image is the message type
from cv_bridge import CvBridge		# Package to convert between ROS and OpenCV Images
import cv2							# OpenCV library

from rclpy.time import Time


class ImageSubscriber(Node):
	def __init__(self):
		super().__init__('image_subscriber')
		# self.publisher_ = self.create_publisher(Image, '/rgb/image', 10)

		self.subscription = self.create_subscription(Image, '/rgb/image',  self.timer_callback,  10)
		self.subscription # prevent unused variable warning

		# Used to convert between ROS and OpenCV images
		self.br = CvBridge()

	def timer_callback(self, data):
		"""
		Callback function.
		"""
		# Display the message on the console
		# self.get_logger().info('Receiving video frame')

		# Convert ROS Image message to OpenCV image
		current_frame = self.br.imgmsg_to_cv2(data)

		# Display image
		cv2.imshow("camera", current_frame)
		cv2.waitKey(1)

def main():
	rclpy.init()
	image_subscriber  = ImageSubscriber()		# Create the node
	rclpy.spin(image_subscriber )

	image_subscriber .destroy_node()
	rclpy.shutdown()

if __name__ == "__main__":
	main()
