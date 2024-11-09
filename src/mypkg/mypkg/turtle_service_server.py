import rclpy
from rclpy.node import Node
from rclpy import qos

from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

from std_srvs.srv import SetBool

class turtleControl(Node):
	def __init__(self):
		super().__init__("turtle_avoidance")
		self.vel_pub = self.create_publisher(Twist, "turtle1/cmd_vel", 10)
		self.pose_sub = self.create_subscription(Pose, "turtle1/pose",
				self.pose_callback, qos_profile =qos.qos_profile_sensor_data)	# ถ้ามีค่าส่งมา ... ... ...

		self.vel_timer = self.create_timer(1, self.vel_timer_callback)
		self.x = 0.0
		self.y = 0.0
		self.v = 0.0
		self.w = 0.0

		# service server
		self.srv = self.create_service(SetBool, 'start_stop_service', self.srv_callback)
		self.trigger = 0

	def srv_callback(self, request, response):
		print("request is: ", request.data)

		if request.data:
			print("turtle start moving")
			self.trigger = 1
			response.success = True
			response.message = "your turtle moving"
		else:
			print("Trying to Stop")
			self.trigger = 0
			response.success = True
			response.message = "your turtle stopped"
		return response

	def pose_callback(self, pose_in):
		self.x = pose_in.x
		self.y = pose_in.y
		print("X: {}, Y: {}".format(self.x, self.y))

		if self.trigger == 1:
			self.collision_check()
		else:
			self.v = 0.0
			self.w = 0.0

	def collision_check(self):
		if (self.x <= 2 or self.x >= 8 or self.y <=2 or self.y >= 8):
			self.v = 0.3
			self.w = 1.0
		else:
			self.v = 0.5
			self.w = 0.0

	def vel_timer_callback(self):
		cmd_vel = Twist()
		cmd_vel.linear.x = self.v
		cmd_vel.angular.z = self.w
		self.vel_pub.publish(cmd_vel)

def main():
	rclpy.init()
	turtle_control = turtleControl()

	rclpy.spin(turtle_control)
	rclpy.shutdown()

if __name__ == "__main__":
	main()
