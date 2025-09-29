import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

import time

class TurnRobot(Node):
    def __init__(self):
        super().__init__('turn_robot')
        # self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.get_logger().info("Turning robot 90 degrees left...")

    def turn_left_90(self):
        # Initialize a Twist message
        twist = Twist()
        
        # Set angular velocity for turning left
        twist.linear.x = 1.0
        twist.linear.y = 1.0
        twist.angular.z = 0.5  # Adjust based on robot's turning speed
        
        # Time to turn 90 degrees
        # turn_duration = 90 / (0.5 * 180 / 3.14159)  # Adjust for radians/sec
        turn_duration = 100
        # Start turning
        start_time = time.time()
        while time.time() - start_time < turn_duration:
            self.publisher.publish(twist)
            time.sleep(0.1)

        # Stop the robot
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.angular.z = 0.0
        self.publisher.publish(twist)
        self.get_logger().info("Turn complete!")


def main(args=None):
    rclpy.init(args=args)
    turn_robot = TurnRobot()
    try:
        turn_robot.turn_left_90()
    except KeyboardInterrupt:
        pass
    finally:
        turn_robot.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()