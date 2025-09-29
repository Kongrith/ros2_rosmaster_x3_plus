import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class MoveForwardNode(Node):
    def __init__(self):
        super().__init__('move_forward_node')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.get_logger().info("Node has been started, moving the robot forward.")

    def move_forward(self, distance, speed):
        # คำนวณเวลาที่ต้องวิ่ง (time = distance / speed)
        time_to_run = distance / speed

        # สร้างข้อความ Twist
        twist = Twist()
        twist.linear.x = speed  # ตั้งค่าความเร็วเชิงเส้น
        twist.angular.z = 0.0   # ไม่มีการหมุน

        # ส่งคำสั่งวิ่ง
        start_time = time.time()
        while (time.time() - start_time) < time_to_run:
            self.publisher_.publish(twist)
            time.sleep(0.1)  # ส่งคำสั่งทุก 0.1 วินาที

        # หยุดหุ่นยนต์
        twist.linear.x = 0.0
        self.publisher_.publish(twist)
        self.get_logger().info("Robot has stopped.")

def main(args=None):
    rclpy.init(args=args)
    node = MoveForwardNode()

    try:
        # ให้หุ่นยนต์วิ่งไปข้างหน้า 2 เมตร ด้วยความเร็ว 0.2 m/s
        node.move_forward(distance=2.0, speed=0.2)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()