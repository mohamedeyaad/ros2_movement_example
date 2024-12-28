import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class MoverNode(Node):
    def __init__(self):
        super().__init__('mover_node')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.move_robot)  # 10 Hz timer
        self.get_logger().info("MoverNode has been started.")

    def move_robot(self):
        twist = Twist()
        # Example: Move the robot forward
        twist.linear.x = 0.5  # Adjust speed
        twist.angular.z = 0.2  # Adjust turning rate
        self.publisher.publish(twist)
        self.get_logger().info(f"Published: linear.x = {twist.linear.x}, angular.z = {twist.angular.z}")

def main(args=None):
    rclpy.init(args=args)
    node = MoverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Node stopped by user.')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
