import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class MoverNode(Node):
    def __init__(self):
        super().__init__('mover_node')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.get_logger().info("MoverNode with UI has been started.")

    def get_user_input(self):
        """Function to get user input for velocities."""
        try:
            linear_velocity = float(input("Enter linear velocity (x): "))
            angular_velocity = float(input("Enter angular velocity (z): "))
            return linear_velocity, angular_velocity
        except ValueError:
            self.get_logger().error("Invalid input. Please enter numeric values.")
            return 0.0, 0.0

    def move_robot(self, linear_velocity, angular_velocity):
        """Publish the velocity commands based on user input."""
        twist = Twist()
        twist.linear.x = linear_velocity
        twist.angular.z = angular_velocity
        self.publisher.publish(twist)
        self.get_logger().info(f"Published: linear.x = {twist.linear.x}, angular.z = {twist.angular.z}")


def main(args=None):
    rclpy.init(args=args)
    node = MoverNode()

    try:
        while rclpy.ok():
            # Take user input in the main loop
            linear_velocity, angular_velocity = node.get_user_input()
            node.move_robot(linear_velocity, angular_velocity)
    except KeyboardInterrupt:
        node.get_logger().info('Node stopped by user.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
