import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import threading


class MoverNode(Node):
    def __init__(self):
        super().__init__('mover_node')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.linear_velocity = 0.0
        self.angular_velocity = 0.0

        # Start a separate thread for the UI input
        ui_thread = threading.Thread(target=self.user_input, daemon=True)
        ui_thread.start()

        # Create a timer to send velocity commands
        self.timer = self.create_timer(0.1, self.move_robot)
        #self.get_logger().info("MoverNode with UI has been started.")

    def user_input(self):
        """Thread function to take user input."""
        while True:
            try:
                # Wait until both inputs are taken
                self.linear_velocity, self.angular_velocity = self.get_velocity_input()
            except ValueError:
                self.get_logger().error("Invalid input. Please enter numeric values.")

    def get_velocity_input(self):
        """Collect both linear and angular velocities together."""
        linear = float(input("Enter linear velocity (x): "))
        angular = float(input("Enter angular velocity (z): "))
        return linear, angular

    def move_robot(self):
        """Publish the velocity commands based on user input."""
        twist = Twist()
        twist.linear.x = self.linear_velocity
        twist.angular.z = self.angular_velocity
        self.publisher.publish(twist)
        #self.get_logger().info(f"Published: linear.x = {twist.linear.x}, angular.z = {twist.angular.z}")


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