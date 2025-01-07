import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import time

class RobotMover(Node):
    def __init__(self):
        super().__init__('robot_mover')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.subscription = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        self.cmd = Twist()

    def odom_callback(self, msg):
        """Prints odometry data."""
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        self.get_logger().info(f'Odom: Position=({position.x:.2f}, {position.y:.2f}), Orientation Z={orientation.z:.2f}')

    def move_robot(self, linear_vel, angular_vel):
        """Moves the robot for 1 second."""
        self.cmd.linear.x = linear_vel
        self.cmd.angular.z = angular_vel
        self.publisher_.publish(self.cmd)
        self.get_logger().info(f'Publishing: Linear={linear_vel}, Angular={angular_vel}')
        
        start_time = time.time()
        while time.time() - start_time < 1.0:
            rclpy.spin_once(self)  # Process callbacks

        # Stop the robot
        self.cmd = Twist()
        self.publisher_.publish(self.cmd)
        self.get_logger().info('Robot stopped.')


def main(args=None):
    rclpy.init(args=args)
    robot_mover = RobotMover()

    try:
        while rclpy.ok():
            try:
                # Get user input for velocities
                linear_vel = float(input("Enter linear velocity: "))
                angular_vel = float(input("Enter angular velocity: "))
                
                # Move the robot with the provided velocities
                robot_mover.move_robot(linear_vel, angular_vel)
            except ValueError:
                robot_mover.get_logger().error("Invalid input. Please enter numeric values.")
            except KeyboardInterrupt:
                break
    except KeyboardInterrupt:
        pass
    finally:
        robot_mover.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
