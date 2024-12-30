import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class RobotMover(Node):
    def __init__(self):
        super().__init__('robot_mover')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.move_robot)
        self.cmd = Twist()

    def move_robot(self):
        self.publisher_.publish(self.cmd)
        self.get_logger().info(f'Publishing: Linear={self.cmd.linear.x}, Angular={self.cmd.angular.z}')
        time.sleep(1)
        self.cmd = Twist()  # Stop the robot
        self.publisher_.publish(self.cmd)

def main(args=None):
    rclpy.init(args=args)
    robot_mover = RobotMover()

    try:
        while rclpy.ok():
            try:
                linear_vel = float(input("Enter linear velocity: "))
                angular_vel = float(input("Enter angular velocity: "))
                robot_mover.cmd.linear.x = linear_vel
                robot_mover.cmd.angular.z = angular_vel
                rclpy.spin_once(robot_mover)
            except ValueError:
                robot_mover.get_logger().error("Invalid input. Please enter numeric values.")
    except KeyboardInterrupt:
        pass

    robot_mover.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
