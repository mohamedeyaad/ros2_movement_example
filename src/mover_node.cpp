#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

class RobotMover : public rclcpp::Node {
public:
    RobotMover() : Node("robot_mover") {
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100), 
            std::bind(&RobotMover::move_robot, this)
        );
    }

private:
    void move_robot() {
        auto message = geometry_msgs::msg::Twist();
        // Set linear and angular velocities
        message.linear.x = 0.5;  // Move forward
        message.angular.z = 0.2; // Turn slightly
        publisher_->publish(message);
        RCLCPP_INFO(this->get_logger(), "Publishing: linear.x=%.2f, angular.z=%.2f",
                    message.linear.x, message.angular.z);
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RobotMover>());
    rclcpp::shutdown();
    return 0;
}
