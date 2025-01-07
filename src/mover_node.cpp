#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <iostream>
#include <chrono>
#include <thread>
#include <string>

class RobotMover : public rclcpp::Node {
public:
    RobotMover() : Node("robot_mover") {
        // Create publisher and subscriber
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
        subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "odom", 10, std::bind(&RobotMover::odom_callback, this, std::placeholders::_1));
    }

    void run() {
        while (rclcpp::ok()) {
            double linear_velocity, angular_velocity;

            // Get user input
            std::cout << "Enter linear velocity: ";
            if (!(std::cin >> linear_velocity)) {
                handle_invalid_input();
                continue;
            }

            std::cout << "Enter angular velocity: ";
            if (!(std::cin >> angular_velocity)) {
                handle_invalid_input();
                continue;
            }

            // Publish velocity commands
            publish_velocity(linear_velocity, angular_velocity);

            // Move robot for 1 second
            auto start_time = std::chrono::steady_clock::now();
            while (std::chrono::steady_clock::now() - start_time < std::chrono::seconds(1)) {
                rclcpp::spin_some(this->get_node_base_interface()); // Process callbacks 
            }

            // Stop the robot
            stop_robot();
        }
    }

private:
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        // Log odometry data
        auto position = msg->pose.pose.position;
        auto orientation = msg->pose.pose.orientation;
        RCLCPP_INFO(this->get_logger(),
            "Odom: Position=(%.2f, %.2f), Orientation Z=%.2f",
            position.x, position.y, orientation.z);
    }

    void publish_velocity(double linear, double angular) {
        auto message = geometry_msgs::msg::Twist();
        message.linear.x = linear;
        message.angular.z = angular;
        publisher_->publish(message);
        RCLCPP_INFO(this->get_logger(), "Publishing: Linear=%.2f, Angular=%.2f", linear, angular);
    }

    void stop_robot() {
        auto message = geometry_msgs::msg::Twist();
        message.linear.x = 0.0;
        message.angular.z = 0.0;
        publisher_->publish(message);
        RCLCPP_INFO(this->get_logger(), "Robot stopped.");
    }

    void handle_invalid_input() {
        std::cerr << "Invalid input. Please enter a numeric value." << std::endl;
        std::cin.clear(); // Clear the error flag
        std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n'); // Discard invalid input
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto robot_mover = std::make_shared<RobotMover>();

    try {
        robot_mover->run();
    } catch (const std::exception &e) {
        RCLCPP_ERROR(robot_mover->get_logger(), "Exception: %s", e.what());
    }

    rclcpp::shutdown();
    return 0;
}
