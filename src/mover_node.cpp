/**
 * @file mover_node.cpp
 * @brief ROS 2 Node for controlling a robot's movement by publishing velocity commands and subscribing to odometry data.
 *
 * This node allows the user to input linear and angular velocities via the terminal, 
 * publishes these commands to the `/cmd_vel` topic, and logs odometry data from the `/odom` topic.
 *
 * @author Mohamed Eyad
 * @date March 19, 2025
 */

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <iostream>
#include <chrono>
#include <thread>
#include <string>

/**
 * @class RobotMover
 * @brief A ROS 2 Node for controlling a robot's movement.
 *
 * The RobotMover class provides functionality to:
 * - Publish velocity commands to the `/cmd_vel` topic.
 * - Subscribe to the `/odom` topic to log odometry data.
 * - Handle user input for controlling the robot's movement.
 */
class RobotMover : public rclcpp::Node {
public:
    /**
     * @brief Constructor for the RobotMover class.
     *
     * Initializes the ROS 2 node, creates a publisher for the `/cmd_vel` topic, 
     * and a subscriber for the `/odom` topic.
     */
    RobotMover();

    /**
     * @brief Main loop for the RobotMover node.
     *
     * Continuously prompts the user for linear and angular velocities, publishes 
     * these commands, and processes callbacks for odometry data.
     */
    void run();

private:
    /**
     * @brief Callback function for the `/odom` topic.
     *
     * Logs the robot's position and orientation from the odometry data.
     *
     * @param msg Shared pointer to the received odometry message.
     */
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);

    /**
     * @brief Publishes velocity commands to the `/cmd_vel` topic.
     *
     * @param linear The linear velocity to publish.
     * @param angular The angular velocity to publish.
     */
    void publish_velocity(double linear, double angular);

    /**
     * @brief Stops the robot by publishing zero velocities.
     */
    void stop_robot();

    /**
     * @brief Handles invalid user input.
     *
     * Clears the input stream and prompts the user to enter valid numeric values.
     */
    void handle_invalid_input();

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_; ///< Publisher for velocity commands.
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_; ///< Subscriber for odometry data.
};

RobotMover::RobotMover() : Node("robot_mover") {
    // Create publisher and subscriber
    publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "odom", 10, std::bind(&RobotMover::odom_callback, this, std::placeholders::_1));
}

void RobotMover::run() {
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

void RobotMover::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    // Log odometry data
    auto position = msg->pose.pose.position;
    auto orientation = msg->pose.pose.orientation;
    RCLCPP_INFO(this->get_logger(),
        "Odom: Position=(%.2f, %.2f), Orientation Z=%.2f",
        position.x, position.y, orientation.z);
}

void RobotMover::publish_velocity(double linear, double angular) {
    auto message = geometry_msgs::msg::Twist();
    message.linear.x = linear;
    message.angular.z = angular;
    publisher_->publish(message);
    RCLCPP_INFO(this->get_logger(), "Publishing: Linear=%.2f, Angular=%.2f", linear, angular);
}

void RobotMover::stop_robot() {
    auto message = geometry_msgs::msg::Twist();
    message.linear.x = 0.0;
    message.angular.z = 0.0;
    publisher_->publish(message);
    RCLCPP_INFO(this->get_logger(), "Robot stopped.");
}

void RobotMover::handle_invalid_input() {
    std::cerr << "Invalid input. Please enter a numeric value." << std::endl;
    std::cin.clear(); // Clear the error flag
    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n'); // Discard invalid input
}

/**
 * @brief Main function for the RobotMover node.
 *
 * Initializes the ROS 2 system, creates an instance of the RobotMover node, 
 * and starts its main loop.
 *
 * @param argc Argument count.
 * @param argv Argument vector.
 * @return int Exit code.
 */
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
