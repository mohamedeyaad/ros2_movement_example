#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <iostream>
#include <string>
#include <chrono>
#include <thread>

class RobotMover : public rclcpp::Node {
public:
    RobotMover() : Node("robot_mover") {
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&RobotMover::stop_robot, this)
        );
        timer_->cancel(); // Start with the timer canceled
        get_user_input();
    }

private:
    void get_user_input() {
        while (rclcpp::ok()) {
            double linear_velocity, angular_velocity;
            std::cout << "Enter linear velocity: ";
            if (!(std::cin >> linear_velocity)) {
                std::cerr << "Invalid input. Please enter a number." << std::endl;
                std::cin.clear(); // clear the error flag
                std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n'); // discard invalid input
                continue;
            }

            std::cout << "Enter angular velocity: ";
            if (!(std::cin >> angular_velocity)) {
                std::cerr << "Invalid input. Please enter a number." << std::endl;
                std::cin.clear();
                std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
                continue;
            }

            publish_velocity(linear_velocity, angular_velocity);
            std::this_thread::sleep_for(std::chrono::seconds(1));
            stop_robot();
        }
    }

    void publish_velocity(double linear, double angular) {
        auto message = geometry_msgs::msg::Twist();
        message.linear.x = linear;
        message.angular.z = angular;
        publisher_->publish(message);
        timer_->reset(); // Start the timer to stop the robot after 1 second
    }

    void stop_robot() {
        auto message = geometry_msgs::msg::Twist();
        message.linear.x = 0.0;
        message.angular.z = 0.0;
        publisher_->publish(message);
        timer_->cancel(); // Stop the timer
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RobotMover>());
    rclcpp::shutdown();
    return 0;
}
