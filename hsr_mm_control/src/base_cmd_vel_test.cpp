#include <chrono>
#include <memory>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

using namespace std::chrono_literals;

class SimpleMover : public rclcpp::Node {
public:
    SimpleMover() : Node("simple_mover") {
        // HSR-specific velocity topic
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/omni_base_controller/cmd_vel", 10);
        subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10, std::bind(&SimpleMover::odom_callback, this, std::placeholders::_1));
        
        RCLCPP_INFO(this->get_logger(), "Moving to (5.0, 5.0)...");
    }

private:
    double wrapAngle(double a) {
        while (a > M_PI) a -= 2.0 * M_PI;
        while (a < -M_PI) a += 2.0 * M_PI;
        return a;
        }

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        double curr_x = msg->pose.pose.position.x;
        double curr_y = msg->pose.pose.position.y;

        // Get Heading (Yaw)
        tf2::Quaternion q(
            msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
        double r, p, yaw;
        tf2::Matrix3x3(q).getRPY(r, p, yaw);

        // Target
        double goal_x = 1.0;
        double goal_y = 1.0;
        double goal_yaw = 1.0;


        // Errors in World Frame
        double err_x = goal_x - curr_x;
        double err_y = goal_y - curr_y;
        double yaw_error = wrapAngle(goal_yaw - yaw);
        double distance = std::sqrt(err_x * err_x + err_y * err_y);

        auto cmd = geometry_msgs::msg::Twist();

        if (distance > 0.05) { // Stop within 5cm
            double kp = 0.4;
            // Transform World Error to Robot Local Velocity
            // x_vel drives forward/backward, y_vel drives left/right

            cmd.linear.x = kp * (err_x * std::cos(yaw) + err_y * std::sin(yaw));
            cmd.linear.y = kp * (-err_x * std::sin(yaw) + err_y * std::cos(yaw));
        } else {
                cmd.linear.x = 0.0;
                cmd.linear.y = 0.0;
        }

        if (std::abs(yaw_error) > 0.05) {
            double k_theta = 1.0;
            cmd.angular.z = k_theta * yaw_error;
        } else {
            cmd.angular.z = 0.0;
        }

        publisher_->publish(cmd);
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SimpleMover>());
    rclcpp::shutdown();
    return 0;
}