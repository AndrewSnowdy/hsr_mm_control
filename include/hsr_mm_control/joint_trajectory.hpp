#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <Eigen/Dense>
#include <vector>
#include <string>
#include <map>

/**
 * Computes a 5th-order polynomial for smooth motion
 */

class QuinticSpline {
    public:
    QuinticSpline() = default;

    void solve(double q0, double qf, double v0, double vf, double a0, double af, double T);

    double get_pos(double t) const;
    double get_vel(double t) const;

    private:
    Eigen::VectorXd a_ = Eigen::VectorXd::Zero(6);
    double T_ = 1.0;
};

class JointTrajectoryController : public rclcpp::Node {
public:
    JointTrajectoryController();

private:
    void on_goal_recieved(const sensor_msgs::msg::JointState::SharedPtr msg);
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void timer_callback();

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr goal_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr arm_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr base_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::map<std::string, QuinticSpline> splines_;
    std::vector<std::string> arm_joints_;
    double current_yaw_ = 0.0;
    rclcpp::Time start_time_;
    bool is_executing_ = false;
    double duration_ = 4.0;
};