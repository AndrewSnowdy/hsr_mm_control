#ifndef MISSION_SEQUENCER_HPP
#define MISSION_SEQUENCER_HPP

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <chrono>

enum class MissionState { IDLE, APPROACH, READY, PRESS, RETRACT, DONE };

class MissionSequencer : public rclcpp::Node {
public:
    MissionSequencer();

private:
    // Main Logic
    void state_machine_timer();
    bool is_at_goal(double tx, double ty, double tol);
    bool wait_for(double seconds);

    // ROS 2 Infrastructure
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr target_pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // State & Data
    MissionState state_;
    double current_x_{0.0};
    double current_y_{0.0};
    
    // Hardcoded Targets
    double button_x, button_y, button_z;

    // Timing helper
    rclcpp::Time start_time_;
    bool timer_started_{false};
};

#endif