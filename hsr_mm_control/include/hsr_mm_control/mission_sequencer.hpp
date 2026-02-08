#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <cmath>
#include <memory>

// Minimal states
enum class SimpleState { APPROACH, PRE_PRESS, PRESS, DONE, EXIT, RETRACT};

class MissionSequencer : public rclcpp::Node {
public:
    MissionSequencer();

private:
    void simple_timer();

    // --- TF helpers ---
    bool get_tf_xyz(const std::string& parent,
                    const std::string& child,
                    double &x, double &y, double &z);
    bool get_base_yaw(double &yaw);

    bool base_close_xyw(double tx, double ty, double tw, double tol_xy, double tol_w);
    bool ee_close_xyz(double tx, double ty, double tz, double tol_xyz);

private:
    // --- state ---
    SimpleState simple_state_{SimpleState::PRE_PRESS};

    // --- pubs ---
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr mode_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr target_pub_;

    // --- timer ---
    rclcpp::TimerBase::SharedPtr timer_;

    // --- TF ---
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    // --- button location ---
    double button_x{2.44};
    double button_y{0.0};
    double button_z{1.0};

    // --- frame names ---
    std::string odom_frame_{"odom"};
    std::string base_frame_{"base_link"};
    std::string ee_frame_{"hand_palm_link"};
};
