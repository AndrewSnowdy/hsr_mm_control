#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>


#include <visualization_msgs/msg/marker_array.hpp>

#include <cmath>
#include <memory>

// Minimal states
enum class SimpleState { MANUAL, APPROACH, PRE_PRESS, PRESS, DONE, EXIT, RETRACT};

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
    geometry_msgs::msg::Pose compute_standoff_goal(double bx, double by, double rx, double ry, double r_standoff);

private:
    // --- state ---
    SimpleState simple_state_{SimpleState::MANUAL};

    // --- pubs ---
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr mode_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr target_pub_;

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;

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
    std::string map_frame_{"odom"};
    std::string odom_frame_{"odom"};
    std::string base_frame_{"base_link"};
    std::string ee_frame_{"hand_palm_link"};


    // --- Logic Control ---
    bool goal_captured_{false};

    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr mission_sub_;

    // Variables to store the 'snapped' goal
    double goal_x{0.0};
    double goal_y{0.0};
    double goal_z{0.0};



    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_;
    nav_msgs::msg::OccupancyGrid latest_costmap_;
    bool have_costmap_ = false;

    // Cache the chosen standoff so it doesn't jump each tick
    geometry_msgs::msg::Pose cached_standoff_;
    bool have_standoff_ = false;

    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_array_pub_;
    std::vector<geometry_msgs::msg::Pose> debug_feasible_poses_;
    void publish_feasible_cloud(const std::vector<geometry_msgs::msg::Pose>& poses);
};
