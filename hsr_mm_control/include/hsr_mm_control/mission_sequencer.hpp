#pragma once

#include <memory>
#include <cmath>
#include <vector>
#include <optional>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "hsr_mm_control/msg/mission_goal.hpp"

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// State Definitions
enum class SimpleState { 
    MANUAL, 
    APPROACH, 
    PRE_PRESS, 
    PRESS, 
    RETRACT, 
    EXIT, 
    DONE 
};

struct RobotState {
    double x, y, z;
    double yaw;
    geometry_msgs::msg::Pose pose;
};

struct DoorInfo {
    geometry_msgs::msg::Pose center;
    geometry_msgs::msg::Pose pillar;
};

class MissionSequencer : public rclcpp::Node {
public:
    MissionSequencer();

private:
    // --- Core FSM ---
    void simple_timer();
    SimpleState simple_state_{SimpleState::MANUAL};

    // --- Mission Goal Helper ---
    rclcpp::Publisher<hsr_mm_control::msg::MissionGoal>::SharedPtr mission_pub_;
    void publish_mission_goal(const geometry_msgs::msg::Pose& pose, bool ik_mode, double cruise_speed, double door_yaw);

    // --- Math & Checking Helpers ---
    bool base_close_xyw(const RobotState& state, const geometry_msgs::msg::Pose& target, double tol_xy, double tol_w);
    bool ee_close_xyz(const geometry_msgs::msg::Point& current_pos, double tx, double ty, double tz, double tol_xyz);
    void set_yaw(geometry_msgs::msg::Pose &pose, double yaw);

    // --- TF & Sensor Processing ---
    std::optional<RobotState> get_base_position();
    std::optional<geometry_msgs::msg::Point> get_ee_position();
    std::optional<geometry_msgs::msg::Pose> get_closest_button();
    std::optional<geometry_msgs::msg::Pose> get_door_near_button(double bx, double by);
    std::optional<DoorInfo>  get_complete_door(double bx, double by);
    
    // --- Subscriptions ---
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_;
    rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr marker_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr mission_sub_;

    // --- Publishers ---
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr mode_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr target_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr door_lock_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_array_pub_;

    // --- Robot/Environment Data ---
    nav_msgs::msg::OccupancyGrid latest_costmap_;
    visualization_msgs::msg::MarkerArray latest_markers_;
    bool have_costmap_{false};
    bool mission_locked_{false};

    // --- Mission Parameters ---
    double button_x, button_y, button_z;
    std::string odom_frame_{"odom"};
    std::string base_frame_{"base_link"};
    std::string ee_frame_{"hand_palm_link"};

    // --- Caching & Logic Control ---
    // standoff_yaw starts at -999.0 so we know it's uninitialized
    double standoff_yaw{-999.0}; 
    double approach_start_dist = -1.0;
    geometry_msgs::msg::Pose approach_start_pose;
    geometry_msgs::msg::Pose cached_standoff_;
    std::vector<geometry_msgs::msg::Pose> debug_feasible_poses_;

    // --- Infrastructure ---
    rclcpp::TimerBase::SharedPtr timer_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    // --- Future/Debug Functions (Kept as requested) ---
    
    // void publish_feasible_cloud(const std::vector<geometry_msgs::msg::Pose>& poses);
};