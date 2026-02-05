#pragma once

#include <rclcpp/rclcpp.hpp>
#include <Eigen/Dense>
#include <unordered_map>
#include <map>
#include <string>
#include <vector>
#include <memory>

// Pinocchio
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>

// ROS Messages
#include <sensor_msgs/msg/joint_state.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <std_msgs/msg/bool.hpp>
#include <tf2_ros/transform_broadcaster.h>

enum class ControlState {IDLE, PLANNING, REACHED};

class FinalPoseNode : public rclcpp::Node
{
public:
    explicit FinalPoseNode();

private:
    // --- ROS Callbacks & Main Loop ---
    void tick();
    
    // --- Analytical Brain (IK) ---
    Eigen::VectorXd solveGlobalIK(const Eigen::Vector3d& target_p);
    
    // --- Helpers ---
    void loadPinocchioModel();
    bool haveArmState() const;
    bool haveBaseState() const;
    void publishGhostPose(const Eigen::VectorXd &q_goal);

    // --- ROS Interface ---
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr target_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr mode_sub_;
    
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr ghost_joint_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    // --- Target Tracking ---
    geometry_msgs::msg::Pose current_target_;
    geometry_msgs::msg::Pose last_solved_target_;
    bool has_target_ = false;
    bool use_ik_mode_ = false; 
    const double TARGET_THRESHOLD = 0.01; // 1cm threshold for re-solving

    // --- Robot State & Pinocchio ---
    pinocchio::Model model_;
    std::unique_ptr<pinocchio::Data> data_;
    pinocchio::FrameIndex ee_fid_;
    bool model_ready_ = false;

    // Joint Mapping
    std::unordered_map<std::string, double> q_map_;
    std::map<std::string, int> joint_name_to_id_;
    std::unordered_map<std::string, int> arm_name_to_qidx_;
    std::vector<std::string> arm_joint_names_;

    // Base State
    double base_pos_x_ = 0.0;
    double base_pos_y_ = 0.0;
    double base_yaw_ = 0.0;
    bool odom_received_ = false;
    bool joints_received_ = false;

    // Ghost Publishing Layout
    std::vector<std::string> ghost_joint_names_;
    std::vector<double> ghost_joint_pos_;
    std::unordered_map<std::string, size_t> ghost_name_to_i_;

    Eigen::VectorXd q_goal_;
};