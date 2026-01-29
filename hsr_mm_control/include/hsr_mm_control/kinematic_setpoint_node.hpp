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
#include <geometry_msgs/msg/twist.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <tf2_ros/transform_broadcaster.h>

enum class ControlState { IDLE, PLANNING, EXECUTING, REACHED };

class FinalPoseNode : public rclcpp::Node
{
public:
    explicit FinalPoseNode();

private:
    // --- ROS Interface ---
    bool odom_received_ = false;
    bool joints_received_ = false;
    //   void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg);
    //   void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void tick();

    // --- Analytical Brain (IK) ---
    Eigen::VectorXd solveGlobalIK(const Eigen::Vector3d& target_p);
    
    // --- Movement Execution ---
    //   void executeTrajectory(double current_time);
    //   void publishCommands(const Eigen::VectorXd &q_cmd, const Eigen::VectorXd &dq_cmd);

    // --- Helpers ---
    void loadPinocchioModel();
    bool haveArmState() const;
    bool haveBaseState() const;

    void publishGhostPose(const Eigen::VectorXd &q_goal);

    // --- MEMBER VARIABLES (Declare these so the .cpp can see them) ---
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // State and Planning
    ControlState state_ = ControlState::IDLE;
    Eigen::VectorXd q_start_, q_goal_;
    double start_time_ = 0.0;
    double total_duration_ = 5.0;

    // Pinocchio Data
    pinocchio::Model model_;
    std::unique_ptr<pinocchio::Data> data_;
    pinocchio::FrameIndex ee_fid_;

    // Robot State tracking
    std::unordered_map<std::string, double> q_map_;
    std::map<std::string, int> joint_name_to_id_;
    
    double base_pos_x_ = 0.0;
    double base_pos_y_ = 0.0;
    double base_yaw_ = 0.0;
    std::vector<std::string> arm_joint_names_;
    bool model_ready_ = false;


    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr ghost_joint_pub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;


    // Precomputed “ghost joints” publishing layout
    std::vector<std::string> ghost_joint_names_;              // ordered list of non-fixed joints (excluding root/universe)
    std::vector<double>      ghost_joint_pos_;                // same size as ghost_joint_names_
    std::unordered_map<std::string, size_t> ghost_name_to_i_; // name -> index into ghost_joint_* arrays

    // Cache idx_q for arm joints so override is cheap
    std::unordered_map<std::string, int> arm_name_to_qidx_;

};