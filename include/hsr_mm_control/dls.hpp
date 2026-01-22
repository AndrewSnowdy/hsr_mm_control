#pragma once

#include <unordered_map>
#include <map>
#include <string>
#include <vector>
#include <memory>
#include <Eigen/Dense>

// ROS 2 Core - ADDED THIS
#include <rclcpp/rclcpp.hpp> 

// Pinocchio
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/algorithm/jacobian.hpp>

// Messages
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

class DampedLeastSquares : public rclcpp::Node
{
public:
  explicit DampedLeastSquares();

private:
  // ROS
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr joint_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr base_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::unordered_map<std::string, double> q_map_;
  std::map<std::string, int> joint_name_to_id_;

  double base_pos_x_, base_pos_y_, base_yaw_;
  const std::vector<std::string> arm_joint_names_;

  // Pinocchio
  pinocchio::Model model_;
  std::unique_ptr<pinocchio::Data> data_;
  bool model_ready_;
  pinocchio::FrameIndex ee_fid_;   // needed because your .cpp uses ee_fid_

  // Callbacks / helpers
  void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg);
  void tick();

  void loadPinocchioModel();
  bool buildQ(Eigen::VectorXd &q_out);

  void publishArmCommand(const Eigen::VectorXd &q, const Eigen::VectorXd &dq);
  void publishBaseCommand(const Eigen::VectorXd &dq, double dt);



};
