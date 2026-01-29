#include "hsr_mm_control/dls.hpp"

#include <chrono>
#include <algorithm>

// Pinocchio algorithms
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>

#include <rclcpp/parameter_client.hpp>
#include <tf2/utils.h> // Useful helper for getYaw

using namespace std::chrono_literals;

DampedLeastSquares::DampedLeastSquares()
: Node("final_pose_node"),
  base_pos_x_(0.0),
  base_pos_y_(0.0),
  base_yaw_(0.0),  
  arm_joint_names_{
    "arm_lift_joint",
    "arm_flex_joint",
    "arm_roll_joint",
    "wrist_flex_joint",
    "wrist_roll_joint"
  },
  model_ready_(false)
{
  RCLCPP_INFO(get_logger(), "final_pose_node started.");

  this->declare_parameter("target_x", 2.0);
  this->declare_parameter("target_y", 0.0);
  this->declare_parameter("target_z", 1.04);

  joint_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
    "/joint_states",
    10,
    std::bind(&DampedLeastSquares::jointStateCallback, this, std::placeholders::_1)
  );

odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
  "/odom", 10, [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
    base_pos_x_ = msg->pose.pose.position.x;
    base_pos_y_ = msg->pose.pose.position.y;

    // Use the manual conversion from your SimpleMover script
    tf2::Quaternion q(
      msg->pose.pose.orientation.x,
      msg->pose.pose.orientation.y,
      msg->pose.pose.orientation.z,
      msg->pose.pose.orientation.w);

    double r, p, yaw;
    tf2::Matrix3x3(q).getRPY(r, p, yaw);
    base_yaw_ = yaw;
  });

  timer_ = this->create_wall_timer(
    50ms,
    std::bind(&DampedLeastSquares::tick, this)
  );

  joint_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
    "/arm_trajectory_controller/joint_trajectory", 10);
  base_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/omni_base_controller/cmd_vel", 10);
  
  loadPinocchioModel();
}

void DampedLeastSquares::jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  for (size_t i = 0; i < msg->name.size(); ++i) {
    q_map_[msg->name[i]] = msg->position[i];
  }
}

bool DampedLeastSquares::buildQ(Eigen::VectorXd &q_out)
{
  if (!model_ready_) return false;
  Eigen::VectorXd q = pinocchio::neutral(model_);

  q[0] = base_pos_x_;
  q[1] = base_pos_y_;
  q[2] = std::cos(base_yaw_);
  q[3] = std::sin(base_yaw_);

  for (const auto &kv : q_map_) {
      if (joint_name_to_id_.count(kv.first)) {
          int jid = joint_name_to_id_[kv.first];
          const auto &jmodel = model_.joints[jid];
          if (jmodel.nq() == 1) {
              q[jmodel.idx_q()] = kv.second;
          }
      }
  }
  q_out = q;
  return true;
}


void DampedLeastSquares::tick()
{
  if (!model_ready_) return;

  Eigen::VectorXd q;
  if (!buildQ(q)) return;

  pinocchio::forwardKinematics(model_, *data_, q);
  pinocchio::updateFramePlacements(model_, *data_);

  const auto &oMf = data_->oMf[ee_fid_];
  const auto p_current = oMf.translation();

  Eigen::Vector3d p_target(
      get_parameter("target_x").as_double(),
      get_parameter("target_y").as_double(),
      get_parameter("target_z").as_double()
    );

  Eigen::Vector3d error = p_target - p_current;

  Eigen::Matrix<double, 6, Eigen::Dynamic> J(6, model_.nv);
  J.setZero();
  pinocchio::computeFrameJacobian(model_, *data_, q, ee_fid_, pinocchio::LOCAL_WORLD_ALIGNED, J);
  auto J_pos = J.topRows<3>();

  Eigen::VectorXd weights = Eigen::VectorXd::Ones(model_.nv);
  weights[0] = 0.5;
  weights[1] = 0.1;
  weights[2] = 10.0;

  // Make the arm joints very "cheap" to encourage them to move first
  for(int i=3; i < model_.nv; ++i) weights[i] = 1.0;

  Eigen::DiagonalMatrix<double, Eigen::Dynamic> W(weights);
  Eigen::MatrixXd Winv = W.inverse();

  double lambda = 0.3;
  auto J_T = J_pos.transpose();


  auto JJT_weighted_damped = J_pos * Winv * J_T + Eigen::Matrix3d::Identity() * (lambda * lambda);;
  Eigen::VectorXd dq = Winv * J_T * JJT_weighted_damped.ldlt().solve(error);

  publishBaseCommand(dq, 0.05);
  publishArmCommand(q, dq);

  RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "distance to target: %.4f", error.norm());

}

void DampedLeastSquares::publishBaseCommand(const Eigen::VectorXd &dq, double dt)
{
  auto twist_msg = geometry_msgs::msg::Twist();
  double yaw = base_yaw_;

  // 3. Transform World Displacement to Robot-Frame Velocity (The SimpleMover math)
  // v_robot = RotationMatrix(-yaw) * v_world
  twist_msg.linear.x = (dq[0] * std::cos(yaw) + dq[1] * std::sin(yaw)) / dt;
  twist_msg.linear.y = (-dq[0] * std::sin(yaw) + dq[1] * std::cos(yaw)) / dt;

  twist_msg.angular.z = dq[2] / dt;

  // 4. Safety: Clamp velocities
  twist_msg.linear.x = std::clamp(twist_msg.linear.x, -0.2, 0.2);
  twist_msg.linear.y = std::clamp(twist_msg.linear.y, -0.2, 0.2);
  twist_msg.angular.z = std::clamp(twist_msg.angular.z, -0.4, 0.4);

  base_pub_->publish(twist_msg);
}


void DampedLeastSquares::publishArmCommand(const Eigen::VectorXd &q, const Eigen::VectorXd &dq)
{
  // Use Pinocchio to safely integrate the displacement into the next state
  Eigen::VectorXd q_next = pinocchio::integrate(model_, q, dq);

  auto msg = trajectory_msgs::msg::JointTrajectory();
  msg.joint_names = arm_joint_names_;

  trajectory_msgs::msg::JointTrajectoryPoint point;
  for (const auto & name : arm_joint_names_) {
    int jid = joint_name_to_id_[name];
    int idx = model_.joints[jid].idx_q();
    point.positions.push_back(q_next[idx]);
  }

  point.time_from_start = rclcpp::Duration::from_seconds(0.1);
  msg.points.push_back(point);

  joint_pub_->publish(msg);
}

void DampedLeastSquares::loadPinocchioModel()
{
  auto client = std::make_shared<rclcpp::SyncParametersClient>(
    this, "/whole_body/robot_state_publisher"
  );

  while (!client->wait_for_service(1s)) {
    RCLCPP_INFO(get_logger(), "Waiting for robot_state_publisher...");
  }

  std::string urdf = client->get_parameter<std::string>("robot_description");

  pinocchio::urdf::buildModelFromXML(urdf, pinocchio::JointModelPlanar(), model_);

  data_ = std::make_unique<pinocchio::Data>(model_);
  model_ready_ = true;

  ee_fid_ = model_.getFrameId("hand_palm_link");

  RCLCPP_INFO(get_logger(),
              "Pinocchio model loaded (nq=%d nv=%d)",
              model_.nq, model_.nv);


  // In loadPinocchioModel after model is built
  for (pinocchio::JointIndex i = 0; i < (pinocchio::JointIndex)model_.njoints; ++i) {
      joint_name_to_id_[model_.names[i]] = i;
  }

  for (pinocchio::JointIndex jid = 1; jid < (pinocchio::JointIndex)model_.njoints; ++jid) {
    RCLCPP_INFO(this->get_logger(), "Joint [%ld]: %s | Type: %s | nv index: %d", 
                jid, 
                model_.names[jid].c_str(),
                model_.joints[jid].shortname().c_str(),
                model_.joints[jid].idx_v());
  }
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DampedLeastSquares>());
  rclcpp::shutdown();
  return 0;
}
