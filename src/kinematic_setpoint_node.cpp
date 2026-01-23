#include "hsr_mm_control/kinematic_setpoint_node.hpp"

#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

using namespace std::chrono_literals;

FinalPoseNode::FinalPoseNode()
: Node("final_pose_node"),
  state_(ControlState::IDLE),
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
    // 1. Parameters (Same as before)
    this->declare_parameter("target_x", 2.2);
    // this->declare_parameter("target_y", 0.0);
    this->declare_parameter("target_z", 1.04);


    joint_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
    "/joint_states", 10, [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
        for (size_t i = 0; i < msg->name.size(); ++i) {
        q_map_[msg->name[i]] = msg->position[i];
        }
        joints_received_ = true;
    });

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom", 10, [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
        base_pos_x_ = msg->pose.pose.position.x;
        base_pos_y_ = msg->pose.pose.position.y;

        tf2::Quaternion q(
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w);

        double r, p, yaw;
        tf2::Matrix3x3(q).getRPY(r, p, yaw);
        base_yaw_ = yaw;
        odom_received_ = true;
    });
        

    ghost_joint_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/ghost_joint_states", 10);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    // 3. Load Model
    loadPinocchioModel();

    // 4. Main Loop
    timer_ = this->create_wall_timer(50ms, std::bind(&FinalPoseNode::tick, this));
}

Eigen::VectorXd FinalPoseNode::solveGlobalIK(const Eigen::Vector3d& target_p)
{
    // Warm start: MUST be real state (don’t leave it as neutral)
    Eigen::VectorXd q = pinocchio::neutral(model_);

    // planar: [x, y, cos(yaw), sin(yaw)]
    q[0] = base_pos_x_;
    q[1] = base_pos_y_;
    q[2] = std::cos(base_yaw_);
    q[3] = std::sin(base_yaw_);

    for (const auto& kv : q_map_) {
        if (!joint_name_to_id_.count(kv.first)) continue;
        const auto jid = joint_name_to_id_.at(kv.first);
        int idx = model_.joints[jid].idx_q();
        if (idx >= 0 && idx < q.size()) q[idx] = kv.second;
    }

    const double damping = 0.1;
    const double step_size = 0.1;
    const int max_iters = 500;
    pinocchio::Data data(model_);




    for (int i = 0; i < max_iters; ++i) {
        pinocchio::forwardKinematics(model_, data, q);
        pinocchio::updateFramePlacements(model_, data);

        Eigen::Vector3d p = data.oMf[ee_fid_].translation();
        Eigen::Vector3d err = target_p - p;

        if (i % 25 == 0) {
            RCLCPP_INFO(get_logger(), "IK iter %d |err|=%.4f  ee=[%.3f %.3f %.3f]",
                    i, err.norm(), p.x(), p.y(), p.z());
        }

        if (err.norm() < 1e-3) {
            RCLCPP_INFO(get_logger(), "IK converged in %d iters (|err|=%.6f)", i, err.norm());
            return q;
        }

        Eigen::Matrix<double, 6, Eigen::Dynamic> J(6, model_.nv);
        pinocchio::computeFrameJacobian(model_, data, q, ee_fid_, pinocchio::LOCAL_WORLD_ALIGNED, J);
                                    
        if (!J.array().isFinite().all()) return Eigen::VectorXd();;

        Eigen::MatrixXd Jt = J.topRows<3>();           // 3 x nv
        Eigen::Matrix3d A  = Jt * Jt.transpose();      // 3 x 3
        A.diagonal().array() += damping * damping;

        Eigen::Vector3d alpha = A.ldlt().solve(err);   // 3 x 1
        Eigen::VectorXd dq    = Jt.transpose() * alpha; // nv x 1  <-- IMPORTANT

        if (!dq.array().isFinite().all()) return Eigen::VectorXd();;
 
        q = pinocchio::integrate(model_, q, dq * step_size);

        if (!q.allFinite()) return Eigen::VectorXd();;

    }

    RCLCPP_WARN(get_logger(), "IK timeout after %d iters", max_iters);
    return Eigen::VectorXd(); // failure
}

void FinalPoseNode::tick() {
    if (!model_ready_ ) return;

    if (!model_ready_ || !odom_received_ || !joints_received_ || !haveBaseState() || !haveArmState()) {
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
            "Waiting: odom=%d joints=%d baseFinite=%d armComplete=%d",
            (int)odom_received_, (int)joints_received_, (int)haveBaseState(), (int)haveArmState());
        return;
    }


    if (state_ == ControlState::IDLE) {
        Eigen::Vector3d cube_target(2.0, 0.0, 1.04);
        Eigen::VectorXd result = solveGlobalIK(cube_target);

        // CHECK 1: Did the solver return an empty vector (failure)?
        // CHECK 2: Is it NaN?
        if (result.size() > 0 && !result.array().isNaN().any()) {
            q_goal_ = result;
            state_ = ControlState::REACHED; 
            RCLCPP_INFO(get_logger(), "Goal locked successfully.");
        } else {
            // If failed, wait and try again in the next tick
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "IK failed, retrying...");
        }
    }

    // Only publish if we actually have a goal; otherwise, ghost = current robot
    if (q_goal_.size() > 0) {
        publishGhostPose(q_goal_);
    }
}

void FinalPoseNode::loadPinocchioModel()
{
    auto client = std::make_shared<rclcpp::SyncParametersClient>(
        this, "/whole_body/robot_state_publisher"
    );

    RCLCPP_INFO(get_logger(), "Waiting for robot_state_publisher service...");
    while (!client->wait_for_service(1s)) {
        if (!rclcpp::ok()) return;
        RCLCPP_INFO(get_logger(), "Service not found, retrying...");
    }

    RCLCPP_INFO(get_logger(), "Service found! Fetching URDF...");
    std::string urdf = client->get_parameter<std::string>("robot_description");

    pinocchio::urdf::buildModelFromXML(urdf, pinocchio::JointModelPlanar(), model_);
    data_ = std::make_unique<pinocchio::Data>(model_);

    if (!model_.existFrame("hand_palm_link")) {
        RCLCPP_ERROR(get_logger(), "Frame hand_palm_link not found!");
        model_ready_ = false;
        return;
    }
    ee_fid_ = model_.getFrameId("hand_palm_link");
    
    for (pinocchio::JointIndex i = 0; i < (pinocchio::JointIndex)model_.njoints; ++i) {
        joint_name_to_id_[model_.names[i]] = i;
    }

    // ---- PRECOMPUTE GHOST LAYOUT ----
    ghost_joint_names_.clear();
    ghost_name_to_i_.clear();
    arm_name_to_qidx_.clear();

    // Map arm joints to their indices in q
    for (const auto& name : arm_joint_names_) {
        if (joint_name_to_id_.count(name)) {
            int jid = joint_name_to_id_.at(name);
            arm_name_to_qidx_[name] = model_.joints[jid].idx_q();
        }
    }

    // Build the list of joints to publish to RViz
    for (const auto& [name, jid] : joint_name_to_id_) {
        if (name == "universe" || name == "root_joint") continue;

        // SKIP MIMIC JOINTS: Search the URDF string for the mimic tag
        std::string search_str = "<joint name=\"" + name + "\"";
        size_t pos = urdf.find(search_str);
        if (pos != std::string::npos) {
            size_t end_joint = urdf.find("</joint>", pos);
            std::string joint_xml = urdf.substr(pos, end_joint - pos);
            if (joint_xml.find("<mimic") != std::string::npos) {
                RCLCPP_INFO(get_logger(), "Auto-skipped mimic joint: %s", name.c_str());
                continue; 
            }
        }

        // Only include active joints (nq > 0)
        if (model_.joints[jid].nq() > 0) {
            ghost_name_to_i_[name] = ghost_joint_names_.size();
            ghost_joint_names_.push_back(name);
        }
    }

    ghost_joint_pos_.assign(ghost_joint_names_.size(), 0.0);
    model_ready_ = true;
    RCLCPP_INFO(get_logger(), "Ghost setup complete. Joints: %zu", ghost_joint_names_.size());
}

void FinalPoseNode::publishGhostPose(const Eigen::VectorXd &q_goal)
{
  auto now = this->get_clock()->now();

  // ---- TF: odom -> ghost/base_footprint ----
  geometry_msgs::msg::TransformStamped t;
  t.header.stamp = now;
  t.header.frame_id = "odom";
  t.child_frame_id  = "ghost/base_footprint";

  t.transform.translation.x = q_goal[0];
  t.transform.translation.y = q_goal[1];
  t.transform.translation.z = 0.0;

  tf2::Quaternion q_tf;
  q_tf.setRPY(0.0, 0.0, std::atan2(q_goal[3], q_goal[2]));
  t.transform.rotation.x = q_tf.x();
  t.transform.rotation.y = q_tf.y();
  t.transform.rotation.z = q_tf.z();
  t.transform.rotation.w = q_tf.w();
  tf_broadcaster_->sendTransform(t);



  // ---- JointState: fill precomputed layout ----
  // Fill from latest q_map_ (fallback 0.0)
  // Note: this is O(#ghost joints), no push_backs, no filtering.
  for (size_t i = 0; i < ghost_joint_names_.size(); ++i) {
    const auto& name = ghost_joint_names_[i];
    auto it = q_map_.find(name);
    ghost_joint_pos_[i] = (it != q_map_.end() && std::isfinite(it->second)) ? it->second : 0.0;
  }

  // Override arm joints from q_goal (O(#arm joints))
  for (const auto& name : arm_joint_names_) {
    auto it_idx = arm_name_to_qidx_.find(name);
    if (it_idx == arm_name_to_qidx_.end()) continue;

    int qidx = it_idx->second;
    if (!(qidx >= 0 && qidx < q_goal.size())) continue;

    auto it_pub = ghost_name_to_i_.find(name);
    if (it_pub == ghost_name_to_i_.end()) continue;

    ghost_joint_pos_[it_pub->second] = q_goal[qidx];
  }

  sensor_msgs::msg::JointState js;
  js.header.stamp = now;

  
  js.name = ghost_joint_names_;
  js.position = ghost_joint_pos_;

  js.name.push_back("base_x");
  js.position.push_back(q_goal[0]);

  js.name.push_back("base_y");
  js.position.push_back(q_goal[1]);

  js.name.push_back("base_yaw");
  js.position.push_back(std::atan2(q_goal[3], q_goal[2]));
  
  ghost_joint_pub_->publish(js);
}




bool FinalPoseNode::haveArmState() const {
  for (const auto& name : arm_joint_names_) {
    auto it = q_map_.find(name);
    if (it == q_map_.end()) return false;
    if (!std::isfinite(it->second)) return false;
  }
  return true;
}

bool FinalPoseNode::haveBaseState() const {
  return std::isfinite(base_pos_x_) &&
         std::isfinite(base_pos_y_) &&
         std::isfinite(base_yaw_);
}


int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FinalPoseNode>());
  rclcpp::shutdown();
  return 0;
}