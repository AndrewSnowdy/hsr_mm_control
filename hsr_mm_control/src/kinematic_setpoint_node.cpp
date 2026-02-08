#include "hsr_mm_control/kinematic_setpoint_node.hpp"

#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <iostream>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

using namespace std::chrono_literals;

FinalPoseNode::FinalPoseNode()
: Node("final_pose_node"),
  use_ik_mode_(false),
  model_ready_(false),      
  arm_joint_names_{        
    "arm_lift_joint",
    "arm_flex_joint",
    "arm_roll_joint",
    "wrist_flex_joint",
    "wrist_roll_joint"
  },
  base_pos_x_(0.0),
  base_pos_y_(0.0),
  base_yaw_(0.0)
{

    target_sub_ = this->create_subscription<geometry_msgs::msg::Pose>(
        "/waypoint_target", 10, 
        [this](const geometry_msgs::msg::Pose::SharedPtr msg) {
            current_target_ = *msg;
            has_target_ = true;
            // Optional: Trigger the IK solve immediately when a new point arrives
        }
    );

    mode_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        "/use_ik_mode", 10, 
        [this](const std_msgs::msg::Bool::SharedPtr msg) {
            use_ik_mode_ = msg->data;
            // RCLCPP_INFO(this->get_logger(), "Mode Switched: %s", 
            //             use_ik_mode_ ? "IK MANIPULATION" : "TUCKED NAVIGATION");
        }
    );

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

// Eigen::VectorXd FinalPoseNode::solveGlobalIK(const Eigen::Vector3d& target_p)
// {
//     RCLCPP_INFO(get_logger(), "nq=%d nv=%d", model_.nq, model_.nv);
//     RCLCPP_INFO(get_logger(), "joint[1]=%s idx_q=%d nq=%d idx_v=%d nv=%d",
//                 model_.names[1].c_str(),
//                 model_.joints[1].idx_q(), model_.joints[1].nq(),
//                 model_.joints[1].idx_v(), model_.joints[1].nv());

//     RCLCPP_INFO(get_logger(), "Solver Called! Target: %.2f, %.2f, %.2f", target_p.x(), target_p.y(), target_p.z());
//     tf2::Quaternion q_target_tf(
//         current_target_.orientation.x,
//         current_target_.orientation.y,
//         current_target_.orientation.z,
//         current_target_.orientation.w);


//     if (std::abs(q_target_tf.length() - 1.0) > 0.1) {
//         RCLCPP_WARN(get_logger(), "Target Quaternion not normalized! Length: %.4f", q_target_tf.length());
//     }

//     q_target_tf.normalize();

//     double r, p, target_yaw;
//     tf2::Matrix3x3(q_target_tf).getRPY(r, p, target_yaw);

//     // Warm start
//     Eigen::VectorXd q = pinocchio::neutral(model_);
//     q[0] = base_pos_x_;
//     q[1] = base_pos_y_;
//     q[2] = std::cos(base_yaw_);
//     q[3] = std::sin(base_yaw_);



//     RCLCPP_INFO(get_logger(), "Warm Start Base: X=%.2f, Y=%.2f, Yaw=%.2f", q[0], q[1], base_yaw_);

//     pinocchio::normalize(model_, q);

//     RCLCPP_INFO(get_logger(), "Warm Start normalized Base: X=%.2f, Y=%.2f, Yaw=%.2f", q[0], q[1], base_yaw_);


//     // for (const auto& kv : q_map_) {
//     //     if (!joint_name_to_id_.count(kv.first)) continue;
//     //     const auto jid = joint_name_to_id_.at(kv.first);
//     //     int idx = model_.joints[jid].idx_q();
//     //     if (idx >= 0 && idx < q.size()) q[idx] = kv.second;
//     // }
//     for (const auto& kv : q_map_) {
//         if (!joint_name_to_id_.count(kv.first)) continue;
//         int jid = joint_name_to_id_.at(kv.first);
        
//         // Extract properties from the model for this specific joint ID
//         int qidx = model_.joints[jid].idx_q();
//         int nq   = model_.joints[jid].nq();

//         // Check bounds and ensure it's a standard 1-DOF joint
//         if (nq == 1 && qidx >= 0 && qidx < q.size()) {
//             // First, set the position from the map
//             q[qidx] = kv.second;
//             // Second, clamp it to legal limits for safety
//             q[qidx] = std::max(model_.lowerPositionLimit[qidx], 
//                                std::min(model_.upperPositionLimit[qidx], q[qidx]));
//         }
//     }



//     if (!std::isfinite(target_yaw)) {
//         RCLCPP_ERROR(get_logger(), "Target Yaw is NaN! Orientation was: w=%.2f z=%.2f", 
//                      current_target_.orientation.w, current_target_.orientation.z);
//         return Eigen::VectorXd();
//     }

//     const double damping = 0.5;
//     const double step_size = 0.1;
//     const int max_iters = 500;
//     pinocchio::Data data(model_);


    
//     RCLCPP_INFO(get_logger(), "starting IK");

//     for (int i = 0; i < max_iters; ++i) {

//         double norm_check = std::sqrt(q[2]*q[2] + q[3]*q[3]);
//         if (std::abs(norm_check - 1.0) > 1e-4) {
//              RCLCPP_ERROR(get_logger(), "ITER %d: Base not normalized BEFORE FK! Norm: %.6f", i, norm_check);
//              pinocchio::normalize(model_, q);
//         }

//         pinocchio::normalize(model_, q);
//         pinocchio::forwardKinematics(model_, data, q);
//         pinocchio::updateFramePlacements(model_, data);
        

//         RCLCPP_INFO(get_logger(), "completed forwardKinematics");


//         Eigen::Vector3d p = data.oMf[ee_fid_].translation();
//         Eigen::Vector3d err = target_p - p;
//         RCLCPP_INFO(get_logger(), "Initial EE Position: %.2f, %.2f, %.2f", p.x(), p.y(), p.z());

//         if (i % 1 == 0) {
//             RCLCPP_INFO(get_logger(), "IK iter %d |err|=%.4f  ee=[%.3f %.3f %.3f]",
//                     i, err.norm(), p.x(), p.y(), p.z());
//         }

//         if (err.norm() < 1e-3) {
//             RCLCPP_INFO(get_logger(), "IK converged in %d iters (|err|=%.6f)", i, err.norm());
//             return q;
//         }

//         std::cout << "Model nv: " << model_.nv << " | Data J size: " << data.J.cols() << std::endl;

//         Eigen::Matrix<double, 6, Eigen::Dynamic> J(6, model_.nv);
//         J.setZero();
//         pinocchio::computeFrameJacobian(model_, data, q, ee_fid_, pinocchio::LOCAL_WORLD_ALIGNED, J);

//         RCLCPP_INFO(get_logger(), "got Jacobian");

//         for (int col = 0; col < J.cols(); ++col) {
//             if (!J.col(col).allFinite() || J.col(col).array().abs().maxCoeff() > 1e6) {
//                 // If a column is exploding, zero it out so it doesn't poison the solver
//                 J.col(col).setZero(); 
//             }
//         }

//         J.col(2).setZero();


//         std::cout << "--- [ITER " << i << "] JACOBIAN Matrix ---" << std::endl;
//         std::cout << J << std::endl; 
//         std::cout << "---------------------------------------" << std::endl;

//         RCLCPP_INFO(get_logger(), "got Jacobian");

                                    
//         if (!J.allFinite()) {
//             RCLCPP_ERROR(get_logger(), "ITER %d: Jacobian contains Non-Finite values!", i);
//             return Eigen::VectorXd();
//         }
//         // if (!J.array().isFinite().all()) return Eigen::VectorXd();;


//         Eigen::MatrixXd Jt = J.topRows<3>();           // 3 x nv
//         Eigen::Matrix3d A  = Jt * Jt.transpose();      // 3 x 3
//         A.diagonal().array() += damping * damping;

//         Eigen::Vector3d alpha = A.ldlt().solve(err);   // 3 x 1
//         Eigen::VectorXd dq    = Jt.transpose() * alpha; // nv x 1  <-- IMPORTANT

//         if (!dq.allFinite()) {
//             RCLCPP_ERROR(get_logger(), "ITER %d: Delta-Q (dq) is Non-Finite!", i);
//             return Eigen::VectorXd();
//         }
//         // if (!dq.array().isFinite().all()) return Eigen::VectorXd();;
 
//         pinocchio::normalize(model_, q);

//         try {
//             q = pinocchio::integrate(model_, q, dq * step_size);
//         } catch (...) {
//             RCLCPP_ERROR(get_logger(), "ITER %d: pinocchio::integrate CRASHED!", i);
//             return Eigen::VectorXd();
//         }
//         // q = pinocchio::integrate(model_, q, dq * step_size);

//         q[2] = std::cos(target_yaw);
//         q[3] = std::sin(target_yaw);

//         // for (int j = 0; j < model_.nq; ++j) {
//         //     if (j == 2 || j == 3) continue; 
//         //     q[j] = std::max(model_.lowerPositionLimit[j], std::min(model_.upperPositionLimit[j], q[j]));
//         // }
//         for (pinocchio::JointIndex jid = 0; jid < (pinocchio::JointIndex)model_.njoints; ++jid) {
//             int qidx = model_.joints[jid].idx_q();
//             int nq   = model_.joints[jid].nq();

//             // Check: Is it a standard joint? Is the index valid (not -1)? Is it within the vector size?
//             if (nq == 1 && qidx >= 0 && qidx < q.size()) {
//                 q[qidx] = std::clamp(q[qidx],
//                                     model_.lowerPositionLimit[qidx],
//                                     model_.upperPositionLimit[qidx]);
//             }
//         }

//         pinocchio::normalize(model_, q);

//         if (!q.allFinite()) {
//             RCLCPP_ERROR(get_logger(), "ITER %d: q contains Non-Finite values after limits!", i);
//             return Eigen::VectorXd();
//         }
//         // if (!q.allFinite()) return Eigen::VectorXd();;

//     }

//     RCLCPP_WARN(get_logger(), "IK timeout after %d iters", max_iters);
//     return Eigen::VectorXd(); // failure
// }


Eigen::VectorXd FinalPoseNode::solveGlobalIK(const Eigen::Vector3d& target_p)
{
    // --- 1. TARGET SETUP ---
    tf2::Quaternion q_target_tf(
        current_target_.orientation.x, current_target_.orientation.y,
        current_target_.orientation.z, current_target_.orientation.w);
    q_target_tf.normalize();

    double r, p, target_yaw;
    tf2::Matrix3x3(q_target_tf).getRPY(r, p, target_yaw);

    if (!std::isfinite(target_yaw)) {
        RCLCPP_ERROR(get_logger(), "Target Yaw is NaN! Aborting solve.");
        return Eigen::VectorXd();
    }

    // --- 2. WARM START CONFIGURATION ---
    // Use last solved goal if available, otherwise start from current odom
    Eigen::VectorXd q;
    if (q_goal_.size() == model_.nq && q_goal_.allFinite()) {
        q = q_goal_;
    } else {
        q = pinocchio::neutral(model_);
        q[0] = base_pos_x_;
        q[1] = base_pos_y_;
    }

    // Update arm joints from current hardware state (q_map_)
    for (const auto& [name, pos] : q_map_) {
        if (!joint_name_to_id_.count(name)) continue;
        int jid = joint_name_to_id_.at(name);
        int qidx = model_.joints[jid].idx_q();
        
        // Safety: only update standard 1-DOF joints and check bounds (skips 'universe')
        if (model_.joints[jid].nq() == 1 && qidx >= 0 && qidx < q.size()) {
            q[qidx] = pos;
        }
    }

    // --- 3. SOLVER PARAMETERS ---
    const double damping = 0.5;
    const double step_size = 0.1;
    const int max_iters = 200; // konsolidated for performance
    const double max_dq = 0.1;  // Safety clamp for dq
    pinocchio::Data data(model_);

    RCLCPP_INFO(get_logger(), "IK Start | Target: [%.2f, %.2f, %.2f]", target_p.x(), target_p.y(), target_p.z());

    // --- 4. ITERATIVE LOOP ---
    for (int i = 0; i < max_iters; ++i) {
        // FK and Placement Update
        pinocchio::normalize(model_, q);
        pinocchio::forwardKinematics(model_, data, q);
        pinocchio::updateFramePlacements(model_, data);

        Eigen::Vector3d p_curr = data.oMf[ee_fid_].translation();
        Eigen::Vector3d err = target_p - p_curr;

        // Check for convergence
        if (err.norm() < 1e-3) {
            RCLCPP_INFO(get_logger(), "IK converged in %d iters | final_err: %.6f", i, err.norm());
            return q;
        }

        // --- JACOBIAN COMPUTATION ---
        Eigen::Matrix<double, 6, Eigen::Dynamic> J(6, model_.nv);
        J.setZero();
        pinocchio::computeFrameJacobian(model_, data, q, ee_fid_, pinocchio::LOCAL_WORLD_ALIGNED, J);

        // Task-Space Pruning: Zero out columns 8+ (mimics/head) and any non-finite values
        for (int col = 0; col < J.cols(); ++col) {
            if (col >= 8 || !J.col(col).allFinite() || J.col(col).norm() > 1e6) {
                J.col(col).setZero();
            }
        }
        J.col(2).setZero(); // Lock Base Yaw velocity

        // --- PSEUDO-INVERSE SOLVE ---
        Eigen::MatrixXd Jt = J.topRows<3>(); 
        Eigen::Matrix3d A  = Jt * Jt.transpose();
        A.diagonal().array() += (damping * damping);

        Eigen::Vector3d alpha = A.ldlt().solve(err);
        Eigen::VectorXd dq    = Jt.transpose() * alpha;

        // --- SAFETY: VELOCITY CLAMPING ---
        double dq_norm = dq.lpNorm<Eigen::Infinity>();
        if (dq_norm > max_dq) {
            dq *= (max_dq / dq_norm);
        }

        if (!dq.allFinite()) {
            RCLCPP_ERROR(get_logger(), "Iter %d: Non-finite dq detected. Aborting.", i);
            break;
        }

        // Consolidated Debug Logging (Every 20 iterations or first)
        if (i == 0 || i % 20 == 0) {
            RCLCPP_INFO(get_logger(), "Iter %d | err: %.4f | EE: [%.2f, %.2f, %.2f]", 
                        i, err.norm(), p_curr.x(), p_curr.y(), p_curr.z());
        }

        // --- INTEGRATION AND CONSTRAINTS ---
        q = pinocchio::integrate(model_, q, dq * step_size);

        // Post-integration constraints
        q[2] = std::cos(target_yaw); // Lock orientation
        q[3] = std::sin(target_yaw);

        // Clamp arm joint limits safely (avoiding universe joint at index -1)
        for (pinocchio::JointIndex jid = 0; jid < (pinocchio::JointIndex)model_.njoints; ++jid) {
            int qidx = model_.joints[jid].idx_q();
            if (model_.joints[jid].nq() == 1 && qidx >= 0 && qidx < q.size()) {
                q[qidx] = std::clamp(q[qidx], model_.lowerPositionLimit[qidx], model_.upperPositionLimit[qidx]);
            }
        }
    }

    RCLCPP_WARN(get_logger(), "IK reached timeout (%d iters) without full convergence.", max_iters);
    return q; 
}


void FinalPoseNode::tick() {
    // Check if we are ready and have a target
    if (!model_ready_ || !has_target_ || !odom_received_ || !joints_received_) {
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, 
                             "Solver Waiting: [Model: %d, Target: %d, Odom: %d, Joints: %d]",
                             model_ready_, has_target_, odom_received_, joints_received_);
        return;
    }

    tf2::Quaternion q_tf(
        current_target_.orientation.x,
        current_target_.orientation.y,
        current_target_.orientation.z,
        current_target_.orientation.w);
    double r, p, target_yaw;
    tf2::Matrix3x3(q_tf).getRPY(r, p, target_yaw);  

    double dist_to_waypoint = std::hypot(current_target_.position.x - base_pos_x_, 
                                          current_target_.position.y - base_pos_y_);

    if (!use_ik_mode_) {
        // --- MODE: TUCKED NAVIGATION ---
        // Create a neutral configuration based on the model
        Eigen::VectorXd q_tucked = pinocchio::neutral(model_);


        // Position base at the waypoint target
        q_tucked[0] = current_target_.position.x;
        q_tucked[1] = current_target_.position.y;
        q_tucked[2] = std::cos(target_yaw);; // cos(yaw) - looking straight
        q_tucked[3] = std::sin(target_yaw);; // sin(yaw)

        // Set arm joints to a safe "Tucked" pose (Neutral)
        // Using your precomputed map to find the correct indices
        if (arm_name_to_qidx_.count("arm_lift_joint")) 
            q_tucked[arm_name_to_qidx_["arm_lift_joint"]] = 0.01; // 5cm up from floor
        if (arm_name_to_qidx_.count("arm_flex_joint")) 
            q_tucked[arm_name_to_qidx_["arm_flex_joint"]] = -0.01; // Slightly back

        q_goal_ = q_tucked;
        publishGhostPose(q_goal_);

        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, 
                             "Approach Mode: Dist=%.2f. Arm is Tucked.", dist_to_waypoint);
    } 
    else {
        // --- MODE: IK MANIPULATION ---
        
        // Use your Change Detection logic here to save CPU
        double dist_change = 100.0;
        if (q_goal_.size() > 0) {
            dist_change = std::sqrt(
                std::pow(current_target_.position.x - last_solved_target_.position.x, 2) +
                std::pow(current_target_.position.y - last_solved_target_.position.y, 2) +
                std::pow(current_target_.position.z - last_solved_target_.position.z, 2)
            );
        }

        if (dist_change < TARGET_THRESHOLD && q_goal_.size() > 0) {
            publishGhostPose(q_goal_);
            return; 
        }

        Eigen::Vector3d target_vec(current_target_.position.x, current_target_.position.y, current_target_.position.z);
        Eigen::VectorXd result = solveGlobalIK(target_vec);

        if (result.size() > 0 && !result.array().isNaN().any()) {
            q_goal_ = result;
            last_solved_target_ = current_target_;
            publishGhostPose(q_goal_);
        }
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
        RCLCPP_INFO(get_logger(), "Joint [%ld]: %s | Vel Index (col): %d | nq: %d", 
                i, model_.names[i].c_str(), model_.joints[i].idx_v(), model_.joints[i].nv());
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