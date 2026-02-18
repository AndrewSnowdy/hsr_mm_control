#include "hsr_mm_control/joint_trajectory.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <cmath>
#include <algorithm>

void QuinticSpline::solve(double q0, double qf, double v0, double vf, double a0, double af, double T){
    T_ = T;
    Eigen::MatrixXd M(6, 6);
    M << 1, 0, 0, 0, 0, 0,
         0, 1, 0, 0, 0, 0,
         0, 0, 2, 0, 0, 0,
         1, T, pow(T,2), pow(T,3), pow(T,4), pow(T,5),
         0, 1, 2*T, 3*pow(T,2), 4*pow(T,3), 5*pow(T,4),
         0, 0, 2, 6*T, 12*pow(T,2), 20*pow(T, 3);

    Eigen::VectorXd b(6);
    b << q0, v0, a0, qf, vf, af;
    a_ = M.colPivHouseholderQr().solve(b);
}

double QuinticSpline::get_pos(double t) const {
    if (t <= 0) return a_[0];
    if (t >= T_) t = T_; 

    return a_[0] + 
           a_[1]*t + 
           a_[2]*std::pow(t, 2) + 
           a_[3]*std::pow(t, 3) + 
           a_[4]*std::pow(t, 4) + 
           a_[5]*std::pow(t, 5);
}

double QuinticSpline::get_vel(double t) const {
    if (t <= 0 || t >= T_) return 0.0;
    return a_[1] + 2*a_[2]*t + 3*a_[3]*pow(t,2) + 4*a_[4]*pow(t,3) + 5*a_[5]*pow(t,4);
}

JointTrajectoryController::JointTrajectoryController() : Node("joint_trajectory_controller") {
    goal_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
        "/ghost_joint_states", 10,
        std::bind(&JointTrajectoryController::on_goal_recieved, this, std::placeholders::_1)
    );

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom",
        // "/omni_base_controller/wheel_odom", // since /odom is not published (issues with lidar)
         10,
        std::bind(&JointTrajectoryController::odom_callback, this, std::placeholders::_1)
    );

    state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states", 10,
        [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
            for (size_t i = 0; i < msg->name.size(); ++i) {
                current_arm_positions_[msg->name[i]] = msg->position[i];
                current_arm_velocities_[msg->name[i]] = msg->velocity[i];
            }
        }
    );


    arm_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>("/arm_trajectory_controller/joint_trajectory", 10);
    base_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/omni_base_controller/cmd_vel", 10);

    timer_ = this->create_wall_timer(std::chrono::milliseconds(10), std::bind(&JointTrajectoryController::timer_callback, this));

    arm_joints_ = {"arm_lift_joint", "arm_flex_joint", "arm_roll_joint", "wrist_flex_joint", "wrist_roll_joint"};

}

void JointTrajectoryController::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    current_base_x_ = msg->pose.pose.position.x;
    current_base_y_ = msg->pose.pose.position.y;

    current_vx_ = msg->twist.twist.linear.x;
    current_vy_ = msg->twist.twist.linear.y;
    current_vw_ = msg->twist.twist.angular.z;

    tf2::Quaternion q(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
                      msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
    double r, p, y;

    double cy = std::cos(current_yaw_);
    double sy = std::sin(current_yaw_);

    current_vx_world_ = current_vx_ * cy - current_vy_ * sy;
    current_vy_world_ = current_vx_ * sy + current_vy_ * cy;

    tf2::Matrix3x3(q).getRPY(r, p, y);
    current_yaw_ = y;
}

void JointTrajectoryController::on_goal_recieved(const sensor_msgs::msg::JointState::SharedPtr msg) {
    std::unordered_map<std::string, double> goal;
    for (size_t i = 0; i < msg->name.size(); ++i) goal[msg->name[i]] = msg->position[i];

    // 1. Check for Goal Drift
    double goal_drift = std::hypot(goal["base_x"] - last_goal_x_, goal["base_y"] - last_goal_y_);
    if (goal_drift < 0.01) return;
    if (is_executing_ && goal_drift < 0.10) return;

    // 2. Calculate Base Requirements
    double dx = goal["base_x"] - current_base_x_;
    double dy = goal["base_y"] - current_base_y_;
    double L = std::hypot(dx, dy);

    // 3. NEW: Calculate Arm Requirements
    double max_arm_delta = 0.0;
    for (const auto& name : arm_joints_) {
        double delta = std::abs(goal[name] - current_arm_positions_[name]);
        if (delta > max_arm_delta) max_arm_delta = delta;
    }

    // 4. DYNAMIC TIME CALCULATION (Bottleneck-based)
    double base_time = L / 0.1;            // Assume 0.15 m/s cruise
    double arm_time  = max_arm_delta / 0.1; // Assume 0.2 rad/s cruise

    // Take the slowest component as the master clock, but keep a floor for smoothness
    double T_dynamic = std::max({base_time, arm_time, 2.5}); 
    
    RCLCPP_INFO(get_logger(), "!!! NEW TARGET !!! Base Dist: %.3f m | Arm Move: %.3f rad | Time: %.2f s", 
                L, max_arm_delta, T_dynamic);
    
    last_goal_x_ = goal["base_x"];
    last_goal_y_ = goal["base_y"];
    total_expected_time_ = T_dynamic;

    // 5. Solve Splines (Base)
    splines_["base_x"].solve(current_base_x_, goal["base_x"], current_vx_world_, 0, 0, 0, T_dynamic);
    splines_["base_y"].solve(current_base_y_, goal["base_y"], current_vy_world_, 0, 0, 0, T_dynamic);
    splines_["base_yaw"].solve(current_yaw_, goal["base_yaw"], 0, 0, 0, 0, T_dynamic); // or current_vw_

    // 6. Solve Splines (Arm)
    for (const auto& name : arm_joints_) {
        double q0 = current_arm_positions_[name];
        double qf = goal.count(name) ? goal[name] : q0;
        double v0 = current_arm_velocities_[name]; // Use actual current velocity for C2 continuity
        
        splines_[name].solve(q0, qf, v0, 0, 0, 0, T_dynamic);
    }

    last_t_ = this->now();
    current_time_s_ = 0.0;
    is_executing_ = true;
}

void JointTrajectoryController::timer_callback() {
    if (!is_executing_) return;

    auto now = this->now();
    if (last_t_.nanoseconds() == 0) { 
        last_t_ = now; 
        return; // Skip the first tick to get a valid delta next time
    }

    double dt = (now - last_t_).seconds();
    last_t_ = now;
    dt = std::clamp(dt, 0.0, 0.05);

    current_time_s_ += dt;

    // Check for completion based on time
    if (current_time_s_ >= total_expected_time_) {
        is_executing_ = false;
        base_pub_->publish(geometry_msgs::msg::Twist());
        return;
    }

    // --- SET TARGETS DIRECTLY FROM SPLINE ---
    // target_vx and target_vy are now in m/s directly
    double target_x  = splines_["base_x"].get_pos(current_time_s_);
    double target_vx = splines_["base_x"].get_vel(current_time_s_);

    double target_y  = splines_["base_y"].get_pos(current_time_s_);
    double target_vy = splines_["base_y"].get_vel(current_time_s_);

    double target_yaw = splines_["base_yaw"].get_pos(current_time_s_);
    double target_vw  = splines_["base_yaw"].get_vel(current_time_s_);

    // PD Error Calculation
    double yaw_err = target_yaw - current_yaw_;
    while (yaw_err > M_PI) yaw_err -= 2.0 * M_PI;
    while (yaw_err < -M_PI) yaw_err += 2.0 * M_PI;

    // Failsafe
    double pos_error = std::hypot(target_x - current_base_x_, target_y - current_base_y_);
    if (pos_error > 0.25 || std::abs(yaw_err) > 0.5) {
        RCLCPP_FATAL(get_logger(), "!!! CRITICAL SAFETY VIOLATION !!!");
        RCLCPP_FATAL(get_logger(), "Pos Error: %.3fm | Yaw Error: %.3frad", pos_error, std::abs(yaw_err));
        base_pub_->publish(geometry_msgs::msg::Twist());
        rclcpp::shutdown();
        return;
    }

    

    // PD Controller (Gains: kp=3.0, kd=0.1)
    double vx_world = target_vx + 3.0 * (target_x - current_base_x_) + 0.1 * (target_vx - current_vx_world_);
    double vy_world = target_vy + 3.0 * (target_y - current_base_y_) + 0.1 * (target_vy - current_vy_world_);

    

    const double MAX_VEL = 0.8; // m/s
    double current_speed = std::hypot(vx_world, vy_world);

    // Failsafe
    if (current_speed > MAX_VEL) {
        RCLCPP_FATAL(get_logger(), "Velocity Command Unsafe: %.3f m/s. Killing Node.", current_speed);
        base_pub_->publish(geometry_msgs::msg::Twist());
        rclcpp::shutdown();
        return;
    }

    // // --- UPDATED ARM CONTROL (Decoupled Frequency) ---
    // arm_pub_counter_++;
    // if (arm_pub_counter_ >= 20) { // Publish arm goal every 200ms
    //     arm_pub_counter_ = 0;

    //     auto traj_msg = trajectory_msgs::msg::JointTrajectory();
    //     traj_msg.joint_names = arm_joints_;
    //     traj_msg.header.stamp = this->now();

    //     trajectory_msgs::msg::JointTrajectoryPoint pnt;
        
    //     // Use a longer look-ahead for the hardware buffer (e.g., 200ms)
    //     double look_ahead = 0.1; 
    //     double eval_time = current_time_s_ + look_ahead;

    //     for (const auto& name : arm_joints_) {
    //         pnt.positions.push_back(splines_[name].get_pos(eval_time));
    //         pnt.velocities.push_back(splines_[name].get_vel(eval_time));
    //     }

    //     RCLCPP_INFO(get_logger(), "Sending Arm Lift: %.3f | Arm Flex: %.3f", pnt.positions[0], pnt.positions[1]);
        
    //     pnt.time_from_start = rclcpp::Duration::from_seconds(look_ahead);
    //     traj_msg.points.push_back(pnt);
        
    //     arm_pub_->publish(traj_msg);
    // }

    // --- ARM CONTROL (Synchronized) ---
    auto traj_msg = trajectory_msgs::msg::JointTrajectory();
    traj_msg.joint_names = arm_joints_;
    trajectory_msgs::msg::JointTrajectoryPoint pnt;
    for (const auto& name : arm_joints_) {
        pnt.positions.push_back(splines_[name].get_pos(current_time_s_));
        pnt.velocities.push_back(splines_[name].get_vel(current_time_s_)); // Pure rad/s
    }
    pnt.time_from_start = rclcpp::Duration::from_seconds(dt);
    traj_msg.points.push_back(pnt);
    arm_pub_->publish(traj_msg);

    // --- BASE PUBLISHING ---
    geometry_msgs::msg::Twist twist;
    twist.linear.x = vx_world * cos(target_yaw) + vy_world * sin(target_yaw);
    twist.linear.y = -vx_world * sin(target_yaw) + vy_world * cos(target_yaw);
    twist.angular.z = target_vw + (2.5 * yaw_err);
    
    base_pub_->publish(twist);
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<JointTrajectoryController>());
    rclcpp::shutdown();
    return 0;
}