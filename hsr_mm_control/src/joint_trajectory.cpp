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
        // "/switched_odom",
        "/omni_base_controller/wheel_odom",
         10,
        std::bind(&JointTrajectoryController::odom_callback, this, std::placeholders::_1)
    );

    // for testing 
    switched_odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/switched_odom", 10,
        [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
            // this->switched_x_ = msg->pose.pose.position.x;
            // this->switched_y_ = msg->pose.pose.position.y;
            current_base_x_ = msg->pose.pose.position.x;
            current_base_y_ = msg->pose.pose.position.y;
            
            this->switched_vx_ = msg->twist.twist.linear.x;
            this->switched_vy_ = msg->twist.twist.linear.y;
        }
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
    gripper_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>("/gripper_controller/joint_trajectory", 10);

    timer_ = this->create_wall_timer(std::chrono::milliseconds(10), std::bind(&JointTrajectoryController::timer_callback, this));

    arm_joints_ = {"arm_lift_joint", "arm_flex_joint", "arm_roll_joint", "wrist_flex_joint", "wrist_roll_joint"};

}

void JointTrajectoryController::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    // current_base_x_ = msg->pose.pose.position.x;
    // current_base_y_ = msg->pose.pose.position.y;
    wheel_base_x_ = msg->pose.pose.position.x;
    wheel_base_y_ = msg->pose.pose.position.y;

    current_vx_ = msg->twist.twist.linear.x;
    current_vy_ = msg->twist.twist.linear.y;
    current_vw_ = msg->twist.twist.angular.z;

    // Update yaw FIRST
    tf2::Quaternion q(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
                      msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
    double r, p, y;
    tf2::Matrix3x3(q).getRPY(r, p, y);
    current_yaw_ = y;

    // THEN rotate body velocities into world frame using correct yaw
    // double cy = std::cos(current_yaw_);
    // double sy = std::sin(current_yaw_);
    // current_vx_world_ = current_vx_ * cy - current_vy_ * sy;
    // current_vy_world_ = current_vx_ * sy + current_vy_ * cy;
}

void JointTrajectoryController::on_goal_recieved(const sensor_msgs::msg::JointState::SharedPtr msg) {
    std::unordered_map<std::string, double> goal_pos;
    std::unordered_map<std::string, double> goal_vel;

    for (size_t i = 0; i < msg->name.size(); ++i) {
        goal_pos[msg->name[i]] = msg->position[i];
        // Capture the target velocity sent by the sequencer
        if (msg->velocity.size() > i) {
            goal_vel[msg->name[i]] = msg->velocity[i];
        } else {
            goal_vel[msg->name[i]] = 0.0;
        }
    }

    if (goal_pos.count("hand_motor_joint")) {
        this->target_gripper_pos_ = goal_pos["hand_motor_joint"];
    }
    double gripper_drift = std::abs(this->target_gripper_pos_ - this->last_gripper_goal_);


    // 1. Check for Goal Drift or Velocity Change
    double goal_drift = std::hypot(goal_pos["base_x"] - last_goal_x_, goal_pos["base_y"] - last_goal_y_);
    double target_v_mag = std::hypot(goal_vel["base_x"], goal_vel["base_y"]);
    double vel_change = std::abs(target_v_mag - last_goal_vel_);

    // Re-solve if we moved OR if the speed target changed (important for leap-frogging)
    bool is_base_moving = (goal_drift >= 0.02);
    bool is_vel_changing = (vel_change >= 0.05);
    bool is_gripper_moving = (gripper_drift >= 0.01);

    if (!is_base_moving && !is_vel_changing && !is_gripper_moving) {
        // This will print once every 2 seconds
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000, 
            "Goal Ignored (No change): BaseDrift=%.3f, VelChange=%.3f, GripDrift=%.3f", 
            goal_drift, vel_change, gripper_drift);
        return;
    }

    // 2. Calculate Distance
    double dx = goal_pos["base_x"] - current_base_x_;
    double dy = goal_pos["base_y"] - current_base_y_;
    double L = std::hypot(dx, dy);

    // 3. PHYSICS-BASED TIME (The "Juggling" Fix)
    // We calculate T so the robot moves at a natural speed.
    // We use the max of current speed or target speed to ensure we don't stall.
    double v_start = std::hypot(current_vx_, current_vy_);
    double v_avg = (v_start + target_v_mag) / 2.0;
    v_avg = std::clamp(v_avg, 0.1, 0.25); // Minimum 0.1m/s to avoid infinite time

    // Golden Rule: Time = Distance / Speed
    double T_kinematic = L / v_avg;

    // Arm bottleneck
    double max_arm_delta = 0.0;
    for (const auto& name : arm_joints_) {
        max_arm_delta = std::max(max_arm_delta, std::abs(goal_pos[name] - current_arm_positions_[name]));
    }
    
    // Final T: At least 1.5s for stability, but matches physics for the sprint
    double T_dynamic = std::max({T_kinematic, max_arm_delta / 0.2, 1.5}); 
    

    double raw_goal_yaw = goal_pos["base_yaw"];
    double delta_yaw = raw_goal_yaw - current_yaw_;
    while (delta_yaw > M_PI)  delta_yaw -= 2.0 * M_PI;
    while (delta_yaw < -M_PI) delta_yaw += 2.0 * M_PI;
    double linearized_goal_yaw = current_yaw_ + delta_yaw;

    // 4. Solve Splines
    // Use the captured goal_vel for X, Y, and Yaw
    splines_["base_x"].solve(current_base_x_, goal_pos["base_x"], current_vx_, goal_vel["base_x"], 0, 0, T_dynamic);
    splines_["base_y"].solve(current_base_y_, goal_pos["base_y"], current_vy_, goal_vel["base_y"], 0, 0, T_dynamic);
    splines_["base_yaw"].solve(current_yaw_, linearized_goal_yaw, current_vw_, goal_vel["base_yaw"], 0, 0, T_dynamic);

    for (const auto& name : arm_joints_) {
        splines_[name].solve(current_arm_positions_[name], goal_pos[name], current_arm_velocities_[name], goal_vel[name], 0, 0, T_dynamic);
    }

    // 5. Update state
    last_goal_x_ = goal_pos["base_x"];
    last_goal_y_ = goal_pos["base_y"];
    last_goal_vel_ = target_v_mag;
    total_expected_time_ = T_dynamic;
    current_time_s_ = 0.0;
    last_t_ = this->now();
    is_executing_ = true;

    RCLCPP_INFO(get_logger(), "New Spline: Dist=%.2f, T=%.2f, V_end=%.2f", L, T_dynamic, target_v_mag);
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
    if (pos_error > 0.45 || std::abs(yaw_err) > 0.9) {
        RCLCPP_FATAL(get_logger(), "!!! CRITICAL SAFETY VIOLATION !!!");
        RCLCPP_FATAL(get_logger(), "Pos Error: %.3fm | Yaw Error: %.3frad", pos_error, std::abs(yaw_err));
        base_pub_->publish(geometry_msgs::msg::Twist());
        rclcpp::shutdown();
        return;
    }

    // --- CALCULATE ERRORS ---
    double err_x = target_x - current_base_x_;
    double err_y = target_y - current_base_y_;
    // double pos_error = std::hypot(err_x, err_y);

    // --- PRINT DEBUG EVERY 10 TICKS (100ms) ---
    static int print_count = 0;
    if (print_count++ >= 10) {
        print_count = 0;
        // Calculate Drifts
        double map_drift_x = wheel_base_x_ - current_base_x_;
        double map_drift_y = wheel_base_y_ - current_base_y_;
        double tracking_err_x = target_x - current_base_x_;
        double tracking_err_y = target_y - current_base_y_;

        RCLCPP_INFO(get_logger(), "========================================");
        RCLCPP_INFO(get_logger(), "TIME: %.2fs / %.2fs", current_time_s_, total_expected_time_);
        
        RCLCPP_INFO(get_logger(), "--- POSITION ---");
        RCLCPP_INFO(get_logger(), "  TARGET (Ghost):  X: %6.3f | Y: %6.3f", target_x, target_y);
        RCLCPP_INFO(get_logger(), "  CONTROL (Wheel): X: %6.3f | Y: %6.3f", wheel_base_x_, wheel_base_y_);
        RCLCPP_INFO(get_logger(), "  FUSED (Switched): X: %6.3f | Y: %6.3f", current_base_x_, current_base_y_);
        
        RCLCPP_INFO(get_logger(), "--- VELOCITY ---");
        RCLCPP_INFO(get_logger(), "  TARGET (Spline): VX: %6.3f | VY: %6.3f", target_vx, target_vy);
        RCLCPP_INFO(get_logger(), "  WHEEL (Stable):  VX: %6.3f | VY: %6.3f", current_vx_, current_vy_);
        RCLCPP_INFO(get_logger(), "  SWITCH (Spiky):  VX: %6.3f | VY: %6.3f", switched_vx_, switched_vy_);
        
        RCLCPP_INFO(get_logger(), "--- ERRORS / DRIFT ---");
        RCLCPP_INFO(get_logger(), "  TRACKING ERROR:  %6.3fm", std::hypot(tracking_err_x, tracking_err_y));
        RCLCPP_INFO(get_logger(), "  MAP DRIFT:       %6.3fm", std::hypot(map_drift_x, map_drift_y));
        
        if (std::abs(switched_vx_) > 2.0) {
            RCLCPP_WARN(get_logger(), "  [!] SWITCHED ODOM VELOCITY SPIKE DETECTED");
        }
        RCLCPP_INFO(get_logger(), "========================================");
    }

    

    // PD Controller (Gains: kp=3.0, kd=0.1)
    double vx_world = target_vx + 2.0 * err_x + 0.01 * (target_vx - current_vx_);
    double vy_world = target_vy + 2.0 * err_y + 0.01 * (target_vy - current_vy_);

    

    const double MAX_VEL = 2.2; // m/s
    double current_speed = std::hypot(vx_world, vy_world);

    // Failsafe
    if (current_speed > MAX_VEL) {
        RCLCPP_FATAL(get_logger(), "Velocity Command Unsafe: %.3f m/s. Killing Node.", current_speed);
        base_pub_->publish(geometry_msgs::msg::Twist());
        rclcpp::shutdown();
        return;
    }

    // --- UPDATED ARM CONTROL (Decoupled Frequency) ---
    arm_pub_counter_++;
    if (arm_pub_counter_ >= 5) { // Publish arm goal every 200ms
        arm_pub_counter_ = 0;

        auto traj_msg = trajectory_msgs::msg::JointTrajectory();
        traj_msg.joint_names = arm_joints_;
        traj_msg.header.stamp = this->now();

        trajectory_msgs::msg::JointTrajectoryPoint pnt;
        
        // Use a longer look-ahead for the hardware buffer (e.g., 200ms)
        double look_ahead = 0.05; 
        double eval_time = current_time_s_ + look_ahead;

        for (const auto& name : arm_joints_) {
            pnt.positions.push_back(splines_[name].get_pos(eval_time));
            // pnt.velocities.push_back(splines_[name].get_vel(eval_time));
        }

        // RCLCPP_INFO(get_logger(), "Sending Arm Lift: %.3f | Arm Flex: %.3f", pnt.positions[0], pnt.positions[1]);
        
        pnt.time_from_start = rclcpp::Duration::from_seconds(look_ahead);
        traj_msg.points.push_back(pnt);
        
        arm_pub_->publish(traj_msg);    
    }

    // // --- ARM CONTROL (Synchronized) ---
    // auto traj_msg = trajectory_msgs::msg::JointTrajectory();
    // traj_msg.joint_names = arm_joints_;
    // trajectory_msgs::msg::JointTrajectoryPoint pnt;
    // for (const auto& name : arm_joints_) {
    //     pnt.positions.push_back(splines_[name].get_pos(current_time_s_));
    //     pnt.velocities.push_back(splines_[name].get_vel(current_time_s_)); // Pure rad/s
    // }
    // pnt.time_from_start = rclcpp::Duration::from_seconds(dt);
    // traj_msg.points.push_back(pnt);
    // arm_pub_->publish(traj_msg);


    // --- GRIPPER COMMAND --- 
    if (std::abs(target_gripper_pos_ - last_gripper_goal_) > 0.01) {
        if (gripper_pub_->get_subscription_count() > 0) {
            auto grip_msg = trajectory_msgs::msg::JointTrajectory();
            grip_msg.joint_names = {"hand_motor_joint"};

            trajectory_msgs::msg::JointTrajectoryPoint grip_pnt;
            grip_pnt.positions.push_back(this->target_gripper_pos_);
            grip_pnt.time_from_start = rclcpp::Duration::from_seconds(1.0); 

            grip_msg.points.push_back(grip_pnt);
            gripper_pub_->publish(grip_msg);

            last_gripper_goal_ = target_gripper_pos_;
            RCLCPP_INFO(this->get_logger(), "SUCCESS: Gripper Command Sent & Received by Controller: %.3f", target_gripper_pos_);
        } else {
            // Log this so you can see if the network is the bottleneck
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
                                "Waiting for gripper controller subscription...");
        }
    }

    double c = std::cos(current_yaw_);
    double s = std::sin(current_yaw_);

    // --- BASE PUBLISHING ---
    geometry_msgs::msg::Twist twist;
    // twist.linear.x = vx_world * cos(target_yaw) + vy_world * sin(target_yaw);
    // twist.linear.y = -vx_world * sin(target_yaw) + vy_world * cos(target_yaw);
    twist.linear.x =  vx_world * c + vy_world * s;
    twist.linear.y = -vx_world * s + vy_world * c;
    twist.angular.z = target_vw + (2.5 * yaw_err);
    
    base_pub_->publish(twist);
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<JointTrajectoryController>());
    rclcpp::shutdown();
    return 0;
}