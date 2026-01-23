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
        "/odom", 10,
        std::bind(&JointTrajectoryController::odom_callback, this, std::placeholders::_1)
    );

    arm_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>("/arm_trajectory_controller/joint_trajectory", 10);
    base_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/omni_base_controller/cmd_vel", 10);

    timer_ = this->create_wall_timer(std::chrono::milliseconds(20), std::bind(&JointTrajectoryController::timer_callback, this));

    arm_joints_ = {"arm_lift_joint", "arm_flex_joint", "arm_roll_joint", "wrist_flex_joint", "wrist_roll_joint"};

}

void JointTrajectoryController::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    tf2::Quaternion q(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
    msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
    double r, p, y;
    tf2::Matrix3x3(q).getRPY(r, p, y);
    current_yaw_ = y;
}

void JointTrajectoryController::on_goal_recieved(const sensor_msgs::msg::JointState::SharedPtr msg) {
    if (is_executing_) return;

    for (size_t i = 0; i < msg->name.size(); ++i) {
        double q0 = 0.0;
        splines_[msg->name[i]].solve(q0, msg->position[i], 0, 0, 0, 0, duration_);
    }

    start_time_ = this->now();
    is_executing_ = true;
    RCLCPP_INFO(get_logger(), "Executing smooth whole body trajectory...");
}

void JointTrajectoryController::timer_callback() {
    if (!is_executing_) return;

    double t = (this->now() - start_time_).seconds();
    if (t > duration_) {
        is_executing_ = false;
        base_pub_->publish(geometry_msgs::msg::Twist());
        return;
    }

    auto traj_msg = trajectory_msgs::msg::JointTrajectory();
    traj_msg.joint_names = arm_joints_;
    trajectory_msgs::msg::JointTrajectoryPoint pnt;
    for (const auto& name : arm_joints_) {
        pnt.positions.push_back(splines_[name].get_pos(t));
        pnt.velocities.push_back(splines_[name].get_vel(t));
    }
    pnt.time_from_start = rclcpp::Duration::from_seconds(0.02);
    traj_msg.points.push_back(pnt);
    arm_pub_->publish(traj_msg);

    double vx_world = splines_["base_x"].get_vel(t);
    double vy_world = splines_["base_y"].get_vel(t);
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500, "Base World Vel: X=%.2f", vx_world);

    geometry_msgs::msg::Twist twist;
    twist.linear.x = vx_world*cos(current_yaw_) + vy_world*sin(current_yaw_);
    twist.linear.y = -vx_world*sin(current_yaw_) + vy_world*cos(current_yaw_);
    twist.angular.z = splines_["base_yaw"].get_vel(t);
    base_pub_->publish(twist);
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<JointTrajectoryController>());
    rclcpp::shutdown();
    return 0;
}