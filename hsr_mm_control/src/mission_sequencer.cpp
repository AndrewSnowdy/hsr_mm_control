#include "hsr_mm_control/mission_sequencer.hpp"

using namespace std::chrono_literals;

MissionSequencer::MissionSequencer() : Node("mission_sequencer"), state_(MissionState::APPROACH) {
    // Publisher to send the (X,Y,Z) target to the IK Solver
    target_pub_ = this->create_publisher<geometry_msgs::msg::Point>("/waypoint_target", 10);
    
    // Subscriber to monitor where the robot actually is
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10, [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
            current_x_ = msg->pose.pose.position.x;
            current_y_ = msg->pose.pose.position.y;
        });

    timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&MissionSequencer::state_machine_timer, this));
    
    // Define our Hardcoded Button Location
    button_x = 2.44; // From your Gazebo SDF
    button_y = 0.0;
    button_z = 1.0;
}

bool MissionSequencer::wait_for(double seconds) {
    if (!timer_started_) {
        start_time_ = this->now();
        timer_started_ = true;
        return false;
    }

    if ((this->now() - start_time_).seconds() >= seconds) {
        timer_started_ = false; // Reset for next use
        return true;
    }
    return false;
}

void MissionSequencer::state_machine_timer() {
    geometry_msgs::msg::Point target_msg;

    switch (state_) {
        case MissionState::APPROACH:
            // Waypoint 1: Move base close (0.8m away), arm tucked low (z=0.4)
            target_msg.x = button_x - 0.8;
            target_msg.y = button_y;
            target_msg.z = 0.4; 
            target_pub_->publish(target_msg);

            if (is_at_goal(target_msg.x, target_msg.y, 0.05)) {
                RCLCPP_INFO(this->get_logger(), "Arrived at APPROACH. Transitioning to READY.");
                state_ = MissionState::READY;
            }
            break;

        case MissionState::READY:
            // Waypoint 2: Arm out to button height, 10cm away
            target_msg.x = button_x - 0.1;
            target_msg.y = button_y;
            target_msg.z = button_z;
            target_pub_->publish(target_msg);

            // Here we check if the arm is extended (You can add joint check later)
            // For now, let's wait 3 seconds to ensure the spline finishes
            if (wait_for(3.0)) state_ = MissionState::PRESS;
            break;

        case MissionState::PRESS:
            // Waypoint 3: The "Dumb Push" - 2cm past the button surface
            target_msg.x = button_x + 0.02;
            target_msg.y = button_y;
            target_msg.z = button_z;
            target_pub_->publish(target_msg);

            if (wait_for(2.0)) state_ = MissionState::RETRACT;
            break;

        case MissionState::RETRACT:
            // Final Waypoint: Pull back and tuck
            target_msg.x = button_x - 0.5;
            target_msg.y = button_y;
            target_msg.z = 0.5;
            target_pub_->publish(target_msg);
            state_ = MissionState::DONE;
            break;
            
        case MissionState::DONE:
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Sequence Finished.");
            break;
    }
}

bool MissionSequencer::is_at_goal(double tx, double ty, double tol) {
    return std::hypot(tx - current_x_, ty - current_y_) < tol;
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MissionSequencer>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}