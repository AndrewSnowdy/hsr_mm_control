#include "hsr_mm_control/mission_sequencer.hpp"

using namespace std::chrono_literals;


MissionSequencer::MissionSequencer()
: Node("mission_sequencer"),
    simple_state_(SimpleState::PRE_PRESS)
{
    mode_pub_ = this->create_publisher<std_msgs::msg::Bool>("/use_ik_mode", 10);
    target_pub_ = this->create_publisher<geometry_msgs::msg::Pose>("/waypoint_target", 10);

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    timer_ = this->create_wall_timer(100ms, std::bind(&MissionSequencer::simple_timer, this));

    // Button location (odom frame)
    // button_x = 2.44;
    button_x = 1.15;
    button_y = 0.0;
    button_z = 1.0;

    RCLCPP_INFO(this->get_logger(), "MissionSequencer started (MINIMAL).");
}

bool MissionSequencer::get_tf_xyz(const std::string& parent,
                                  const std::string& child,
                                  double &x, double &y, double &z)
{
  try {
    auto tf = tf_buffer_->lookupTransform(parent, child, tf2::TimePointZero);
    x = tf.transform.translation.x;
    y = tf.transform.translation.y;
    z = tf.transform.translation.z;
    return std::isfinite(x) && std::isfinite(y) && std::isfinite(z);
  } catch (const tf2::TransformException &ex) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                         "TF lookup failed (%s -> %s): %s",
                         parent.c_str(), child.c_str(), ex.what());
    return false;
  }
}

bool MissionSequencer::get_base_yaw(double &yaw)
{
    try {
        auto tf = tf_buffer_->lookupTransform(odom_frame_, base_frame_, tf2::TimePointZero);
        tf2::Quaternion q(
        tf.transform.rotation.x, tf.transform.rotation.y,
        tf.transform.rotation.z, tf.transform.rotation.w);
        
        double r, p;
        tf2::Matrix3x3(q).getRPY(r, p, yaw);
        return true;
    } catch (const tf2::TransformException &ex) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                            "Yaw lookup failed: %s", ex.what());
        return false;
    }
}


bool MissionSequencer::base_close_xyw(double tx, double ty, double tw, double tol_xy, double tol_w)
{
    double x, y, z, current_yaw;

    // 1. Safe Lookups: If TF is lagging, we return false immediately
    if (!get_tf_xyz(odom_frame_, base_frame_, x, y, z)) return false;
    if (!get_base_yaw(current_yaw)) return false;

    // 2. Calculate Translation Error
    const double dist_err = std::hypot(tx - x, ty - y);

    // 3. Calculate Orientation Error (Shortest Path)
    double yaw_err = tw - current_yaw;
    while (yaw_err > M_PI)  yaw_err -= 2.0 * M_PI;
    while (yaw_err < -M_PI) yaw_err += 2.0 * M_PI;
    yaw_err = std::abs(yaw_err);

    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                        "BASE Tracking: DistErr=%.3fm, YawErr=%.3frad", dist_err, yaw_err);

    // 4. Convergence Check
    return (dist_err < tol_xy) && (yaw_err < tol_w);
}

bool MissionSequencer::ee_close_xyz(double tx, double ty, double tz, double tol_xyz)
{
    double x, y, z;
    if (!get_tf_xyz(odom_frame_, ee_frame_, x, y, z)) return false;
    const double dx = tx - x, dy = ty - y, dz = tz - z;
    const double err = std::sqrt(dx*dx + dy*dy + dz*dz);
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                        "EE err=%.3f (tol=%.3f) ee=[%.2f %.2f %.2f]",
                        err, tol_xyz, x, y, z);
    return err < tol_xyz;
}

void MissionSequencer::simple_timer()
{
    geometry_msgs::msg::Pose target_pose;
    std_msgs::msg::Bool mode_msg;

    auto set_yaw = [](geometry_msgs::msg::Pose &pose, double yaw) {
        tf2::Quaternion q;
        q.setRPY(0, 0, yaw);
        pose.orientation.x = q.x();
        pose.orientation.y = q.y();
        pose.orientation.z = q.z();
        pose.orientation.w = q.w();
    };


    switch (simple_state_) {
        case SimpleState::APPROACH: {
            mode_msg.data = false;               // base mode
            mode_pub_->publish(mode_msg);

            // Drive near the button
            target_pose.position.x = button_x - 1.5;
            target_pose.position.y = button_y;
            //   target_pose.position.z = 0.4; //does not matter because in base mode
            double target_yaw = 0.0; 
            set_yaw(target_pose, target_yaw);
            target_pub_->publish(target_pose);

            // Use a forgiving tolerance first
            if (base_close_xyw(target_pose.position.x, target_pose.position.y, target_yaw, 0.05, 0.1)) {
                simple_state_ = SimpleState::PRE_PRESS;
                RCLCPP_INFO(this->get_logger(), "APPROACH -> PRE_PRESS");
            }
            break;
        }
        case SimpleState::PRE_PRESS: {
            mode_msg.data = true; // Switch to IK mode
            mode_pub_->publish(mode_msg);

            // Move EE to a "Standoff" position 10cm away from the button
            target_pose.position.x = button_x - 0.15; 
            target_pose.position.y = button_y;
            target_pose.position.z = button_z;
            set_yaw(target_pose, 0.0);
            target_pub_->publish(target_pose);

            // Wait for high precision before the final push
            if (ee_close_xyz(target_pose.position.x, target_pose.position.y, target_pose.position.z, 0.02)) {
                simple_state_ = SimpleState::DONE;
                RCLCPP_INFO(this->get_logger(), "PRE_PRESS -> PRESS");
            }
            break;
        }

        case SimpleState::PRESS: {
            mode_msg.data = true;                // IK mode
            mode_pub_->publish(mode_msg);

            // Just go to the press goal directly
            target_pose.position.x = button_x - 0.08;
            target_pose.position.y = button_y;
            target_pose.position.z = button_z - 0.02;
            set_yaw(target_pose, 0.0);
            target_pub_->publish(target_pose);

            // Use something realistic (1–3cm)
            if (ee_close_xyz(target_pose.position.x, target_pose.position.y, target_pose.position.z, /*tol_xyz=*/0.02)) {
                simple_state_ = SimpleState::RETRACT;
                RCLCPP_INFO(this->get_logger(), "PRESS -> RETRACT");
            }
            break;
        }

        case SimpleState::RETRACT: {
            mode_msg.data = false; // Keep IK mode for precise arm movement
            mode_pub_->publish(mode_msg);

            // Back away from the button (move back 30cm and slightly up)
            target_pose.position.x = button_x - 0.80; 
            target_pose.position.y = button_y;
            target_pose.position.z = button_z + 0.10; 
            double target_yaw = 1.57; 
            set_yaw(target_pose, target_yaw);
            target_pub_->publish(target_pose);

            // Check if we've backed away enough
            if (base_close_xyw(target_pose.position.x, target_pose.position.y, target_yaw, 0.05, 0.1)) {
                simple_state_ = SimpleState::EXIT;
                RCLCPP_INFO(this->get_logger(), "RETRACT -> EXIT");
            }
            break;
        }
        case SimpleState::EXIT: {
            mode_msg.data = false; 
            mode_pub_->publish(mode_msg);

            // Waypoint: Drive to the center of the gap
            // Side wall tip is at X=0.5, Front wall is at X=2.5. Midpoint X = 1.5.
            // Move Y to 2.0 to clear the hallway
            double tx = 1.5; 
            double ty = 1.5; 
            double tw = 1.5708; // Face "North" (toward the gap)

            target_pose.position.x = tx;
            target_pose.position.y = ty;
            set_yaw(target_pose, tw);
            target_pub_->publish(target_pose);

            if (base_close_xyw(tx, ty, tw, 0.15, 0.2)) {
                simple_state_ = SimpleState::DONE;
                RCLCPP_INFO(this->get_logger(), "EXIT -> DONE (Gap cleared)");
            }
            break;
        }

        case SimpleState::DONE: {
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                "DONE (holding).");
            break;
        }
    }
}

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MissionSequencer>());
    rclcpp::shutdown();
    return 0;
}
