#include "hsr_mm_control/mission_sequencer.hpp"

using namespace std::chrono_literals;


MissionSequencer::MissionSequencer()
: Node("mission_sequencer"),
    simple_state_(SimpleState::MANUAL)
{
    mode_pub_ = this->create_publisher<std_msgs::msg::Bool>("/use_ik_mode", 10);
    target_pub_ = this->create_publisher<geometry_msgs::msg::Pose>("/waypoint_target", 10);

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    timer_ = this->create_wall_timer(100ms, std::bind(&MissionSequencer::simple_timer, this));

    // We can keep a simplified Joy sub just as an Emergency Stop
    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
        "/joy", 10, [this](const sensor_msgs::msg::Joy::SharedPtr msg) {
            if (msg->buttons[1] == 1) { // 'B' button
                RCLCPP_ERROR(this->get_logger(), "EMERGENCY STOP: Reverting to MANUAL");
                simple_state_ = SimpleState::MANUAL;
            }
        });

    RCLCPP_INFO(this->get_logger(), "MissionSequencer initialized. Waiting for /initial_mission_pose...");

    
    mission_sub_ = this->create_subscription<geometry_msgs::msg::Pose>(
        "/initial_mission_pose", 10, 
        [this](const geometry_msgs::msg::Pose::SharedPtr msg) {
            if (simple_state_ == SimpleState::MANUAL || simple_state_ == SimpleState::DONE) {
                button_x = msg->position.x;
                button_y = msg->position.y;
                button_z = msg->position.z;
                RCLCPP_INFO(this->get_logger(), "MISSION START: Target [%.2f, %.2f, %.2f]", 
                            button_x, button_y, button_z);
                simple_state_ = SimpleState::APPROACH;
            }
        });
    // Button location (odom frame)
    // button_x = 2.44;
    // button_y = 0.0;
    // button_z = 1.0;

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

    double current_x, current_y, current_z, current_yaw;
    if (!get_tf_xyz("map", base_frame_, current_x, current_y, current_z) || 
        !get_base_yaw(current_yaw)) {
        return; // Don't act if TF is missing
    }

    auto set_yaw = [](geometry_msgs::msg::Pose &pose, double yaw) {
        tf2::Quaternion q;
        q.setRPY(0, 0, yaw);
        pose.orientation.x = q.x();
        pose.orientation.y = q.y();
        pose.orientation.z = q.z();
        pose.orientation.w = q.w();
    };


    switch (simple_state_) {

        case SimpleState::MANUAL: {
            // In Manual, we don't publish target_pose.
            // This ensures the JointTrajectoryController doesn't move the robot.

            // ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/omni_base_controller/cmd_vel
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000, 
                                "MANUAL MODE: System waiting for 'A' button trigger.");
            
            // We publish mode = false just to be safe (base mode)
            mode_msg.data = false;
            mode_pub_->publish(mode_msg);
            break; 
        }
        
        case SimpleState::APPROACH: {
            double rx, ry, rz;
            if (!get_tf_xyz("map", base_frame_, rx, ry, rz)) return;

                const double r_standoff = 0.80;      // pick 0.7–1.2 typically
                const double tol_xy     = 0.10;      // standoff acceptance
                const double tol_yaw    = 0.25;      // ~15 deg

                auto goal = compute_standoff_goal(button_x, button_y, rx, ry, r_standoff);
                target_pub_->publish(goal);

                // check convergence (in map frame!)
                double gx = goal.position.x;
                double gy = goal.position.y;

                // extract yaw from goal
                tf2::Quaternion q(goal.orientation.x, goal.orientation.y,
                                    goal.orientation.z, goal.orientation.w);
                double gr, gp, gyaw;
                tf2::Matrix3x3(q).getRPY(gr, gp, gyaw);

                if (base_close_xyw(gx, gy, gyaw, tol_xy, tol_yaw)) {
                    RCLCPP_INFO(this->get_logger(), "Standoff reached -> PRE_PRESS");
                    simple_state_ = SimpleState::PRE_PRESS;
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
                simple_state_ = SimpleState::PRESS;
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
            if (base_close_xyw(target_pose.position.x, target_pose.position.y, target_yaw, 0.05, 0.015)) {
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

geometry_msgs::msg::Pose MissionSequencer::compute_standoff_goal(
    double bx, double by,
    double rx, double ry,
    double r_standoff)
{
  geometry_msgs::msg::Pose goal;

  // 1) direction from button -> robot
  double vx = rx - bx;
  double vy = ry - by;
  double d  = std::hypot(vx, vy);

  // guard against divide-by-zero (robot exactly at button)
  if (d < 1e-3) {
    // pick any direction (e.g., +x)
    vx = 1.0; vy = 0.0; d = 1.0;
  }

  // 2) unit vector
  double ux = vx / d;
  double uy = vy / d;

  // 3) standoff point on ring
  goal.position.x = bx + r_standoff * ux;
  goal.position.y = by + r_standoff * uy;
  goal.position.z = 0.0; // base goal, z unused

  // 4) face the button
  double yaw = std::atan2(by - goal.position.y, bx - goal.position.x);

  tf2::Quaternion q;
  q.setRPY(0, 0, yaw);
  goal.orientation.x = q.x();
  goal.orientation.y = q.y();
  goal.orientation.z = q.z();
  goal.orientation.w = q.w();

  return goal;
}


int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MissionSequencer>());
    rclcpp::shutdown();
    return 0;
}
