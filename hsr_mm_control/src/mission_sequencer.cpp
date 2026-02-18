#include "hsr_mm_control/mission_sequencer.hpp"
#include "hsr_mm_control/utils.hpp"

using namespace std::chrono_literals;


MissionSequencer::MissionSequencer()
: Node("mission_sequencer"),
    simple_state_(SimpleState::MANUAL)
{
    mode_pub_ = this->create_publisher<std_msgs::msg::Bool>("/use_ik_mode", 10);
    target_pub_ = this->create_publisher<geometry_msgs::msg::Pose>("/waypoint_target", 10);
    marker_array_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("feasible_samples", 10);

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

    auto costmap_qos = rclcpp::QoS(rclcpp::KeepLast(1))
        .reliable()
        .transient_local(); // Crucial for Nav2 Costmaps

    costmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>( // Change type here
        "/local_costmap/costmap",
        costmap_qos,
        [this](const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
            latest_costmap_ = *msg;
            have_costmap_ = true;
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
    if (!get_tf_xyz(map_frame_, base_frame_, current_x, current_y, current_z) || 
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
            // Current base pose in map
            double rx, ry, rz;
            if (!get_tf_xyz(map_frame_, base_frame_, rx, ry, rz)) return;

            const double r_standoff = 1.25;
            const double tol_xy     = 0.10;
            const double tol_yaw    = 0.25;

            // Compute final standoff goal (on ring facing the button)
            auto final_goal = compute_standoff_goal(button_x, button_y, rx, ry, r_standoff);

            target_pub_->publish(final_goal);

            // Transition based on reaching the *final* goal, not the sub-goal
            tf2::Quaternion q(final_goal.orientation.x, final_goal.orientation.y,
                            final_goal.orientation.z, final_goal.orientation.w);
            double gr, gp, final_yaw;
            tf2::Matrix3x3(q).getRPY(gr, gp, final_yaw);

            if (base_close_xyw(final_goal.position.x, final_goal.position.y, final_yaw, tol_xy, tol_yaw)) {
                RCLCPP_INFO(this->get_logger(), "Standoff reached -> PRE_PRESS");
                simple_state_ = SimpleState::PRE_PRESS;
            }

            break;
        }


        case SimpleState::PRE_PRESS: {
            mode_msg.data = true; // Switch to IK mode
            mode_pub_->publish(mode_msg);

            if (!have_costmap_) {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, 
                                    "Waiting for costmap on topic: /local_costmap/costmap...");
                return; // Exit early since we can't calculate a standoff without a map
            }

            // 1) Compute feasible standoff ONCE
            if (have_costmap_ && !have_standoff_) {
                geometry_msgs::msg::Pose result_pose;

                double rx, ry, rz;
                if (!get_tf_xyz(map_frame_, base_frame_, rx, ry, rz)) return;

                bool success = standoff_utils::compute_feasible_standoff(
                    button_x, button_y, rx, ry, // Pass robot position
                    latest_costmap_, 0.80, 72, 15,
                    result_pose, debug_feasible_poses_);

                if (success) {
                    cached_standoff_ = result_pose;
                    have_standoff_ = true;
                    RCLCPP_INFO(this->get_logger(), "Feasible standoff locked.");
                    publish_feasible_cloud(debug_feasible_poses_);
                } else {
                    RCLCPP_ERROR(this->get_logger(), "No safe standoff found! Aborting to MANUAL.");
                    simple_state_ = SimpleState::MANUAL;
                    break;
                }
            }

            // 2) Extract the pre-calculated yaw from the cached pose
            tf2::Quaternion q(cached_standoff_.orientation.x, cached_standoff_.orientation.y,
                            cached_standoff_.orientation.z, cached_standoff_.orientation.w);
            double r, p, press_yaw;
            tf2::Matrix3x3(q).getRPY(r, p, press_yaw);

            // 3) Offset the EE slightly from the button along that same yaw
            const double ee_standoff_dist = 0.15;
            target_pose.position.x = button_x - ee_standoff_dist * std::cos(press_yaw);
            target_pose.position.y = button_y - ee_standoff_dist * std::sin(press_yaw);
            target_pose.position.z = button_z;
            set_yaw(target_pose, press_yaw);

            target_pub_->publish(target_pose);

            // 4) Check for convergence
            if (ee_close_xyz(target_pose.position.x, target_pose.position.y, target_pose.position.z, 0.02)) {
                simple_state_ = SimpleState::PRESS;
                RCLCPP_INFO(this->get_logger(), "PRE_PRESS -> PRESS");
            }
            break;
        }

        case SimpleState::PRESS: {
            mode_msg.data = true; // IK mode
            mode_pub_->publish(mode_msg);

            // 1) Extract the yaw from the standoff we validated in PRE_PRESS
            tf2::Quaternion q(cached_standoff_.orientation.x, cached_standoff_.orientation.y,
                            cached_standoff_.orientation.z, cached_standoff_.orientation.w);
            double r, p, press_yaw;
            tf2::Matrix3x3(q).getRPY(r, p, press_yaw);

            // 2) Move "deeper" into the button. 
            // We change the offset from 0.15 (standoff) to 0.05 (pressing depth)
            const double push_depth = 0.025; 
            target_pose.position.x = button_x - push_depth * std::cos(press_yaw);
            target_pose.position.y = button_y - push_depth * std::sin(press_yaw);
            
            // Keep your hardcoded Z-offset for a slight downward "press" feel
            target_pose.position.z = button_z - 0.05; 
            
            set_yaw(target_pose, press_yaw);
            target_pub_->publish(target_pose);

            // 3) Check for convergence
            if (ee_close_xyz(target_pose.position.x, target_pose.position.y, target_pose.position.z, 0.025)) {
                simple_state_ = SimpleState::RETRACT;
                RCLCPP_INFO(this->get_logger(), "PRESS -> RETRACT");
            }
            break;
        }

        case SimpleState::RETRACT: {
            mode_msg.data = false; // Base control mode
            mode_pub_->publish(mode_msg);

            // 1. Get the original press heading
            tf2::Quaternion q_standoff(cached_standoff_.orientation.x, cached_standoff_.orientation.y,
                                    cached_standoff_.orientation.z, cached_standoff_.orientation.w);
            double r, p, press_yaw;
            tf2::Matrix3x3(q_standoff).getRPY(r, p, press_yaw);

            // 2. Define the target heading (Pivot 40-60 degrees toward the door)
            // Using your M_PI * 2 / 9.0 (40 degrees) logic
            double target_yaw = press_yaw + (M_PI * 2.0 / 9.0); 
            while (target_yaw > M_PI) target_yaw -= 2.0 * M_PI;
            while (target_yaw < -M_PI) target_yaw += 2.0 * M_PI;

            // 3. Move back to a "Safety Distance" (0.6m is usually enough to rotate safely)
            const double safety_dist = 0.60;
            target_pose.position.x = button_x - safety_dist * std::cos(press_yaw);
            target_pose.position.y = button_y - safety_dist * std::sin(press_yaw);
            target_pose.position.z = button_z; // Keep arm height steady
            
            // Face the target yaw while backing up
            set_yaw(target_pose, target_yaw);
            target_pub_->publish(target_pose);

            // 4. Convergence Check
            // Use a slightly looser tolerance for the exit transition
            if (base_close_xyw(target_pose.position.x, target_pose.position.y, target_yaw, 0.08, 0.15)) {
                simple_state_ = SimpleState::DONE;
                RCLCPP_INFO(this->get_logger(), "RETRACT -> EXIT (Safe pivot achieved)");
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

void MissionSequencer::publish_feasible_cloud(const std::vector<geometry_msgs::msg::Pose>& poses)
{
    visualization_msgs::msg::MarkerArray arr;

    // Clear previous
    visualization_msgs::msg::Marker clear;
    clear.action = visualization_msgs::msg::Marker::DELETEALL;
    arr.markers.push_back(clear);

    int id = 0;
    for (const auto& p : poses) {
        visualization_msgs::msg::Marker m;
        // Use the frame the costmap lives in (likely "odom")
        m.header.frame_id = latest_costmap_.header.frame_id; 
        m.header.stamp = this->now();
        m.ns = "feasible_points";
        m.id = id++;
        m.type = visualization_msgs::msg::Marker::SPHERE;
        m.action = visualization_msgs::msg::Marker::ADD;

        m.pose = p;
        m.pose.position.z = 0.05; 
        m.scale.x = 0.08; m.scale.y = 0.08; m.scale.z = 0.08;

        m.color.r = 0.0f; m.color.g = 1.0f; m.color.b = 0.0f; m.color.a = 0.8f;
        arr.markers.push_back(m);
    }
    marker_array_pub_->publish(arr);
}

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MissionSequencer>());
    rclcpp::shutdown();
    return 0;
}
