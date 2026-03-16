#include "hsr_mm_control/mission_sequencer.hpp"
#include "hsr_mm_control/utils_feasible_points.hpp"

using namespace std::chrono_literals;


geometry_msgs::msg::Pose interpolate_pose(const geometry_msgs::msg::Pose& start, const geometry_msgs::msg::Pose& end, double ratio) {
    geometry_msgs::msg::Pose p = end;
    p.position.x = start.position.x + ratio * (end.position.x - start.position.x);
    p.position.y = start.position.y + ratio * (end.position.y - start.position.y);
    return p;
}


MissionSequencer::MissionSequencer()
: Node("mission_sequencer"),
    simple_state_(SimpleState::MANUAL)
{
    // mode_pub_ = this->create_publisher<std_msgs::msg::Bool>("/use_ik_mode", 10);
    // target_pub_ = this->create_publisher<geometry_msgs::msg::Pose>("/waypoint_target", 10);
    // marker_array_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("feasible_samples", 10);
    mission_pub_ = this->create_publisher<hsr_mm_control::msg::MissionGoal>("/mission_command", 10);

    door_lock_pub_ = this->create_publisher<std_msgs::msg::Bool>("/door_lock_trigger", 10);

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    timer_ = this->create_wall_timer(100ms, std::bind(&MissionSequencer::simple_timer, this));

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


    marker_sub_ = this->create_subscription<visualization_msgs::msg::MarkerArray>(
    "/visualization_marker", 10,
    [this](const visualization_msgs::msg::MarkerArray::SharedPtr msg) {
        // Just save the data; let the functions do the heavy lifting
        latest_markers_ = *msg; 
    });

    RCLCPP_INFO(this->get_logger(), "MissionSequencer started.");
}
// --------------
// Publish Parser
// --------------
void MissionSequencer::publish_mission_goal(const geometry_msgs::msg::Pose& pose, bool ik_mode, double cruise_speed, double door_yaw) 
{
    hsr_mm_control::msg::MissionGoal goal_msg;
    goal_msg.target_pose = pose;
    goal_msg.use_ik_mode = ik_mode;

    // Decompose speed into world X and Y based on door orientation
    if (cruise_speed > 0.0) {
        goal_msg.target_velocity.linear.x = cruise_speed * std::cos(door_yaw);
        goal_msg.target_velocity.linear.y = cruise_speed * std::sin(door_yaw);
    } else {
        goal_msg.target_velocity.linear.x = 0.0;
        goal_msg.target_velocity.linear.y = 0.0;
    }

    mission_pub_->publish(goal_msg);
}

// ----------------------
// TF setting and getting
// ----------------------
std::optional<RobotState> MissionSequencer::get_base_position() {
    try {
        // Look up the transform once
        auto tf = tf_buffer_->lookupTransform(odom_frame_, base_frame_, tf2::TimePointZero);
        
        RobotState state;
        state.x = tf.transform.translation.x;
        state.y = tf.transform.translation.y;
        state.z = tf.transform.translation.z;

        // Extract yaw
        tf2::Quaternion q(tf.transform.rotation.x, tf.transform.rotation.y,
                        tf.transform.rotation.z, tf.transform.rotation.w);
        double r, p;
        tf2::Matrix3x3(q).getRPY(r, p, state.yaw);

        // Fill the pose object for functions that need a Pose msg
        state.pose.position.x = state.x;
        state.pose.position.y = state.y;
        state.pose.position.z = state.z;
        state.pose.orientation = tf.transform.rotation;

        return state;
    } catch (const tf2::TransformException &ex) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "TF Error: %s", ex.what());
        return std::nullopt;
    }
}

std::optional<geometry_msgs::msg::Point> MissionSequencer::get_ee_position() {
    try {
        // We use odom_frame_ as the reference to match your original logic
        auto tf = tf_buffer_->lookupTransform(odom_frame_, ee_frame_, tf2::TimePointZero);
        geometry_msgs::msg::Point p;
        p.x = tf.transform.translation.x;
        p.y = tf.transform.translation.y;
        p.z = tf.transform.translation.z;
        return p;
    } catch (const tf2::TransformException &ex) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "EE TF Error: %s", ex.what());
        return std::nullopt;
    }
}

void MissionSequencer::set_yaw(geometry_msgs::msg::Pose &pose, double yaw) {
    tf2::Quaternion q;
    q.setRPY(0, 0, yaw);
    pose.orientation.x = q.x();
    pose.orientation.y = q.y();
    pose.orientation.z = q.z();
    pose.orientation.w = q.w();
}


// ---------------------------
// Checking waypoint tolerance
//----------------------------
bool MissionSequencer::base_close_xyw(const RobotState& state, const geometry_msgs::msg::Pose& target, double tol_xy, double tol_w) 
{
    const double dist_err = std::hypot(target.position.x - state.x, target.position.y - state.y);

    // converting Quat -> Euler
    tf2::Quaternion q(target.orientation.x, target.orientation.y, 
                    target.orientation.z, target.orientation.w);
    double r, p, target_yaw;
    tf2::Matrix3x3(q).getRPY(r, p, target_yaw);

    // SE3
    double yaw_err = target_yaw - state.yaw;
    while (yaw_err > M_PI)  yaw_err -= 2.0 * M_PI;
    while (yaw_err < -M_PI) yaw_err += 2.0 * M_PI;
    
    yaw_err = std::abs(yaw_err);

    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                        "BASE Tracking: DistErr=%.3fm, YawErr=%.3frad", dist_err, yaw_err);

    return (dist_err < tol_xy) && (yaw_err < tol_w);
}

bool MissionSequencer::ee_close_xyz(const geometry_msgs::msg::Point& current_pos, double tx, double ty, double tz, double tol_xyz)
{
    const double dx = tx - current_pos.x;
    const double dy = ty - current_pos.y;
    const double dz = tz - current_pos.z;
    
    const double err = std::sqrt(dx*dx + dy*dy + dz*dz);

    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                        "EE Tracking: Err=%.3fm (tol=%.3fm)", err, tol_xyz);

    return err < tol_xyz;
}


// --------
// Main FSM
// --------
void MissionSequencer::simple_timer()
{
    geometry_msgs::msg::Pose target_pose;
    std_msgs::msg::Bool mode_msg;

    // TF calls
    auto robot = get_base_position();
    auto ee_pos = get_ee_position();
    if (!robot || !ee_pos) return;

    switch (simple_state_) {

        case SimpleState::MANUAL: {
            auto button = get_closest_button();
            if (button) {
                button_x = button->position.x;
                button_y = button->position.y;
                button_z = button->position.z;
                simple_state_ = SimpleState::APPROACH;
                RCLCPP_INFO(this->get_logger(), "Button found. Moving to APPROACH.");
            }
            break;
        }
        
        // case SimpleState::APPROACH: {
        //     auto final_goal = feasible_standoff_utils::compute_circular_target(button_x, button_y, robot->x, robot->y, 1.25);
            
        //     publish_mission_goal(final_goal, false, 0.0, 0.0);

        //     if (base_close_xyw(*robot, final_goal, 0.10, 0.25)) {
        //         RCLCPP_INFO(this->get_logger(), "Standoff reached -> PRE_PRESS");
        //         simple_state_ = SimpleState::PRE_PRESS;
        //     }

        //     break;
        // }
        case SimpleState::APPROACH: {
            auto final_standoff = feasible_standoff_utils::compute_circular_target(button_x, button_y, robot->x, robot->y, 1.25);
            
            // 1. Initialize start pose and distance once
            if (approach_start_dist < 0) {
                approach_start_dist = std::hypot(final_standoff.position.x - robot->x, final_standoff.position.y - robot->y);
                approach_start_pose = robot->pose; // Store the actual robot pose at start
            }

            double current_dist = std::hypot(final_standoff.position.x - robot->x, final_standoff.position.y - robot->y);
            double progress = 1.0 - (current_dist / approach_start_dist);
            double heading = std::atan2(final_standoff.position.y - robot->y, final_standoff.position.x - robot->x);
            double final_yaw = feasible_standoff_utils::get_pose_yaw(final_standoff);

            geometry_msgs::msg::Pose current_waypoint;

            // --- 3-WAYPOINT LEAP-FROG ---
            if (progress < 0.24) {
                // Target is physically 25% of the way there
                current_waypoint = interpolate_pose(approach_start_pose, final_standoff, 0.26);
                publish_mission_goal(current_waypoint, false, 0.225, heading);
            } 
            else if (progress < 0.74) {
                // Target is physically 75% of the way there
                current_waypoint = interpolate_pose(approach_start_pose, final_standoff, 0.76);
                publish_mission_goal(current_waypoint, false, 0.25, heading);
            } 
            else {
                // Target is the final standoff
                current_waypoint = final_standoff;
                publish_mission_goal(current_waypoint, false, 0.0, 0.0);
            }

            if (base_close_xyw(*robot, final_standoff, 0.10, 0.20)) {
                approach_start_dist = -1.0; 
                simple_state_ = SimpleState::PRE_PRESS;
            }
            break;
        }


        case SimpleState::PRE_PRESS: {
            if (!have_costmap_) {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, 
                                    "Waiting for costmap on topic: /local_costmap/costmap...");
                return; // Exit early since we can't calculate a standoff without a map
            }

            // Compute feasible standoff ONCE
            if (standoff_yaw < -900) {
                standoff_yaw = feasible_standoff_utils::compute_optimized_standoff(
                    button_x, button_y, robot->x, robot->y, latest_costmap_);
                if (standoff_yaw < -900) return;
            }

            target_pose = feasible_standoff_utils::compute_ee_target(button_x, button_y, button_z, standoff_yaw, 0.15);

            set_yaw(target_pose, standoff_yaw);

            publish_mission_goal(target_pose, true, 0.0, 0.0);

            if (ee_close_xyz(*ee_pos, target_pose.position.x, target_pose.position.y, target_pose.position.z, 0.02)) {
                simple_state_ = SimpleState::PRESS;
                RCLCPP_INFO(this->get_logger(), "PRE_PRESS -> PRESS");
            }
            break;
        }

        case SimpleState::PRESS: {
            target_pose = feasible_standoff_utils::compute_ee_target(button_x, button_y, button_z-0.05, standoff_yaw, 0.06);
            set_yaw(target_pose, standoff_yaw);

            publish_mission_goal(target_pose, true, 0.0, 0.0);

            if (ee_close_xyz(*ee_pos, target_pose.position.x, target_pose.position.y, target_pose.position.z, 0.025)) {
                simple_state_ = SimpleState::RETRACT;
                RCLCPP_INFO(this->get_logger(), "PRESS -> RETRACT");
            }
            break;
        }

        case SimpleState::RETRACT: {
            auto door = get_complete_door(button_x, button_y);
            if (!door) return;

            double door_yaw = feasible_standoff_utils::get_pose_yaw(door->center);
            double target_x = door->pillar.position.x - 0.6 * std::cos(door_yaw);
            double target_y = door->pillar.position.y - 0.6 * std::sin(door_yaw);
            
            target_pose.position.x = target_x;
            target_pose.position.y = target_y;

            double fixed_angle_to_center = std::atan2(door->center.position.y - target_y, 
                                                    door->center.position.x - target_x);
            
            set_yaw(target_pose, fixed_angle_to_center);
            publish_mission_goal(target_pose, false, 0.0, 0.0);

            // 3. TRANSITION
            // Note: 0.05m is a very tight tolerance for a 5-second window. 
            // If the robot "hunts" at the end, consider 0.1m.
            if (base_close_xyw(*robot, target_pose, 0.10, 0.15)) {
                simple_state_ = SimpleState::EXIT;
                RCLCPP_INFO(this->get_logger(), "RETRACT -> EXIT");
            }
            break;
        }

        // case SimpleState::EXIT: {
        //     auto door = get_complete_door(button_x, button_y);
        //     if (!door) return;

        //     double door_yaw = feasible_standoff_utils::get_pose_yaw(door->center);
        //     double dx = robot->x - door->center.position.x;
        //     double dy = robot->y - door->center.position.y;
        //     double progress = dx * std::cos(door_yaw) + dy * std::sin(door_yaw);

        //     double target_offset = 0.0;

        //     // --- 4-STAGE RELAY LOGIC ---
        //     // We update the target way BEFORE the robot reaches the current one
        //     // to prevent the quintic "slow-down" phase.
        //     if (progress < -0.3) {
        //         target_offset = 0.2;  // Aim for threshold
        //     } 
        //     else if (progress < 0.1) {
        //         target_offset = 0.6;  // Aim for middle of door
        //     } 
        //     else if (progress < 0.5) {
        //         target_offset = 1.2;  // Aim for clearing the frame
        //     } 
        //     else {
        //         target_offset = 1.8;  // The "Deep Clear" target
        //     }

        //     target_pose.position.x = door->center.position.x + target_offset * std::cos(door_yaw);
        //     target_pose.position.y = door->center.position.y + target_offset * std::sin(door_yaw);
        //     set_yaw(target_pose, door_yaw);
        //     publish_mission_goal(target_pose, false, 0.0, 0.0);

        //     // --- FINAL SUCCESS ---
        //     // We only need to reach 1.2m to be fully clear of the swing
        //     if (progress > 1.2) {
        //         simple_state_ = SimpleState::DONE;
        //         RCLCPP_INFO(this->get_logger(), "Sprint Complete. Final progress: %.2fm", progress);
        //     }
        //     break;
        // }

        case SimpleState::EXIT: {
            auto door = get_complete_door(button_x, button_y);
            if (!door) return;

            double door_yaw = feasible_standoff_utils::get_pose_yaw(door->center);
            
            // Calculate robot's progress ALONG the door's axis
            // P < 0 means robot is in front of the door, P > 0 means robot has passed center
            double dx = robot->x - door->center.position.x;
            double dy = robot->y - door->center.position.y;
            double p = dx * std::cos(door_yaw) + dy * std::sin(door_yaw);

            geometry_msgs::msg::Pose target_pose = door->center;

            // --- SPATIAL RELAY LOGIC ---
            if (p < -0.1) {
                // Still in front: Aim for the middle of the door frame
                target_pose.position.x += 0.2 * std::cos(door_yaw);
                target_pose.position.y += 0.2 * std::sin(door_yaw);
                publish_mission_goal(target_pose, false, 0.25, door_yaw);
            } 
            else if (p < 0.4) {
                // Inside the frame: Aim for the "Clearance" point (0.8m past center)
                target_pose.position.x += 0.8 * std::cos(door_yaw);
                target_pose.position.y += 0.8 * std::sin(door_yaw);
                publish_mission_goal(target_pose, false, 0.3, door_yaw);
            } 
            else {
                // Clearing: Aim for final "Deep Clear" point and SLOW DOWN
                target_pose.position.x += 1.5 * std::cos(door_yaw);
                target_pose.position.y += 1.5 * std::sin(door_yaw);
                publish_mission_goal(target_pose, false, 0.0, door_yaw);
            }

            // --- EXIT CONDITION ---
            // Successfully cleared if we are 1.2m past the door center
            if (p > 1.2) {
                RCLCPP_INFO(this->get_logger(), "Deep Clear achieved. EXIT -> DONE");
                simple_state_ = SimpleState::DONE;
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

// ----------------------
// Helper for Approach
// ----------------------

// visual for approach standoff
// void MissionSequencer::publish_feasible_cloud(const std::vector<geometry_msgs::msg::Pose>& poses)
// {
//     visualization_msgs::msg::MarkerArray arr;

//     // Clear previous
//     visualization_msgs::msg::Marker clear;
//     clear.action = visualization_msgs::msg::Marker::DELETEALL;
//     arr.markers.push_back(clear);

//     int id = 0;
//     for (const auto& p : poses) {
//         visualization_msgs::msg::Marker m;
//         // Use the frame the costmap lives in (likely "odom")
//         m.header.frame_id = latest_costmap_.header.frame_id; 
//         m.header.stamp = this->now();
//         m.ns = "feasible_points";
//         m.id = id++;
//         m.type = visualization_msgs::msg::Marker::SPHERE;
//         m.action = visualization_msgs::msg::Marker::ADD;

//         m.pose = p;
//         m.pose.position.z = 0.05; 
//         m.scale.x = 0.08; m.scale.y = 0.08; m.scale.z = 0.08;

//         m.color.r = 0.0f; m.color.g = 1.0f; m.color.b = 0.0f; m.color.a = 0.8f;
//         arr.markers.push_back(m);
//     }
//     marker_array_pub_->publish(arr);
// }



// --------------------
// Vision Marker Parser
// --------------------

std::optional<geometry_msgs::msg::Pose> MissionSequencer::get_closest_button() {
    double best_dist = 1e6;
    std::optional<geometry_msgs::msg::Pose> best_pose;

    auto robot = get_base_position();
    if (!robot) return std::nullopt;

    for (const auto& m : latest_markers_.markers) {
        if (m.action == visualization_msgs::msg::Marker::DELETEALL || m.ns.empty()) continue;
        
        if (m.ns.find("button") != std::string::npos) {
            // Use robot->x and robot->y from our state struct
            double d = std::hypot(m.pose.position.x - robot->x, m.pose.position.y - robot->y);
            if (d < best_dist) {
                best_dist = d;
                best_pose = m.pose;
            }
        }
    }
    return best_pose;
}

std::optional<DoorInfo> MissionSequencer::get_complete_door(double bx, double by) {
    std::optional<geometry_msgs::msg::Pose> best_center;
    std::string best_ns = "";
    double best_dist = 1e6;

    // 1. Find the Arrow (Center) closest to the button
    for (const auto& m : latest_markers_.markers) {
        if (m.type == visualization_msgs::msg::Marker::ARROW && m.ns.find("door") != std::string::npos) {
            double d = std::hypot(m.pose.position.x - bx, m.pose.position.y - by);
            if (d < 2.0 && d < best_dist) {
                best_dist = d;
                best_center = m.pose;
                // Extract the unique door ID part (e.g., "door_101")
                best_ns = m.ns.substr(0, m.ns.find("_arrow"));
            }
        }
    }

    if (!best_center || best_ns.empty()) return std::nullopt;

    // 2. Find the Pillar (Cylinder) that belongs to that SAME door ID
    DoorInfo door;
    door.center = *best_center;
    double best_pillar_dist = 1e6;
    bool found_pillar = false;

    for (const auto& m : latest_markers_.markers) {
        // Must match the exact door ID and be a pillar
        if (m.ns.find(best_ns) != std::string::npos && m.type == visualization_msgs::msg::Marker::CYLINDER) {
            double d = std::hypot(m.pose.position.x - bx, m.pose.position.y - by);
            if (d < best_pillar_dist) {
                best_pillar_dist = d;
                door.pillar = m.pose;
                found_pillar = true;
            }
        }
    }

    if (found_pillar) return door;
    return std::nullopt;
}

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MissionSequencer>());
    rclcpp::shutdown();
    return 0;
}
