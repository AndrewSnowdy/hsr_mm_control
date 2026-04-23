#include "hsr_mm_control/mission_sequencer.hpp"
#include "hsr_mm_control/utils_feasible_points.hpp"
#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/loggers/bt_cout_logger.h"
#include "std_msgs/msg/int32.hpp"

using namespace std::chrono_literals;


class SearchForButton : public BT::SyncActionNode {
public:
    SearchForButton(const std::string& name, const BT::NodeConfig& config, MissionSequencer* s)
        : BT::SyncActionNode(name, config), s_(s) {}

    static BT::PortsList providedPorts() {
        return { 
            BT::OutputPort<geometry_msgs::msg::Pose>("btn_pose"),
            BT::OutputPort<std::string>("btn_type"),
            BT::OutputPort<DoorInfo>("door_info")
        };
    }

    BT::NodeStatus tick() override {
        // Now returns std::optional<ButtonInfo>
        auto btn_result = s_->get_closest_button(); 
        
        if (btn_result) {
            // result is the struct; result->pose and result->type are the fields
            setOutput("btn_pose", btn_result->pose);
            setOutput("btn_type", btn_result->type);

            auto door_result = s_->get_complete_door(btn_result->pose.position.x, 
                                               btn_result->pose.position.y);
                             
            if (door_result) {
                // Update Door data on Blackboard
                setOutput("door_info", *door_result);
                
                // CRITICAL: Update the MissionSequencer's internal safety target
                // This ensures the person-avoidance zone moves with the vision
                s_->set_current_door_target(*door_result);
                
                RCLCPP_DEBUG(s_->get_logger(), "BT: Button and Door context updated.");
            } else {
                // Optional: Log if button is found but pillars aren't visible yet
                RCLCPP_WARN_THROTTLE(s_->get_logger(), *s_->get_clock(), 2000, 
                                    "BT: Found button, but door pillars not seen.");
            }

            // RCLCPP_INFO(s_->get_logger(), "BT: Found %s! Passing to Blackboard.", btn_result->type.c_str());
            return BT::NodeStatus::SUCCESS;
        }
        
        return BT::NodeStatus::FAILURE;
    }
private:
    MissionSequencer* s_;
};

// --- NODE 2: APPROACH (Replaces case APPROACH) ---
// class ApproachButton : public BT::StatefulActionNode {
// public:
//     ApproachButton(const std::string& name, const BT::NodeConfig& config, MissionSequencer* s)
//         : BT::StatefulActionNode(name, config), s_(s) {}

//     static BT::PortsList providedPorts() {
//         return { BT::InputPort<geometry_msgs::msg::Pose>("btn_pose") };
//     }

//     // Runs once when the node is first called
//     BT::NodeStatus onStart() override {
//         if (!getInput("btn_pose", target_button_)) {
//             return BT::NodeStatus::FAILURE;
//         }
//         start_dist_ = -1.0; // Reset interpolation tracking
//         return BT::NodeStatus::RUNNING;
//     }

//     // Runs repeatedly until it returns SUCCESS or FAILURE
//     BT::NodeStatus onRunning() override {

//         if (!getInput("btn_pose", target_button_)) {
//             return BT::NodeStatus::RUNNING; 
//         }

//         double gripper = s_->target_gripper;


//         auto robot = s_->get_base_position();
//         if (!robot) return BT::NodeStatus::RUNNING;


//         // --- YOUR ORIGINAL LOGIC ---
//         auto final_standoff = feasible_standoff_utils::compute_circular_target(
//             target_button_.position.x, target_button_.position.y, robot->x, robot->y, 1.25);
        
//         if (start_dist_ < 0) {
//             start_dist_ = std::hypot(final_standoff.position.x - robot->x, final_standoff.position.y - robot->y);
//             start_pose_ = robot->pose;
//         }

//         double current_dist = std::hypot(final_standoff.position.x - robot->x, final_standoff.position.y - robot->y);
//         double progress = 1.0 - (current_dist / start_dist_);
//         double heading = std::atan2(final_standoff.position.y - robot->y, final_standoff.position.x - robot->x);

//         if (progress < 0.25) {
//             auto next_waypoint = s_->interpolate_pose(start_pose_, final_standoff, 0.33);
//             s_->publish_mission_goal(next_waypoint, false, 0.1, heading, false, 0.1);
//         } else if (progress < 0.74) {
//             auto next_waypoint = s_->interpolate_pose(start_pose_, final_standoff, 0.80);
//             s_->publish_mission_goal(next_waypoint, false, 0.1, heading, false, 0.1);
//         } else {
//             s_->publish_mission_goal(final_standoff, false, 0.0, 0.0, false, 0.1);
//         }

//         // s_->publish_mission_goal(final_standoff, false, 0.0, 0.0, false, gripper);
//         // The "Spatial Switch" trigger
//         if (s_->base_close_xyw(*robot, final_standoff, 0.10, 0.20)) {
//             RCLCPP_INFO(s_->get_logger(), "BT: Approach Complete.");
//             return BT::NodeStatus::SUCCESS;
//         }

//         return BT::NodeStatus::RUNNING;
//     }

//     void onHalted() override { /* Stop robot if interrupted */ }

// private:
//     MissionSequencer* s_;
//     geometry_msgs::msg::Pose target_button_;
//     geometry_msgs::msg::Pose start_pose_;
//     double start_dist_ = -1.0;
// };


// class ApproachButton : public BT::StatefulActionNode {
// public:
//     ApproachButton(const std::string& name, const BT::NodeConfig& config, MissionSequencer* s)
//         : BT::StatefulActionNode(name, config), s_(s) {}

//     static BT::PortsList providedPorts() {
//         return { BT::InputPort<geometry_msgs::msg::Pose>("btn_pose") };
//     }

//     BT::NodeStatus onStart() override {
//         if (!getInput("btn_pose", target_button_)) return BT::NodeStatus::FAILURE;
//         return BT::NodeStatus::RUNNING;
//     }

//     BT::NodeStatus onRunning() override {
//         if (!getInput("btn_pose", target_button_)) return BT::NodeStatus::RUNNING;
//         auto robot = s_->get_base_position();
//         if (!robot) return BT::NodeStatus::RUNNING;

//         // The goal is always re-computed from current robot position
//         auto final_standoff = feasible_standoff_utils::compute_circular_target(
//             target_button_.position.x, target_button_.position.y,
//             robot->x, robot->y, 1.25);

//         int8_t cost_at_goal = 0;
//         if (feasible_standoff_utils::costAtWorld(s_->get_latest_costmap(), 
//                 final_standoff.position.x, final_standoff.position.y, cost_at_goal)) {
//             RCLCPP_INFO_THROTTLE(s_->get_logger(), *s_->get_clock(), 500,
//                 "[DBG] frame=%s standoff=(%.2f,%.2f) cost=%d",
//                 s_->get_latest_costmap().header.frame_id.c_str(),
//                 final_standoff.position.x, final_standoff.position.y, cost_at_goal);
//         } else {
//             RCLCPP_WARN_THROTTLE(s_->get_logger(), *s_->get_clock(), 500,
//                 "[DBG] standoff (%.2f,%.2f) OUTSIDE COSTMAP — frame=%s",
//                 final_standoff.position.x, final_standoff.position.y,
//                 s_->get_latest_costmap().header.frame_id.c_str());
//         }

//         // Single call — returns goal directly if clear, or a boundary exit point if blocked
//         auto next_wp = feasible_standoff_utils::compute_next_waypoint(
//             robot->x, robot->y, final_standoff, s_->get_latest_costmap());

//         s_->publish_mission_goal(next_wp, false, 0.0, 0.0, false, 0.1);

//         // Visualization
//         visualization_msgs::msg::MarkerArray debug_markers;

        

//         // Ray: robot -> next_wp (green if going direct, blue if via boundary)
//         bool is_avoiding = (std::hypot(next_wp.position.x - final_standoff.position.x,
//                                        next_wp.position.y - final_standoff.position.y) > 0.1);

//         visualization_msgs::msg::Marker ray;
//         ray.header.frame_id = "odom";
//         ray.header.stamp = s_->now();
//         ray.ns = "approach"; ray.id = 0;
//         ray.type = visualization_msgs::msg::Marker::LINE_STRIP;
//         ray.scale.x = 0.03;
//         ray.color.a = 1.0;
//         ray.color.r = is_avoiding ? 1.0 : 0.0;
//         ray.color.g = is_avoiding ? 0.0 : 1.0;
//         geometry_msgs::msg::Point p1, p2;
//         p1.x = robot->x;          p1.y = robot->y;          p1.z = 0.05;
//         p2.x = next_wp.position.x; p2.y = next_wp.position.y; p2.z = 0.05;
//         ray.points.push_back(p1);
//         ray.points.push_back(p2);
//         debug_markers.markers.push_back(ray);

//         // Sphere at next_wp if avoiding
//         if (is_avoiding) {
//             visualization_msgs::msg::Marker sphere;
//             sphere.header.frame_id = "odom";
//             sphere.header.stamp = s_->now();
//             sphere.ns = "approach"; sphere.id = 1;
//             sphere.type = visualization_msgs::msg::Marker::SPHERE;
//             sphere.pose = next_wp;
//             sphere.scale.x = sphere.scale.y = sphere.scale.z = 0.2;
//             sphere.color.a = 1.0; sphere.color.b = 1.0;
//             debug_markers.markers.push_back(sphere);
//         }
//         s_->marker_array_pub2_->publish(debug_markers);

//         // Debug: sample the costmap along the ray to the standoff
//         auto costmap = s_->get_latest_costmap();
//         RCLCPP_INFO_THROTTLE(s_->get_logger(), *s_->get_clock(), 500,
//             "Costmap: origin=(%.2f, %.2f) res=%.3f size=%ux%u",
//             costmap.info.origin.position.x,
//             costmap.info.origin.position.y,
//             costmap.info.resolution,
//             costmap.info.width,
//             costmap.info.height);

//         // Done when we reach the final standoff, not just the current waypoint
//         if (s_->base_close_xyw(*robot, final_standoff, 0.10, 0.20)) {
//             RCLCPP_INFO(s_->get_logger(), "BT: Approach complete.");
//             return BT::NodeStatus::SUCCESS;
//         }

//         return BT::NodeStatus::RUNNING;
//     }

//     void onHalted() override {}

// private:
//     MissionSequencer* s_;
//     geometry_msgs::msg::Pose target_button_;
// };



class ApproachButton : public BT::StatefulActionNode {
public:
    ApproachButton(const std::string& name, const BT::NodeConfig& config, MissionSequencer* s)
        : BT::StatefulActionNode(name, config), s_(s) {}

    static BT::PortsList providedPorts() {
        return { BT::InputPort<geometry_msgs::msg::Pose>("btn_pose") };
    }

    BT::NodeStatus onStart() override {
        if (!getInput("btn_pose", target_button_)) return BT::NodeStatus::FAILURE;
        start_dist_ = -1.0;
        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override {
        if (!getInput("btn_pose", target_button_)) return BT::NodeStatus::RUNNING;
        auto robot = s_->get_base_position();
        if (!robot) return BT::NodeStatus::RUNNING;

        // 1. Compute the ultimate goal
        auto final_standoff = feasible_standoff_utils::compute_circular_target(
            target_button_.position.x, target_button_.position.y,
            robot->x, robot->y, 1.25);

        // 2. Get obstacle-aware waypoint (may be a boundary exit point or final_standoff itself)
        auto next_wp = feasible_standoff_utils::compute_next_waypoint(
            robot->x, robot->y, final_standoff, s_->get_latest_costmap());

        // 3. Blend smoothly back to final_standoff as path clears
        double deviation = std::hypot(
            next_wp.position.x - final_standoff.position.x,
            next_wp.position.y - final_standoff.position.y);
        const double blend_threshold = 0.5;
        double alpha = std::clamp(1.0 - (deviation / blend_threshold), 0.0, 1.0);

        geometry_msgs::msg::Pose avoidance_target;
        avoidance_target.position.x = next_wp.position.x + alpha * (final_standoff.position.x - next_wp.position.x);
        avoidance_target.position.y = next_wp.position.y + alpha * (final_standoff.position.y - next_wp.position.y);
        avoidance_target.position.z = 0.0;
        feasible_standoff_utils::set_pose_yaw(avoidance_target,
            std::atan2(final_standoff.position.y - avoidance_target.position.y,
                       final_standoff.position.x - avoidance_target.position.x));

        // 4. Trapezoidal interpolation — same as your original, but toward avoidance_target
        double dist_to_target = std::hypot(
            avoidance_target.position.x - robot->x,
            avoidance_target.position.y - robot->y);

        if (start_dist_ < 0) {
            start_dist_ = dist_to_target;
            start_pose_ = robot->pose;
        }

        double progress = 1.0 - (dist_to_target / start_dist_);
        double target_yaw;
        if (deviation > 0.1) {
            // Face the direction of the bypass/avoidance target
            target_yaw = std::atan2(avoidance_target.position.y - robot->y, 
                                    avoidance_target.position.x - robot->x);
        } else {
            // We are on the final stretch, face the button (target_button_ is the actual button)
            target_yaw = std::atan2(target_button_.position.y - robot->y, 
                                    target_button_.position.x - robot->x);
        }
        feasible_standoff_utils::set_pose_yaw(avoidance_target, target_yaw);

        // 4. Execution
        // Use the 'heading' variable you already calculated for the publish_mission_goal
        double heading = target_yaw;

        if (progress < 0.22) {
            auto wp = s_->interpolate_pose(start_pose_, avoidance_target, 0.33);
            s_->publish_mission_goal(wp, false, 0.15, heading, false, 0.1);
        } else if (progress < 0.7) {
            auto wp = s_->interpolate_pose(start_pose_, avoidance_target, 0.80);
            s_->publish_mission_goal(wp, false, 0.15, heading, false, 0.1);
        } else {
            s_->publish_mission_goal(avoidance_target, false, 0.0, heading, false, 0.1);
        }

        // 5. Reset interpolation when avoidance_target shifts significantly
        // (obstacle cleared, target jumps back toward final_standoff)
        if (std::hypot(avoidance_target.position.x - last_target_.position.x,
                       avoidance_target.position.y - last_target_.position.y) > 0.3) {
            start_dist_ = -1.0;
            start_pose_ = robot->pose;
        }
        last_target_ = avoidance_target;

        // Visualization
        bool is_avoiding = deviation > 0.1;
        visualization_msgs::msg::MarkerArray debug_markers;
        visualization_msgs::msg::Marker ray;
        ray.header.frame_id = "odom"; ray.header.stamp = s_->now();
        ray.ns = "approach"; ray.id = 0;
        ray.type = visualization_msgs::msg::Marker::LINE_STRIP;
        ray.scale.x = 0.03; ray.color.a = 1.0;
        ray.color.r = is_avoiding ? 1.0 : 0.0;
        ray.color.g = is_avoiding ? 0.0 : 1.0;
        geometry_msgs::msg::Point p1, p2;
        p1.x = robot->x; p1.y = robot->y; p1.z = 0.05;
        p2.x = avoidance_target.position.x; p2.y = avoidance_target.position.y; p2.z = 0.05;
        ray.points.push_back(p1); ray.points.push_back(p2);
        debug_markers.markers.push_back(ray);
        s_->marker_array_pub2_->publish(debug_markers);

        if (s_->base_close_xyw(*robot, final_standoff, 0.10, 0.20)) {
            RCLCPP_INFO(s_->get_logger(), "BT: Approach complete.");
            return BT::NodeStatus::SUCCESS;
        }

        return BT::NodeStatus::RUNNING;
    }

    void onHalted() override {}

private:
    MissionSequencer* s_;
    geometry_msgs::msg::Pose target_button_;
    geometry_msgs::msg::Pose start_pose_;
    geometry_msgs::msg::Pose last_target_;
    double start_dist_ = -1.0;
};

class PrePress : public BT::StatefulActionNode {
public:
    PrePress(const std::string& name, const BT::NodeConfig& config, MissionSequencer* s)
        : BT::StatefulActionNode(name, config), s_(s) {}

    static BT::PortsList providedPorts() {
        return { BT::InputPort<geometry_msgs::msg::Pose>("btn_pose"),
                BT::OutputPort<double>("final_yaw") }; 
    }

    BT::NodeStatus onStart() override {
        if (!getInput("btn_pose", target_button_)) return BT::NodeStatus::FAILURE;
        standoff_yaw_ = -999.0; // Reset for a fresh calculation
        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override {
        // 1. Adaptive update of button location
        if (!getInput("btn_pose", target_button_)) return BT::NodeStatus::RUNNING;

        double gripper = s_->target_gripper;

        // 2. Safety: Wait for costmap
        if (!s_->have_costmap()) {
            RCLCPP_WARN_THROTTLE(s_->get_logger(), *s_->get_clock(), 2000, "PrePress: Waiting for costmap...");
            return BT::NodeStatus::RUNNING;
        }

        auto robot = s_->get_base_position();
        auto ee_pos = s_->get_ee_position();
        if (!robot || !ee_pos) return BT::NodeStatus::RUNNING;

        // 3. Compute optimized yaw once (or keep updating if you want it very adaptive)
        if (standoff_yaw_ < -900) {
            std::vector<geometry_msgs::msg::Pose> debug_cloud;
            std::cout << "created pose vector " << std::endl;

            standoff_yaw_ = feasible_standoff_utils::compute_optimized_standoff(
                target_button_.position.x, target_button_.position.y, 
                robot->x, robot->y, s_->get_latest_costmap(),
                &debug_cloud);

            std::cout << "Got output of optimized pose" << std::endl;
            
            if (standoff_yaw_ < -900) return BT::NodeStatus::RUNNING; 
            s_->publish_feasible_cloud(debug_cloud);
        }

        // 4. Calculate EE target pose (0.15m standoff)
        auto target_pose = feasible_standoff_utils::compute_ee_target(
            target_button_.position.x, target_button_.position.y, target_button_.position.z, 
            standoff_yaw_, 0.15);
        
        s_->set_yaw(target_pose, standoff_yaw_);

        // 5. Publish to controller with IK mode = TRUE
        s_->publish_mission_goal(target_pose, true, 0.0, 0.0, true, gripper);

        // 6. Check if EE has arrived (2cm tolerance)
        if (s_->ee_close_xyz(*ee_pos, target_pose.position.x, target_pose.position.y, target_pose.position.z, 0.04)) {
            RCLCPP_INFO(s_->get_logger(), "BT: PrePress Complete. Arm is in position.");
            setOutput("final_yaw", standoff_yaw_);
            return BT::NodeStatus::SUCCESS;
        }

        return BT::NodeStatus::RUNNING;
    }

    void onHalted() override {}

private:
    MissionSequencer* s_;
    geometry_msgs::msg::Pose target_button_;
    double standoff_yaw_ = -999.0;
};


// class PressButton : public BT::StatefulActionNode {
// public:
//     PressButton(const std::string& name, const BT::NodeConfig& config, MissionSequencer* s)
//         : BT::StatefulActionNode(name, config), s_(s) {}

//     static BT::PortsList providedPorts() {
//         return { BT::InputPort<geometry_msgs::msg::Pose>("btn_pose"),
//                  BT::InputPort<double>("locked_yaw"),
//                  BT::InputPort<double>("depth") }; // New Port
//     }

//     BT::NodeStatus onStart() override {
//         if (!getInput("btn_pose", target_button_)) return BT::NodeStatus::FAILURE;
//         if (!getInput("locked_yaw", standoff_yaw_)) return BT::NodeStatus::FAILURE;
        
//         // If depth isn't provided in XML, default to 0.06
//         if (!getInput("depth", target_depth_)) {
//             target_depth_ = 0.04;
//         }
        
//         return BT::NodeStatus::RUNNING;
//     }

//     BT::NodeStatus onRunning() override {
//         auto ee_pos = s_->get_ee_position();
//         if (!ee_pos) return BT::NodeStatus::RUNNING;

//         bool flat = s_->force_wrist_flat;
//         double gripper = s_->target_gripper;
//         double offset = s_->depth_offset;
//         double adjusted_depth = target_depth_ + offset;


//         // Use the target_depth_ we got from the Blackboard/XML
//         auto press_pose = feasible_standoff_utils::compute_ee_target(
//             target_button_.position.x, target_button_.position.y, (target_button_.position.z + 0.04), 
//             standoff_yaw_, adjusted_depth);
        
//         s_->set_yaw(press_pose, standoff_yaw_);
//         s_->publish_mission_goal(press_pose, true, 0.0, 0.0, true, gripper);

//         // Arrival check
//         if (s_->ee_close_xyz(*ee_pos, press_pose.position.x, press_pose.position.y, press_pose.position.z, 0.02)) {
//             RCLCPP_INFO(s_->get_logger(), "BT: Press/Wave at depth %.2fm Complete.", target_depth_);
//             return BT::NodeStatus::SUCCESS;
//         }

//         return BT::NodeStatus::RUNNING;
//     }

//     void onHalted() override {}

// private:
//     MissionSequencer* s_;
//     geometry_msgs::msg::Pose target_button_;
//     double standoff_yaw_;
//     double target_depth_; // Store the depth here
// };

class PressButton : public BT::StatefulActionNode {
public:
    PressButton(const std::string& name, const BT::NodeConfig& config, MissionSequencer* s)
        : BT::StatefulActionNode(name, config), s_(s) {}

    static BT::PortsList providedPorts() {
        return { BT::InputPort<geometry_msgs::msg::Pose>("btn_pose"),
                 BT::InputPort<double>("locked_yaw"),
                 BT::InputPort<double>("depth") };
    }

    BT::NodeStatus onStart() override {
        if (!getInput("btn_pose", target_button_)) return BT::NodeStatus::FAILURE;
        if (!getInput("locked_yaw", standoff_yaw_)) return BT::NodeStatus::FAILURE;
        
        if (!getInput("depth", target_depth_)) {
            target_depth_ = 0.04;
        }

        // --- NEW: Reset dwell tracking ---
        arrival_time_ = std::nullopt; 
        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override {
        auto ee_pos = s_->get_ee_position();
        if (!ee_pos) return BT::NodeStatus::RUNNING;

        double gripper = s_->target_gripper;
        double offset = s_->depth_offset;
        double adjusted_depth = target_depth_ + offset;

        auto press_pose = feasible_standoff_utils::compute_ee_target(
            target_button_.position.x, target_button_.position.y, target_button_.position.z, 
            standoff_yaw_, 0.09);
        
        s_->set_yaw(press_pose, standoff_yaw_);
        s_->publish_mission_goal(press_pose, true, 0.0, 0.0, true, gripper);

        // 1. Check if we have arrived at the proximity zone
        bool is_close = s_->ee_close_xyz(*ee_pos, press_pose.position.x, press_pose.position.y, press_pose.position.z, 0.02);

        if (is_close) {
            // 2. If this is the FIRST time we are close, start the timer
            if (!arrival_time_.has_value()) {
                arrival_time_ = s_->now();
                RCLCPP_INFO(s_->get_logger(), "BT: Arrived at button. Dwelling for sensor activation...");
            }

            // 3. Check if 2 seconds have passed since arrival
            auto elapsed = (s_->now() - *arrival_time_).seconds();
            if (elapsed >= 0.0) {
                RCLCPP_INFO(s_->get_logger(), "BT: Dwell complete. Success.");
                return BT::NodeStatus::SUCCESS;
            }
        } else {
            // Optional: If the robot drifts away, reset the timer
            // arrival_time_ = std::nullopt; 
        }

        return BT::NodeStatus::RUNNING;
    }

    void onHalted() override {}

private:
    MissionSequencer* s_;
    geometry_msgs::msg::Pose target_button_;
    double standoff_yaw_;
    double target_depth_;
    
    // --- NEW: Timer Variable ---
    std::optional<rclcpp::Time> arrival_time_;
};


class Retract : public BT::StatefulActionNode {
public:
    Retract(const std::string& name, const BT::NodeConfig& config, MissionSequencer* s)
        : BT::StatefulActionNode(name, config), s_(s) {}

    static BT::PortsList providedPorts() {
        return { BT::InputPort<geometry_msgs::msg::Pose>("btn_pose"),
                BT::InputPort<DoorInfo>("door_info") };
    }

    BT::NodeStatus onStart() override {
        // 1. Get inputs from Blackboard
        if (!getInput("btn_pose", target_button_)) return BT::NodeStatus::FAILURE;
        if (!getInput("door_info", door_)) return BT::NodeStatus::FAILURE;

        // 2. Pre-calculate the door yaw and target standoff ONCE
        tf2::Quaternion q(door_.center.orientation.x, door_.center.orientation.y, 
                        door_.center.orientation.z, door_.center.orientation.w);
        double r, p;
        tf2::Matrix3x3(q).getRPY(r, p, door_yaw_);

        double dynamic_standoff = s_->get_door_safe_standoff(target_button_, door_.pillar1, door_.pillar2);

        target_pose_.position.x = door_.pillar1.position.x - dynamic_standoff * std::cos(door_yaw_);
        target_pose_.position.y = door_.pillar1.position.y - dynamic_standoff * std::sin(door_yaw_);
        
        double angle_to_center = std::atan2(door_.center.position.y - target_pose_.position.y, 
                                            door_.center.position.x - target_pose_.position.x);
        s_->set_yaw(target_pose_, angle_to_center);

        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override {
        auto robot = s_->get_base_position();
        if (!robot) return BT::NodeStatus::RUNNING;

        // 3. Just publish the pre-calculated goal
        s_->publish_mission_goal(target_pose_, false, 0.0, 0.0, s_->force_wrist_flat, s_->target_gripper);

        if (s_->base_close_xyw(*robot, target_pose_, 0.125, 0.2)) {
            RCLCPP_INFO(s_->get_logger(), "BT: Retract Complete.");
            return BT::NodeStatus::SUCCESS;
        }
        return BT::NodeStatus::RUNNING;
    }

    void onHalted() override {}

private:
    MissionSequencer* s_;
    geometry_msgs::msg::Pose target_button_;
    geometry_msgs::msg::Pose target_pose_;
    DoorInfo door_;
    double door_yaw_;
};


// class ExitDoor : public BT::StatefulActionNode {
// public:
//     ExitDoor(const std::string& name, const BT::NodeConfig& config, MissionSequencer* s)
//         : BT::StatefulActionNode(name, config), s_(s) {}

//     static BT::PortsList providedPorts() {
//         return { BT::InputPort<DoorInfo>("door_info") }; // Only need door info now
//     }

//     BT::NodeStatus onStart() override {
//         if (!getInput("door_info", door_)) return BT::NodeStatus::FAILURE;
        
//         // Lock door yaw once
//         door_yaw_ = feasible_standoff_utils::get_pose_yaw(door_.center);
//         return BT::NodeStatus::RUNNING;
//     }

//     BT::NodeStatus onRunning() override {
//         auto robot = s_->get_base_position();
//         if (!robot) return BT::NodeStatus::RUNNING;

//         // 1. Calculate Progress (p) using the stable door center
//         double dx = robot->x - door_.center.position.x;
//         double dy = robot->y - door_.center.position.y;
//         double p = dx * std::cos(door_yaw_) + dy * std::sin(door_yaw_);

//         // 2. Define Ghost Poses relative to stable door center
//         geometry_msgs::msg::Pose stage1 = door_.center; 
//         stage1.position.x += 0.2 * std::cos(door_yaw_);
//         stage1.position.y += 0.2 * std::sin(door_yaw_);

//         geometry_msgs::msg::Pose stage2 = door_.center; 
//         stage2.position.x += 0.8 * std::cos(door_yaw_);
//         stage2.position.y += 0.8 * std::sin(door_yaw_);

//         geometry_msgs::msg::Pose final_goal = door_.center; 
//         final_goal.position.x += 1.0 * std::cos(door_yaw_);
//         final_goal.position.y += 1.0 * std::sin(door_yaw_);

//         // if (s_->base_close_xyw(*robot, final_goal, 0.15, 0.20)) {
//         //     RCLCPP_INFO(s_->get_logger(), "BT: Exit Complete.");
//         //     return BT::NodeStatus::SUCCESS;
//         // }

//         double dist_to_final = std::hypot(
//             final_goal.position.x - robot->x,
//             final_goal.position.y - robot->y);

//         // ---- ADD THIS ----
//         RCLCPP_INFO_THROTTLE(s_->get_logger(), *s_->get_clock(), 200,
//             "[ExitDoor] p=%.3f | robot=(%.2f,%.2f) | center=(%.2f,%.2f) | yaw=%.2f | dist_to_final=%.3f",
//             p, robot->x, robot->y, 
//             door_.center.position.x, door_.center.position.y,
//             door_yaw_, dist_to_final);
//         // ------------------

//         if (s_->base_close_xyw(*robot, final_goal, 0.15, 0.20)) {
//             RCLCPP_INFO(s_->get_logger(), "BT: Exit Complete.");
//             return BT::NodeStatus::SUCCESS;
//         }

//         if (p < -0.1) {
//             RCLCPP_INFO_THROTTLE(s_->get_logger(), *s_->get_clock(), 500,
//                 "[ExitDoor] STAGE 1 → target=(%.2f,%.2f) speed=0.25",
//                 stage1.position.x, stage1.position.y);
//             s_->publish_mission_goal(stage1, false, 0.35, door_yaw_);
//         } else if (p < 0.6) {
//             RCLCPP_INFO_THROTTLE(s_->get_logger(), *s_->get_clock(), 500,
//                 "[ExitDoor] STAGE 2 → target=(%.2f,%.2f) speed=0.25",
//                 stage2.position.x, stage2.position.y);
//             s_->publish_mission_goal(stage2, false, 0.35, door_yaw_);
//         } else {
//             RCLCPP_INFO_THROTTLE(s_->get_logger(), *s_->get_clock(), 500,
//                 "[ExitDoor] STAGE 3 → target=(%.2f,%.2f) speed=0.20",
//                 final_goal.position.x, final_goal.position.y);
//             s_->publish_mission_goal(final_goal, false, 0.20, door_yaw_);
//         }


//         return BT::NodeStatus::RUNNING;
//     }

//     void onHalted() override {}

// private:
//     MissionSequencer* s_;
//     DoorInfo door_;
//     double door_yaw_;
// };


class ExitDoor : public BT::StatefulActionNode {
public:
    ExitDoor(const std::string& name, const BT::NodeConfig& config, MissionSequencer* s)
        : BT::StatefulActionNode(name, config), s_(s) {}

    static BT::PortsList providedPorts() {
        return { BT::InputPort<DoorInfo>("door_info") }; // Only need door info now
    }


    BT::NodeStatus onStart() override {
        if (!getInput("door_info", door_)) return BT::NodeStatus::FAILURE;
        door_yaw_ = feasible_standoff_utils::get_pose_yaw(door_.center);
        
        final_goal_ = door_.center;
        final_goal_.position.x += 1.4 * std::cos(door_yaw_);
        final_goal_.position.y += 1.4 * std::sin(door_yaw_);
        
        start_dist_ = -1.0;
        passed_center_ = false;  // reset latch
        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override {
        auto robot = s_->get_base_position();
        if (!robot) return BT::NodeStatus::RUNNING;

        double dist_to_final = std::hypot(
            final_goal_.position.x - robot->x,
            final_goal_.position.y - robot->y);
        double dist_to_center = std::hypot(
            door_.center.position.x - robot->x,
            door_.center.position.y - robot->y);

        if (start_dist_ < 0) {
            start_dist_ = dist_to_final;
            start_pose_ = robot->pose;
        }

        // Latch: once within 0.3m of center, we've passed through
        if (dist_to_center < 0.3) passed_center_ = true;

        double progress = 1.0 - (dist_to_final / start_dist_);

        if (!passed_center_) {
            auto wp = s_->interpolate_pose(start_pose_, door_.center, 0.80);
            s_->publish_mission_goal(wp, false, 0.35, door_yaw_);
        } else if (progress < 0.50) {
            auto wp = s_->interpolate_pose(door_.center, final_goal_, 0.65);
            s_->publish_mission_goal(wp, false, 0.35, door_yaw_);
        } else {
            s_->publish_mission_goal(final_goal_, false, 0.20, door_yaw_);
        }

        if (s_->base_close_xyw(*robot, final_goal_, 0.15, 0.25)) {
            RCLCPP_INFO(s_->get_logger(), "BT: Exit Complete.");
            return BT::NodeStatus::SUCCESS;
        }

        return BT::NodeStatus::RUNNING;
    }
    void onHalted() override {}

private:
    MissionSequencer* s_;
    DoorInfo door_;
    double door_yaw_;
    geometry_msgs::msg::Pose final_goal_;
    geometry_msgs::msg::Pose start_pose_;
    bool passed_center_ = false;
    double start_dist_ = -1.0;
};





class GraspAndRetract : public BT::StatefulActionNode {
public:
    GraspAndRetract(const std::string& name, const BT::NodeConfig& config, MissionSequencer* s)
        : BT::StatefulActionNode(name, config), s_(s), action_started_(false) {}

    static BT::PortsList providedPorts() {
        return { BT::InputPort<geometry_msgs::msg::Pose>("btn_pose"),
                 BT::InputPort<double>("locked_yaw"),
                 BT::InputPort<double>("depth") };
    }

    BT::NodeStatus onStart() override {
        if (!getInput("btn_pose", target_button_)) return BT::NodeStatus::FAILURE;
        if (!getInput("locked_yaw", standoff_yaw_)) return BT::NodeStatus::FAILURE;
        getInput("depth", target_depth_);
        
        // Reset local state for this specific BT activation
        action_started_ = false; 
        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override {
        auto ee_pos = s_->get_ee_position();
        if (!ee_pos) return BT::NodeStatus::RUNNING;

        auto target_pose = feasible_standoff_utils::compute_ee_target(
            target_button_.position.x, target_button_.position.y, target_button_.position.z,
            standoff_yaw_, target_depth_);

        // Determine gripper widths based on mode
        // Grasp: Approach Open (1.2) -> Action Close (0.1)
        // Release: Approach Closed (0.1) -> Action Open (1.2)
        double approach_gripper = (s_->current_mode == "grasp") ? 1.2 : 0.3;
        double action_gripper   = (s_->current_mode == "grasp") ? 0.3 : 1.2;

        // --- PHASE 1: APPROACHING ---
        if (!action_started_) {
            s_->set_yaw(target_pose, standoff_yaw_);
            s_->publish_mission_goal(target_pose, true, 0.0, 0.0, true, approach_gripper);

            if (s_->ee_close_xyz(*ee_pos, target_pose.position.x, target_pose.position.y, target_pose.position.z, 0.02)) {
                RCLCPP_INFO(s_->get_logger(), "BT: Reached object. Starting %s...", s_->current_mode.c_str());
                action_started_ = true;
                
                // Trigger the weld/unweld plugin
                std_msgs::msg::Empty msg;
                if (s_->current_mode == "grasp") {
                    s_->grasp_attach_pub_->publish(msg);
                } else {
                    s_->grasp_detach_pub_->publish(msg);
                }
                
                start_action_time_ = s_->now();
            }
            return BT::NodeStatus::RUNNING;
        }

        // --- PHASE 2: EXECUTING ACTION ---
        if (action_started_) {
            s_->publish_mission_goal(target_pose, true, 0.0, 0.0, true, action_gripper);

            auto elapsed = (s_->now() - start_action_time_).seconds();
            if (elapsed > 1.5) { 
                RCLCPP_INFO(s_->get_logger(), "BT: %s firm. Task Complete.", s_->current_mode.c_str());
                
                // Toggle the mode so the NEXT time this node is called, it does the opposite
                if (s_->current_mode == "grasp") {
                    s_->current_mode = "release";
                    s_->depth_offset = 0.01;
                } else {
                    s_->current_mode = "grasp";
                    s_->depth_offset = 0.0;
                }

                return BT::NodeStatus::SUCCESS;
            }
        }

        return BT::NodeStatus::RUNNING;
    }

    void onHalted() override {}

private:
    MissionSequencer* s_;
    geometry_msgs::msg::Pose target_button_;
    double standoff_yaw_, target_depth_;
    bool action_started_; 
    rclcpp::Time start_action_time_;
};


class IsPathClear : public BT::ConditionNode {
public:
    IsPathClear(const std::string& name, const BT::NodeConfig& config, MissionSequencer* s)
        : BT::ConditionNode(name, config), s_(s) {}

    // Conditions usually don't need ports
    static BT::PortsList providedPorts() { return {}; }

    BT::NodeStatus tick() override {
        // If is_door_path_blocked returns TRUE, the path is NOT clear (FAILURE)
        if (s_->is_door_path_blocked()) {
            RCLCPP_WARN_THROTTLE(s_->get_logger(), *s_->get_clock(), 1000, 
                                 "BT: Path Blocked! Standing by...");
            return BT::NodeStatus::FAILURE;
        }

        // Path is open!
        return BT::NodeStatus::SUCCESS;
    }

private:
    MissionSequencer* s_;
};

class IsDoorOpen : public BT::ConditionNode {
public:
    IsDoorOpen(const std::string& name, const BT::NodeConfig& config, MissionSequencer* s)
        : BT::ConditionNode(name, config), s_(s) {}

    static BT::PortsList providedPorts() {
        return { BT::InputPort<DoorInfo>("door_info") };
    }

    BT::NodeStatus tick() override {
        DoorInfo door;
        if (!getInput("door_info", door)) return BT::NodeStatus::FAILURE;
        if (!s_->have_costmap()) return BT::NodeStatus::FAILURE;
        auto robot = s_->get_base_position();

        bool open = feasible_standoff_utils::is_door_open(
            door.pillar1.position.x, door.pillar1.position.y,
            door.pillar2.position.x, door.pillar2.position.y,
            s_->get_latest_costmap());

        double dist = std::hypot(robot->x - door.center.position.x, 
                             robot->y - door.center.position.y);

        // If we are more than 4 meters away, we can't trust the costmap
        if (dist > 1.0) {
            RCLCPP_INFO_THROTTLE(s_->get_logger(), *s_->get_clock(), 2000, 
                "Too far to verify door (%.2fm). Assuming CLOSED.", dist);
            return BT::NodeStatus::FAILURE; 
        }


        if (open) {
            RCLCPP_INFO(s_->get_logger(), 
                "\033[1;32m[PERCEPTION]\033[0m Door detected as OPEN between pillars: P1(%.2f, %.2f) P2(%.2f, %.2f)", 
                door.pillar1.position.x, door.pillar1.position.y,
                door.pillar2.position.x, door.pillar2.position.y);
                            RCLCPP_INFO_THROTTLE(s_->get_logger(), *s_->get_clock(), 2000, 
                "Too far to verify door (%.2fm). Assuming CLOSED.", dist);
        } else {
            // Throttled so it doesn't spam the terminal while waiting
            RCLCPP_INFO_THROTTLE(s_->get_logger(), *s_->get_clock(), 2000, 
                "\033[1;31m[PERCEPTION]\033[0m Door CLOSED at (%.2f, %.2f). Running press sequence...", 
                door.center.position.x, door.center.position.y);
        }
        

        return open ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    }
private:
    MissionSequencer* s_;
};



geometry_msgs::msg::Pose MissionSequencer::interpolate_pose(const geometry_msgs::msg::Pose& start, 
                                                           const geometry_msgs::msg::Pose& end, 
                                                           double ratio) {
    geometry_msgs::msg::Pose p = end;
    p.position.x = start.position.x + ratio * (end.position.x - start.position.x);
    p.position.y = start.position.y + ratio * (end.position.y - start.position.y);
    return p;
}


MissionSequencer::MissionSequencer()
: Node("mission_sequencer")
{
    // visualize
    safety_zone_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("door_safety_zone", 10);
    marker_array_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("feasible_samples", 10);
    mission_pub_ = this->create_publisher<hsr_mm_control::msg::MissionGoal>("/mission_command", 10);

    door_lock_pub_ = this->create_publisher<std_msgs::msg::Bool>("/door_lock_trigger", 10);

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // timer_ = this->create_wall_timer(100ms, std::bind(&MissionSequencer::simple_timer, this));
    // stuff for can grab exclusively
    grasp_attach_pub_ = this->create_publisher<std_msgs::msg::Empty>("/grasp_attach", 10);
    grasp_detach_pub_ = this->create_publisher<std_msgs::msg::Empty>("/grasp_detach", 10);

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
        latest_markers_ptr_ = msg; 
    });

    marker_array_pub2_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        "mission_debug_markers", 10);

    RCLCPP_INFO(this->get_logger(), "MissionSequencer started.");
}
// --------------
// Publish Parser
// --------------
void MissionSequencer::publish_mission_goal(
    const geometry_msgs::msg::Pose& pose,
    bool ik_mode,
    double cruise_speed,
    double door_yaw,
    bool force_wrist_flat,
    double gripper_pos) 
{
    hsr_mm_control::msg::MissionGoal goal_msg;
    goal_msg.target_pose = pose;
    goal_msg.use_ik_mode = ik_mode;
    goal_msg.force_wrist_flat = force_wrist_flat;
    goal_msg.gripper_pos = gripper_pos; 



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

// ----------------------
// Helper for Approach
// ----------------------

// visual for approach standoff
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
        m.header.frame_id = odom_frame_; 
        m.header.stamp = this->now();
        m.ns = "feasible_points";
        m.id = id++;
        m.type = visualization_msgs::msg::Marker::SPHERE;
        m.action = visualization_msgs::msg::Marker::ADD;

        m.pose = p;
        m.pose.position.z = 0.05; 
        m.scale.x =  m.scale.y =  m.scale.z = 0.06;

        m.color.r = 0.0f; m.color.g = 1.0f; m.color.b = 0.0f; m.color.a = 0.8f;
        arr.markers.push_back(m);
    }
    marker_array_pub_->publish(arr);
}



// --------------------
// Vision Marker Parser
// --------------------

std::optional<ButtonInfo> MissionSequencer::get_closest_button() {
    double best_dist = 1e6;
    std::optional<ButtonInfo> best_found;

    auto robot = get_base_position();
    if (!robot) return std::nullopt;

    auto local_markers = latest_markers_ptr_;
    if (!local_markers) return std::nullopt; 

    for (const auto& m : local_markers->markers) {
        // Skip metadata/empty markers
        if (m.action == visualization_msgs::msg::Marker::DELETEALL || m.ns.empty()) continue;
        
        // Search for the prefix "button"
        if (m.ns.find("button") != std::string::npos || m.ns.find("bottle") != std::string::npos) {
            double d = std::hypot(m.pose.position.x - robot->x, m.pose.position.y - robot->y);
            
            if (d < best_dist) {
                best_dist = d;
                ButtonInfo info;
                info.pose = m.pose;

                // LOGIC: How to decide if it's push or prox?
                // Option A: Based on namespace content
                if (m.ns.find("push") != std::string::npos) {
                    info.type = "push_button";
                } else if (m.ns.find("prox") != std::string::npos) {
                    info.type = "prox_button";
                } else if (m.ns.find("bottle") != std::string::npos) {
                    info.type = "bottle";
                } else {
                    // Option B: Default for the green button in your script
                    info.type = "push_button"; 
                }
                
                best_found = info;
            }
        }
    }
    return best_found;
}


std::optional<DoorInfo> MissionSequencer::get_complete_door(double bx, double by) {
    std::optional<geometry_msgs::msg::Pose> best_center;
    std::string best_ns = "";
    double best_dist = 1e6;

    auto local_markers = latest_markers_ptr_;

    if (!local_markers) return std::nullopt; 

    // 1. Find the Arrow (Center) closest to the button
    for (const auto& m : local_markers->markers) {
        if (m.type == visualization_msgs::msg::Marker::ARROW && m.ns.find("door") != std::string::npos) {
            double d = std::hypot(m.pose.position.x - bx, m.pose.position.y - by);
            if (d < 4.0 && d < best_dist) {
                best_dist = d;
                best_center = m.pose;
                best_ns = m.ns.substr(0, m.ns.find("_arrow"));
            }
        }
    }

    if (!best_center || best_ns.empty()) return std::nullopt;

    // 2. Collect all cylinders in this door's namespace
    struct PillarCandidate {
        geometry_msgs::msg::Pose pose;
        double dist;
    };
    std::vector<PillarCandidate> candidates;

    for (const auto& m : local_markers->markers) {
        if (m.ns.find(best_ns) != std::string::npos && m.type == visualization_msgs::msg::Marker::CYLINDER) {
            double d = std::hypot(m.pose.position.x - bx, m.pose.position.y - by);
            candidates.push_back({m.pose, d});
        }
    }

    // 3. We need at least 2 pillars to define the line for the standoff math
    if (candidates.size() < 2) {
        RCLCPP_WARN(this->get_logger(), "Door %s identified but found only %zu pillars.", 
                    best_ns.c_str(), candidates.size());
        return std::nullopt;
    }

    // Sort by distance to button
    std::sort(candidates.begin(), candidates.end(), [](const PillarCandidate& a, const PillarCandidate& b) {
        return a.dist < b.dist;
    });

    // 4. Package
    DoorInfo door;
    door.center = *best_center;
    door.pillar1 = candidates[0].pose; // Closest
    door.pillar2 = candidates[1].pose; // Second closest

    return door;
}

bool MissionSequencer::is_door_path_blocked() {
    auto local_msg = latest_markers_ptr_;
    if (!local_msg || !current_door_target_.has_value()) return false;

    // 1. Get Door Pose and Yaw
    double tx = current_door_target_->center.position.x;
    double ty = current_door_target_->center.position.y;
    double door_yaw = feasible_standoff_utils::get_pose_yaw(current_door_target_->center);

    bool blocked = false;

    for (const auto & marker : local_msg->markers) {
        if (marker.ns == "people" || marker.ns.rfind("person_", 0) == 0) {
            // 2. Translate person relative to door center
            double dx_world = marker.pose.position.x - tx;
            double dy_world = marker.pose.position.y - ty;

            // 3. Rotate into Door Frame (2D Rotation Matrix)
            // x_local =  dx*cos(theta) + dy*sin(theta)
            // y_local = -dx*sin(theta) + dy*cos(theta)
            double x_local =  dx_world * std::cos(door_yaw) + dy_world * std::sin(door_yaw);
            double y_local = -dx_world * std::sin(door_yaw) + dy_world * std::cos(door_yaw);

            // 4. Check bounds in the LOCAL frame
            // Now "x" is always forward/backward through the door, "y" is side-to-side
            bool in_x_range = (x_local > -0.3 && x_local < 1.0); 
            bool in_y_range = (std::abs(y_local) < 0.7);

            if (in_x_range && in_y_range) {
                blocked = true;
                break;
            }
        }
    }

    // --- Visualization ---
    visualization_msgs::msg::Marker zone_marker;
    zone_marker.header.frame_id = odom_frame_;
    zone_marker.header.stamp = this->now();
    zone_marker.type = visualization_msgs::msg::Marker::CUBE;

    // Determine total dimensions based on the logic above
    double front_x = 0.8;
    double back_x = 0.3;
    double width_y = 0.7;

    zone_marker.scale.x = front_x + back_x; // Total length
    zone_marker.scale.y = width_y * 2.0;    // Total width
    zone_marker.scale.z = 0.02;

    // Offset the center so it aligns with the asymmetric bounds
    // We shift it forward by half the difference between front and back
    double x_offset = (front_x - back_x) / 2.0;

    // Apply rotation to the offset before adding to door center
    zone_marker.pose = current_door_target_->center;
    zone_marker.pose.position.x += x_offset * std::cos(door_yaw);
    zone_marker.pose.position.y += x_offset * std::sin(door_yaw);
    zone_marker.pose.position.z = 0.01;

    zone_marker.color.a = 0.3;
    zone_marker.color.r = blocked ? 1.0 : 0.0;
    zone_marker.color.g = blocked ? 0.0 : 1.0;

    safety_zone_pub_->publish(zone_marker);
    return blocked;
}

double MissionSequencer::get_door_safe_standoff(const geometry_msgs::msg::Pose& button, 
                                                const geometry_msgs::msg::Pose& p1, 
                                                const geometry_msgs::msg::Pose& p2) {
    // 1. Calculate Perpendicular Distance from Button to the Line formed by Pillars
    double x0 = button.position.x;
    double y0 = button.position.y;
    double x1 = p1.position.x;
    double y1 = p1.position.y;
    double x2 = p2.position.x;
    double y2 = p2.position.y;

    double num = std::abs((y2 - y1) * x0 - (x2 - x1) * y0 + x2 * y1 - y2 * x1);
    double den = std::sqrt(std::pow(y2 - y1, 2) + std::pow(x2 - x1, 2));
    
    double dist_to_door_line = (den == 0) ? 0.0 : num / den;

    // 2. Your Preferred Readability Logic:
    // This effectively "bounces" the standoff between 0.6 and 1.2
    double standoff = std::max(0.85, std::min(1.75, dist_to_door_line));

    return standoff;
}

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto sequencer_node = std::make_shared<MissionSequencer>();

    auto mission_count_pub = sequencer_node->create_publisher<std_msgs::msg::Int32>("/completed_mission_count", 10);
    int current_mission_index = 0;

    BT::BehaviorTreeFactory factory;

    // Registering the nodes and "injecting" the sequencer pointer
    factory.registerBuilder<SearchForButton>("SearchForButton", 
        [sequencer_node](const std::string& name, const BT::NodeConfig& config) {
            return std::make_unique<SearchForButton>(name, config, sequencer_node.get());
        });

    factory.registerBuilder<ApproachButton>("ApproachButton", 
        [sequencer_node](const std::string& name, const BT::NodeConfig& config) {
            return std::make_unique<ApproachButton>(name, config, sequencer_node.get());
        });
    
    factory.registerBuilder<PrePress>("PrePress", 
        [sequencer_node](const std::string& name, const BT::NodeConfig& config) {
            return std::make_unique<PrePress>(name, config, sequencer_node.get());
        });

    factory.registerBuilder<PressButton>("PressButton", 
        [sequencer_node](const std::string& name, const BT::NodeConfig& config) {
            return std::make_unique<PressButton>(name, config, sequencer_node.get());
        });

    factory.registerBuilder<Retract>("Retract", 
        [sequencer_node](const std::string& name, const BT::NodeConfig& config) {
            return std::make_unique<Retract>(name, config, sequencer_node.get());
        });

    factory.registerBuilder<ExitDoor>("ExitDoor", 
        [sequencer_node](const std::string& name, const BT::NodeConfig& config) {
            return std::make_unique<ExitDoor>(name, config, sequencer_node.get());
        });

    factory.registerBuilder<GraspAndRetract>("GraspAndRetract", 
        [sequencer_node](const std::string& name, const BT::NodeConfig& config) {
            return std::make_unique<GraspAndRetract>(name, config, sequencer_node.get());
        });

    factory.registerBuilder<IsPathClear>("IsPathClear", 
        [sequencer_node](const std::string& name, const BT::NodeConfig& config) {
            return std::make_unique<IsPathClear>(name, config, sequencer_node.get());
        });

    factory.registerBuilder<IsDoorOpen>("IsDoorOpen", 
        [sequencer_node](const std::string& name, const BT::NodeConfig& config) {
            return std::make_unique<IsDoorOpen>(name, config, sequencer_node.get());
        });

    // std::string xml_text = R"(
    // <root BTCPP_format="4">
    //     <BehaviorTree ID="MainTree">
    //         <Sequence name="root_sequence">
    //             <RetryUntilSuccessful num_attempts="-1">
    //                 <Sequence>
    //                     <Sleep msec="100"/>
    //                     <SearchForButton btn_pose="{target_loc}" btn_type="{target_type}" door_info="{door_data}"/>
    //                 </Sequence>
    //             </RetryUntilSuccessful>

    //             <ReactiveSequence name="adaptive_approach">
    //                 <SearchForButton btn_pose="{target_loc}" btn_type="{target_type}" door_info="{door_data}"/>
    //                 <ApproachButton btn_pose="{target_loc}"/>
    //             </ReactiveSequence>
    //             <PrePress btn_pose="{target_loc}" final_yaw="{locked_yaw}" />

    //             <Fallback name="action_selector">
                    
    //                 <Sequence name="grasp_branch">
    //                     <Precondition if="target_type == 'coke_can'" else="FAILURE">
    //                         <Sequence>
    //                             <GraspAndRetract btn_pose="{target_loc}" locked_yaw="{locked_yaw}" depth="0.04"/>
    //                             <Script code=" is_carrying := true; target_gripper := -0.1 "/>
    //                         </Sequence>
    //                     </Precondition>
    //                 </Sequence>

    //                 <Sequence name="door_full_sequence">
    //                     <Fallback name="press_selector">
    //                         <Sequence name="push_branch">
    //                             <Script code=" is_push := (target_type == 'push_button') "/>
    //                             <Precondition if="is_push" else="FAILURE">
    //                                 <PressButton btn_pose="{target_loc}" locked_yaw="{locked_yaw}" depth="0.085"/>
    //                             </Precondition>
    //                         </Sequence>
    //                         <PressButton btn_pose="{target_loc}" locked_yaw="{locked_yaw}" depth="0.12"/>
    //                     </Fallback>
                        
    //                     <Sequence name="universal_exit_logic">
    //                         <Retract btn_pose="{target_loc}" door_info="{door_data}"/>
    //                         <RetryUntilSuccessful num_attempts="-1">
    //                             <Fallback name="exit_or_wait">
    //                                 <ReactiveSequence name="guarded_exit">
    //                                     <IsPathClear />
    //                                     <ExitDoor door_info="{door_data}"/>
    //                                 </ReactiveSequence>
    //                                 <ForceFailure>
    //                                     <Sequence name="safety_standby">
    //                                         <Retract btn_pose="{target_loc}" door_info="{door_data}"/>
    //                                         <Sleep msec="500"/>
    //                                     </Sequence>
    //                                 </ForceFailure>
    //                             </Fallback>
    //                         </RetryUntilSuccessful>
    //                     </Sequence>
    //                 </Sequence> </Fallback>
    //         </Sequence>
    //     </BehaviorTree>
    // </root>
    // )";
    std::string xml_text = R"(
    <root BTCPP_format="4">
        <BehaviorTree ID="MainTree">
            <Sequence name="root_sequence">
                <RetryUntilSuccessful num_attempts="-1">
                    <Sequence>
                        <Sleep msec="100"/>
                        <SearchForButton btn_pose="{target_loc}" btn_type="{target_type}" door_info="{door_data}"/>
                    </Sequence>
                </RetryUntilSuccessful>

                <ReactiveSequence name="adaptive_approach">
                    <SearchForButton btn_pose="{target_loc}" btn_type="{target_type}" door_info="{door_data}"/>
                    <ApproachButton btn_pose="{target_loc}"/>
                </ReactiveSequence>

                <Fallback name="action_selector">
                    
                    <Sequence name="grasp_branch">
                        <Precondition if="target_type == 'bottle'" else="FAILURE">
                            <Sequence>
                                <PrePress btn_pose="{target_loc}" final_yaw="{locked_yaw}" />
                                <GraspAndRetract btn_pose="{target_loc}" locked_yaw="{locked_yaw}" depth="0.04"/>
                                <Script code=" is_carrying := true; target_gripper := -0.1 "/>
                            </Sequence>
                        </Precondition>
                    </Sequence>

                    <Sequence name="door_full_sequence">
                        <Fallback name="door_decision_gate">
                            <IsDoorOpen door_info="{door_data}"/>

                            <Sequence name="press_button_flow">
                                <PrePress btn_pose="{target_loc}" final_yaw="{locked_yaw}" />
                                <Fallback name="press_selector">
                                    <Sequence name="push_branch">
                                        <Script code=" is_push := (target_type == 'push_button') "/>
                                        <Precondition if="is_push" else="FAILURE">
                                            <PressButton btn_pose="{target_loc}" locked_yaw="{locked_yaw}" depth="0.085"/>
                                        </Precondition>
                                    </Sequence>
                                    <PressButton btn_pose="{target_loc}" locked_yaw="{locked_yaw}" depth="0.4"/>
                                </Fallback>
                                <Retract btn_pose="{target_loc}" door_info="{door_data}"/>
                            </Sequence>
                        </Fallback>
                        
                        <RetryUntilSuccessful num_attempts="-1">
                            <Fallback name="exit_or_wait">
                                <ReactiveSequence name="guarded_exit">
                                    <IsPathClear />
                                    <ExitDoor door_info="{door_data}"/>
                                </ReactiveSequence>
                                <ForceFailure>
                                    <Sequence name="safety_standby">
                                        <Retract btn_pose="{target_loc}" door_info="{door_data}"/>
                                        <Sleep msec="500"/>
                                    </Sequence>
                                </ForceFailure>
                            </Fallback>
                        </RetryUntilSuccessful>
                    </Sequence> 
                </Fallback>
            </Sequence>
        </BehaviorTree>
    </root>
    )";

    auto tree = factory.createTreeFromText(xml_text);


    // BT::StdCoutLogger logger_cout(tree);

    // Replacement for your timer: The BT Tick Loop
    BT::NodeStatus status = BT::NodeStatus::IDLE;
    rclcpp::Rate rate(10); 
    while (rclcpp::ok()) {
        
        rclcpp::spin_some(sequencer_node);
        
        status = tree.tickOnce();
        
        if (status == BT::NodeStatus::SUCCESS) {
            RCLCPP_INFO(sequencer_node->get_logger(), "MISSION %d COMPLETE!", current_mission_index);
            
            // 2. Increment and Publish the new mission state
            current_mission_index++;
            std_msgs::msg::Int32 msg;
            msg.data = current_mission_index;
            mission_count_pub->publish(msg);

            // 3. Reset for the next door
            tree.haltTree(); 
            status = BT::NodeStatus::IDLE;

            RCLCPP_INFO(sequencer_node->get_logger(), "BT RESET: Waiting for Mission %d markers...", current_mission_index);
            
            // Give Python 500ms to swap markers before we start searching again
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        } 
        else if (status == BT::NodeStatus::FAILURE) {
            RCLCPP_ERROR(sequencer_node->get_logger(), "Mission Hard Failure. Resetting to try again.");
            tree.haltTree();
            status = BT::NodeStatus::IDLE;
        }

        rate.sleep();
    }

    rclcpp::shutdown();
    return 0;
}
