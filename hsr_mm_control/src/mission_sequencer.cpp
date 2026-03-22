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
            BT::OutputPort<std::string>("btn_type") // Added this port
        };
    }

    BT::NodeStatus tick() override {
        // Now returns std::optional<ButtonInfo>
        auto result = s_->get_closest_button(); 
        
        if (result) {
            // result is the struct; result->pose and result->type are the fields
            setOutput("btn_pose", result->pose);
            setOutput("btn_type", result->type);

            RCLCPP_INFO(s_->get_logger(), "BT: Found %s! Passing to Blackboard.", result->type.c_str());
            return BT::NodeStatus::SUCCESS;
        }
        
        return BT::NodeStatus::FAILURE;
    }
private:
    MissionSequencer* s_;
};

// --- NODE 2: APPROACH (Replaces case APPROACH) ---
class ApproachButton : public BT::StatefulActionNode {
public:
    ApproachButton(const std::string& name, const BT::NodeConfig& config, MissionSequencer* s)
        : BT::StatefulActionNode(name, config), s_(s) {}

    static BT::PortsList providedPorts() {
        return { BT::InputPort<geometry_msgs::msg::Pose>("btn_pose") };
    }

    // Runs once when the node is first called
    BT::NodeStatus onStart() override {
        if (!getInput("btn_pose", target_button_)) {
            return BT::NodeStatus::FAILURE;
        }
        start_dist_ = -1.0; // Reset interpolation tracking
        return BT::NodeStatus::RUNNING;
    }

    // Runs repeatedly until it returns SUCCESS or FAILURE
    BT::NodeStatus onRunning() override {

        if (!getInput("btn_pose", target_button_)) {
            return BT::NodeStatus::RUNNING; 
        }


        bool flat = s_->force_wrist_flat;
        double gripper = s_->target_gripper;


        auto robot = s_->get_base_position();
        if (!robot) return BT::NodeStatus::RUNNING;


        // --- YOUR ORIGINAL LOGIC ---
        auto final_standoff = feasible_standoff_utils::compute_circular_target(
            target_button_.position.x, target_button_.position.y, robot->x, robot->y, 1.25);
        
        if (start_dist_ < 0) {
            start_dist_ = std::hypot(final_standoff.position.x - robot->x, final_standoff.position.y - robot->y);
            start_pose_ = robot->pose;
        }

        double current_dist = std::hypot(final_standoff.position.x - robot->x, final_standoff.position.y - robot->y);
        double progress = 1.0 - (current_dist / start_dist_);
        double heading = std::atan2(final_standoff.position.y - robot->y, final_standoff.position.x - robot->x);

        if (progress < 0.24) {
            auto next_waypoint = s_->interpolate_pose(start_pose_, final_standoff, 0.26);
            s_->publish_mission_goal(next_waypoint, false, 0.225, heading, flat, gripper);
        } else if (progress < 0.74) {
            auto next_waypoint = s_->interpolate_pose(start_pose_, final_standoff, 0.76);
            s_->publish_mission_goal(next_waypoint, false, 0.25, heading, flat, gripper);
        } else {
            s_->publish_mission_goal(final_standoff, false, 0.0, 0.0, flat, gripper);
        }

        // The "Spatial Switch" trigger
        if (s_->base_close_xyw(*robot, final_standoff, 0.10, 0.20)) {
            RCLCPP_INFO(s_->get_logger(), "BT: Approach Complete.");
            return BT::NodeStatus::SUCCESS;
        }

        return BT::NodeStatus::RUNNING;
    }

    void onHalted() override { /* Stop robot if interrupted */ }

private:
    MissionSequencer* s_;
    geometry_msgs::msg::Pose target_button_;
    geometry_msgs::msg::Pose start_pose_;
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

        bool flat = s_->force_wrist_flat;
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
        s_->publish_mission_goal(target_pose, true, 0.0, 0.0, flat, gripper);

        // 6. Check if EE has arrived (2cm tolerance)
        if (s_->ee_close_xyz(*ee_pos, target_pose.position.x, target_pose.position.y, target_pose.position.z, 0.02)) {
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


class PressButton : public BT::StatefulActionNode {
public:
    PressButton(const std::string& name, const BT::NodeConfig& config, MissionSequencer* s)
        : BT::StatefulActionNode(name, config), s_(s) {}

    static BT::PortsList providedPorts() {
        return { BT::InputPort<geometry_msgs::msg::Pose>("btn_pose"),
                 BT::InputPort<double>("locked_yaw"),
                 BT::InputPort<double>("depth") }; // New Port
    }

    BT::NodeStatus onStart() override {
        if (!getInput("btn_pose", target_button_)) return BT::NodeStatus::FAILURE;
        if (!getInput("locked_yaw", standoff_yaw_)) return BT::NodeStatus::FAILURE;
        
        // If depth isn't provided in XML, default to 0.06
        if (!getInput("depth", target_depth_)) {
            target_depth_ = 0.06;
        }
        
        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override {
        auto ee_pos = s_->get_ee_position();
        if (!ee_pos) return BT::NodeStatus::RUNNING;

        bool flat = s_->force_wrist_flat;
        double gripper = s_->target_gripper;
        double offset = s_->depth_offset;
        double adjusted_depth = target_depth_ + offset;


        // Use the target_depth_ we got from the Blackboard/XML
        auto press_pose = feasible_standoff_utils::compute_ee_target(
            target_button_.position.x, target_button_.position.y, target_button_.position.z, 
            standoff_yaw_, adjusted_depth);
        
        s_->set_yaw(press_pose, standoff_yaw_);
        s_->publish_mission_goal(press_pose, true, 0.0, 0.0, flat, gripper);

        // Arrival check
        if (s_->ee_close_xyz(*ee_pos, press_pose.position.x, press_pose.position.y, press_pose.position.z, 0.015)) {
            RCLCPP_INFO(s_->get_logger(), "BT: Press/Wave at depth %.2fm Complete.", target_depth_);
            return BT::NodeStatus::SUCCESS;
        }

        return BT::NodeStatus::RUNNING;
    }

    void onHalted() override {}

private:
    MissionSequencer* s_;
    geometry_msgs::msg::Pose target_button_;
    double standoff_yaw_;
    double target_depth_; // Store the depth here
};


class Retract : public BT::StatefulActionNode {
public:
    Retract(const std::string& name, const BT::NodeConfig& config, MissionSequencer* s)
        : BT::StatefulActionNode(name, config), s_(s) {}

    static BT::PortsList providedPorts() {
        return { BT::InputPort<geometry_msgs::msg::Pose>("btn_pose") };
    }

    BT::NodeStatus onStart() override {
        if (!getInput("btn_pose", target_button_)) return BT::NodeStatus::FAILURE;
        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override {
        auto robot = s_->get_base_position();
        if (!robot) return BT::NodeStatus::RUNNING;


        bool flat = s_->force_wrist_flat;
        double gripper = s_->target_gripper;

        // 1. Find the door markers near the button
        auto door = s_->get_complete_door(target_button_.position.x, target_button_.position.y);
        if (!door) {
            RCLCPP_WARN_THROTTLE(s_->get_logger(), *s_->get_clock(), 2000, "Retract: Searching for door markers...");
            return BT::NodeStatus::RUNNING;
        }

        // 2. Get door orientation
        tf2::Quaternion q(door->center.orientation.x, door->center.orientation.y, 
                        door->center.orientation.z, door->center.orientation.w);
        double r, p, door_yaw;
        tf2::Matrix3x3(q).getRPY(r, p, door_yaw);

        // 3. Calculate target: "behind" the pillar along the door's axis
        double dynamic_standoff = s_->get_door_safe_standoff(target_button_, door->pillar1, door->pillar2);

        geometry_msgs::msg::Pose target_pose;
        target_pose.position.x = door->pillar1.position.x - dynamic_standoff * std::cos(door_yaw);
        target_pose.position.y = door->pillar1.position.y - dynamic_standoff * std::sin(door_yaw);
        
        // 4. Set heading to face the door center (preparing to drive through)
        double angle_to_center = std::atan2(door->center.position.y - target_pose.position.y, 
                                            door->center.position.x - target_pose.position.x);
        s_->set_yaw(target_pose, angle_to_center);

        // 5. Move (IK mode = false, we want the base to move)
        s_->publish_mission_goal(target_pose, false, 0.0, 0.0, flat, gripper);

        // 6. Check arrival
        if (s_->base_close_xyw(*robot, target_pose, 0.10, 0.15)) {
            RCLCPP_INFO(s_->get_logger(), "BT: Retract Complete.");
            return BT::NodeStatus::SUCCESS;
        }

        return BT::NodeStatus::RUNNING;
    }

    void onHalted() override {}

private:
    MissionSequencer* s_;
    geometry_msgs::msg::Pose target_button_;
};



class ExitDoor : public BT::StatefulActionNode {
public:
    ExitDoor(const std::string& name, const BT::NodeConfig& config, MissionSequencer* s)
        : BT::StatefulActionNode(name, config), s_(s) {}

    static BT::PortsList providedPorts() {
        return { BT::InputPort<geometry_msgs::msg::Pose>("btn_pose") };
    }

    BT::NodeStatus onStart() override {
        if (!getInput("btn_pose", target_button_)) return BT::NodeStatus::FAILURE;
        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override {
        auto robot = s_->get_base_position();
        if (!robot) return BT::NodeStatus::RUNNING;

        auto door = s_->get_complete_door(target_button_.position.x, target_button_.position.y);
        if (!door) return BT::NodeStatus::RUNNING;

        bool flat = s_->force_wrist_flat;
        double gripper = s_->target_gripper;

        double door_yaw = feasible_standoff_utils::get_pose_yaw(door->center);
        
        // 1. Calculate Progress (p) for switching between ghost poses
        double dx = robot->x - door->center.position.x;
        double dy = robot->y - door->center.position.y;
        double p = dx * std::cos(door_yaw) + dy * std::sin(door_yaw);

        // 2. Define our 3 Ghost Poses (Stage 1, Stage 2, and the Final Goal)
        geometry_msgs::msg::Pose stage1 = door->center; // Just past threshold
        stage1.position.x += 0.2 * std::cos(door_yaw);
        stage1.position.y += 0.2 * std::sin(door_yaw);

        geometry_msgs::msg::Pose stage2 = door->center; // Mid-way clear
        stage2.position.x += 0.8 * std::cos(door_yaw);
        stage2.position.y += 0.8 * std::sin(door_yaw);

        geometry_msgs::msg::Pose final_goal = door->center; // Deep clear
        final_goal.position.x += 1.5 * std::cos(door_yaw);
        final_goal.position.y += 1.5 * std::sin(door_yaw);

        // 3. Calculate Distance to Final Goal
        double dist_to_goal = std::hypot(final_goal.position.x - robot->x, 
                                        final_goal.position.y - robot->y);

        // DEBUG: Print status every 500ms
        RCLCPP_INFO_THROTTLE(s_->get_logger(), *s_->get_clock(), 500, 
            "ExitDoor DEBUG: p=%.3f | DistToGoal=%.3f", 
            p, dist_to_goal);

        // 3. THE FINAL ARRIVAL CHECK (The Reset Trigger)
        // We only return SUCCESS when we are physically at the final_goal
        if (s_->base_close_xyw(*robot, final_goal, 0.15, 0.20)) {
            RCLCPP_INFO(s_->get_logger(), "BT: Exit Complete. Arrived at final standoff.");
            return BT::NodeStatus::SUCCESS;
        }

        // 4. THE GHOST POSE RELAY
        // This maintains your specific pathing logic
        if (p < -0.1) {
            // Robot is still in/near the frame, send ghost to Stage 1
            s_->publish_mission_goal(stage1, false, 0.25, door_yaw, flat, gripper);
        } 
        else if (p < 0.6) {
            // Robot is moving through, send ghost to Stage 2
            s_->publish_mission_goal(stage2, false, 0.3, door_yaw, flat, gripper);
        } 
        else {
            // Robot is mostly clear, send ghost to the Final Goal
            s_->publish_mission_goal(final_goal, false, 0.0, door_yaw, flat, gripper);
        } 

        return BT::NodeStatus::RUNNING;
    }

    void onHalted() override {}

private:
    MissionSequencer* s_;
    geometry_msgs::msg::Pose target_button_;
};


// class GraspAndRetract : public BT::StatefulActionNode {
// public:
//     GraspAndRetract(const std::string& name, const BT::NodeConfig& config, MissionSequencer* s)
//         : BT::StatefulActionNode(name, config), s_(s), has_closed_(false) {}

//     static BT::PortsList providedPorts() {
//         return { BT::InputPort<geometry_msgs::msg::Pose>("btn_pose"),
//                  BT::InputPort<double>("locked_yaw"),
//                  BT::InputPort<double>("depth") };
//     }

//     BT::NodeStatus onStart() override {
//         if (!getInput("btn_pose", target_button_)) return BT::NodeStatus::FAILURE;
//         if (!getInput("locked_yaw", standoff_yaw_)) return BT::NodeStatus::FAILURE;
//         getInput("depth", target_depth_);
        
//         // s_->force_wrist_flat = true;
//         // s_->target_gripper = 1.2; // Ensure it starts open
//         return BT::NodeStatus::RUNNING;
//     }

//     BT::NodeStatus onRunning() override {
//         auto ee_pos = s_->get_ee_position();
//         if (!ee_pos) return BT::NodeStatus::RUNNING;

//         auto grasp_pose = feasible_standoff_utils::compute_ee_target(
//             target_button_.position.x, target_button_.position.y, target_button_.position.z,
//             standoff_yaw_, target_depth_);

//         // --- PHASE 1: REACHING ---
//         if (!has_closed_) {
//             // s_->target_gripper = 1.2;
//             s_->set_yaw(grasp_pose, standoff_yaw_);
//             // Still commanding 1.2 (Open) while moving in
//             s_->publish_mission_goal(grasp_pose, true, 0.0, 0.0, true, 1.2);

//             if (s_->ee_close_xyz(*ee_pos, grasp_pose.position.x, grasp_pose.position.y, grasp_pose.position.z, 0.02)) {
//                 RCLCPP_INFO(s_->get_logger(), "BT: Reached object. Closing gripper...");
//                 has_closed_ = true;
//                 std_msgs::msg::Empty msg;
//                 s_->grasp_attach_pub_->publish(msg);
//                 RCLCPP_INFO(s_->get_logger(), "BT: Can welded to hand.");
//                 start_close_time_ = s_->now();
//             }
//             return BT::NodeStatus::RUNNING;
//         }

//         // --- PHASE 2: GRASPING ---
//         if (has_closed_) {
//             // Send the CLOSE command (-0.1)
//             s_->target_gripper = 0.075; // -0.1 - 1.2
//             // s_->depth_offset = 0.07;
//             s_->publish_mission_goal(grasp_pose, true, 0.0, 0.0, true, 0.1);

//             // We MUST wait for the HSR gripper to physically finish (1.0s duration we set earlier)
//             auto elapsed = (s_->now() - start_close_time_).seconds();
//             if (elapsed > 1.5) { 
//                 RCLCPP_INFO(s_->get_logger(), "BT: Grasp firm. Moving to next task.");
//                 return BT::NodeStatus::SUCCESS;
//             }
//         }

//         return BT::NodeStatus::RUNNING;
//     }

//     void onHalted() override {}

// private:
//     MissionSequencer* s_;
//     geometry_msgs::msg::Pose target_button_;
//     double standoff_yaw_;
//     double target_depth_;
//     bool has_closed_; // Local state tracker
//     rclcpp::Time start_close_time_;
// };

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



geometry_msgs::msg::Pose MissionSequencer::interpolate_pose(const geometry_msgs::msg::Pose& start, 
                                                           const geometry_msgs::msg::Pose& end, 
                                                           double ratio) {
    geometry_msgs::msg::Pose p = end;
    p.position.x = start.position.x + ratio * (end.position.x - start.position.x);
    p.position.y = start.position.y + ratio * (end.position.y - start.position.y);
    return p;
}


MissionSequencer::MissionSequencer()
: Node("mission_sequencer"),
    simple_state_(SimpleState::MANUAL)
{
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
        latest_markers_ = *msg; 
    });

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

    for (const auto& m : latest_markers_.markers) {
        // Skip metadata/empty markers
        if (m.action == visualization_msgs::msg::Marker::DELETEALL || m.ns.empty()) continue;
        
        // Search for the prefix "button"
        if (m.ns.find("button") != std::string::npos) {
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
                } else if (m.ns.find("coke_can") != std::string::npos) {
                    info.type = "coke_can";
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

// std::optional<DoorInfo> MissionSequencer::get_complete_door(double bx, double by) {
//     std::optional<geometry_msgs::msg::Pose> best_center;
//     std::string best_ns = "";
//     double best_dist = 1e6;

//     // 1. Find the Arrow (Center) closest to the button
//     for (const auto& m : latest_markers_.markers) {
//         if (m.type == visualization_msgs::msg::Marker::ARROW && m.ns.find("door") != std::string::npos) {
//             double d = std::hypot(m.pose.position.x - bx, m.pose.position.y - by);
//             if (d < 2.0 && d < best_dist) {
//                 best_dist = d;
//                 best_center = m.pose;
//                 // Extract the unique door ID part (e.g., "door_101")
//                 best_ns = m.ns.substr(0, m.ns.find("_arrow"));
//             }
//         }
//     }

//     if (!best_center || best_ns.empty()) return std::nullopt;

//     // 2. Find the Pillar (Cylinder) that belongs to that SAME door ID
//     DoorInfo door;
//     door.center = *best_center;
//     double best_pillar_dist = 1e6;
//     bool found_pillar = false;

//     for (const auto& m : latest_markers_.markers) {
//         // Must match the exact door ID and be a pillar
//         if (m.ns.find(best_ns) != std::string::npos && m.type == visualization_msgs::msg::Marker::CYLINDER) {
//             double d = std::hypot(m.pose.position.x - bx, m.pose.position.y - by);
//             if (d < best_pillar_dist) {
//                 best_pillar_dist = d;
//                 door.pillar = m.pose;
//                 found_pillar = true;
//             }
//         }
//     }

//     if (found_pillar) return door;
//     return std::nullopt;
// }

std::optional<DoorInfo> MissionSequencer::get_complete_door(double bx, double by) {
    std::optional<geometry_msgs::msg::Pose> best_center;
    std::string best_ns = "";
    double best_dist = 1e6;

    // 1. Find the Arrow (Center) closest to the button
    for (const auto& m : latest_markers_.markers) {
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

    for (const auto& m : latest_markers_.markers) {
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
    double standoff = std::max(0.65, std::min(1.75, dist_to_door_line));

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

    // For a sim test only we have this grasp logic
    factory.registerBuilder<GraspAndRetract>("GraspAndRetract", 
        [sequencer_node](const std::string& name, const BT::NodeConfig& config) {
            return std::make_unique<GraspAndRetract>(name, config, sequencer_node.get());
        });

    // Create a simple XML string directly in code for testing
    // std::string xml_text = R"(
    //     <root BTCPP_format="4">
    //         <BehaviorTree ID="MainTree">
    //             <Sequence name="root_sequence">
                    
    //                 <RetryUntilSuccessful num_attempts="-1">
    //                     <Sequence>
    //                         <Sleep msec="100"/>
    //                         <SearchForButton btn_pose="{button_loc}" btn_type="{button_type}"/>
    //                     </Sequence>
    //                 </RetryUntilSuccessful>

                    // <ReactiveSequence name="adaptive_approach">
                    //     <SearchForButton btn_pose="{button_loc}" btn_type="{button_type}"/>
                    //     <ApproachButton btn_pose="{button_loc}"/>
                    // </ReactiveSequence>

    //                 <Sequence name="manipulation_phase">
    //                     <PrePress btn_pose="{button_loc}" final_yaw="{locked_yaw}"/>
                        
    //                     <Fallback name="press_selector">
    //                         <Sequence name="physical_push_branch">
    //                             <Script code=" is_push := (button_type == 'push_button') "/>
    //                             <Precondition if="is_push" else="FAILURE">
    //                                 <PressButton btn_pose="{button_loc}" locked_yaw="{locked_yaw}" depth="0.07"/>
    //                             </Precondition>
    //                         </Sequence>

    //                         <PressButton btn_pose="{button_loc}" locked_yaw="{locked_yaw}" depth="0.12"/>
    //                     </Fallback>
    //                 </Sequence>

    //                 <Retract btn_pose="{button_loc}"/>
    //                 <ExitDoor btn_pose="{button_loc}"/>

    //             </Sequence>
    //         </BehaviorTree>
    //     </root>
    //     )";

    std::string xml_text = R"(
        <root BTCPP_format="4">
            <BehaviorTree ID="MainTree">
                <Sequence name="root_sequence">

                    <RetryUntilSuccessful num_attempts="-1">
                        <Sequence>
                            <Sleep msec="100"/>
                            <SearchForButton btn_pose="{target_loc}" btn_type="{target_type}"/>
                        </Sequence>
                    </RetryUntilSuccessful>

                    <ReactiveSequence name="adaptive_approach">
                        <SearchForButton btn_pose="{target_loc}" btn_type="{target_type}"/>
                        <ApproachButton btn_pose="{target_loc}"/>
                    </ReactiveSequence>

                    <PrePress btn_pose="{target_loc}" final_yaw="{locked_yaw}" />

                    <Fallback name="action_selector">
                        
                        <Sequence name="grasp_branch">
                            <Precondition if="target_type == 'coke_can'" else="FAILURE">
                                <Sequence>
                                    
                                    <GraspAndRetract btn_pose="{target_loc}" locked_yaw="{locked_yaw}" depth="0.04"/>
                                    
                                    <Script code=" is_carrying := true; target_gripper := -0.1 "/>
                                </Sequence>
                            </Precondition>
                        </Sequence>

                        <Sequence name="door_full_sequence">
                            <Fallback name="press_selector">
                                <Sequence name="push_branch">
                                    <Script code=" is_push := (target_type == 'push_button') "/>
                                    <Precondition if="is_push" else="FAILURE">
                                        <PressButton btn_pose="{target_loc}" locked_yaw="{locked_yaw}" depth="0.085"/>
                                    </Precondition>
                                </Sequence>
                                <PressButton btn_pose="{target_loc}" locked_yaw="{locked_yaw}" depth="0.12"/>
                            </Fallback>
                            
                            <Retract btn_pose="{target_loc}"/>
                            <ExitDoor btn_pose="{target_loc}"/>
                        </Sequence>

                    </Fallback>

                </Sequence>
            </BehaviorTree>
        </root>
    )";

    auto tree = factory.createTreeFromText(xml_text);


    BT::StdCoutLogger logger_cout(tree);

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
