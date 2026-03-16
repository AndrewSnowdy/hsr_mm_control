#pragma once

#include <geometry_msgs/msg/pose.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>

namespace feasible_standoff_utils
{

    bool worldToMap(
        const nav_msgs::msg::OccupancyGrid &costmap,
        double wx, double wy,
        unsigned int &mx, unsigned int &my);

    bool costAtWorld(
        const nav_msgs::msg::OccupancyGrid &costmap,
        double wx, double wy,
        unsigned char &cost);

    // 1. The Cost-Aware One (Pre-Press)
    // Concept: The best spot considering costmaps and distances
    double compute_optimized_standoff(
        double bx, double by,
        double rx, double ry, 
        const nav_msgs::msg::OccupancyGrid &costmap);
    

    // 2. The Simple Geometric One (Approach)
    // Concept: Just a circle around the button
    geometry_msgs::msg::Pose compute_circular_target(
        double button_x, double button_y, 
        double robot_x, double robot_y, 
        double radius);

    geometry_msgs::msg::Pose compute_ee_target(
            double bx, double by, double bz, 
            double yaw, double offset);

    double get_pose_yaw(const geometry_msgs::msg::Pose &pose);
    void set_pose_yaw(geometry_msgs::msg::Pose &pose, double yaw);


} // namespace standoff_utils
