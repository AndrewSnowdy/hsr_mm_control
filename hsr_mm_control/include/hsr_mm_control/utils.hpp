#pragma once

#include <geometry_msgs/msg/pose.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>

namespace standoff_utils
{

bool worldToMap(
    const nav_msgs::msg::OccupancyGrid &costmap,
    double wx, double wy,
    unsigned int &mx, unsigned int &my);

bool costAtWorld(
    const nav_msgs::msg::OccupancyGrid &costmap,
    double wx, double wy,
    unsigned char &cost);

bool compute_feasible_standoff(
    double button_x, double button_y,
    double robot_x, double robot_y, // New: Current robot location
    const nav_msgs::msg::OccupancyGrid &costmap,
    double radius, int num_samples, int8_t max_allowed_cost,
    geometry_msgs::msg::Pose &best_pose,
    std::vector<geometry_msgs::msg::Pose> &all_feasible_poses);

} // namespace standoff_utils
