#pragma once

#include <geometry_msgs/msg/pose.hpp>
#include <nav2_msgs/msg/costmap.hpp>

namespace standoff_utils
{

geometry_msgs::msg::Pose compute_feasible_standoff(
    double button_x,
    double button_y,
    const nav2_msgs::msg::Costmap &costmap,
    double radius,
    int num_samples = 32,
    unsigned char max_allowed_cost = 80);

bool worldToMap(
    const nav2_msgs::msg::Costmap &costmap,
    double wx, double wy,
    unsigned int &mx, unsigned int &my);

unsigned char getCost(
    const nav2_msgs::msg::Costmap &costmap,
    unsigned int mx, unsigned int my);

}
