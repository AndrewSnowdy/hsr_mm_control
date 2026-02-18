#include "hsr_mm_control/utils.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <cmath>
#include <limits>

namespace standoff_utils
{

bool worldToMap(
    const nav_msgs::msg::OccupancyGrid &costmap,
    double wx, double wy,
    unsigned int &mx, unsigned int &my)
{
    // OccupancyGrid uses .info, not .metadata
    const double origin_x = costmap.info.origin.position.x;
    const double origin_y = costmap.info.origin.position.y;
    const double resolution = costmap.info.resolution;

    if (!std::isfinite(wx) || !std::isfinite(wy) || resolution <= 0.0) return false;
    if (wx < origin_x || wy < origin_y) return false;

    mx = static_cast<unsigned int>((wx - origin_x) / resolution);
    my = static_cast<unsigned int>((wy - origin_y) / resolution);

    // OccupancyGrid uses .width and .height
    if (mx >= costmap.info.width || my >= costmap.info.height) return false;
    return true;
}

static inline bool getCost(
    const nav_msgs::msg::OccupancyGrid &costmap,
    unsigned int mx, unsigned int my,
    int8_t &out) // OccupancyGrid uses int8_t
{
    const unsigned int width = costmap.info.width;
    const unsigned int height = costmap.info.height;
    if (mx >= width || my >= height) return false;

    const size_t idx = static_cast<size_t>(my) * static_cast<size_t>(width) + static_cast<size_t>(mx);
    if (idx >= costmap.data.size()) return false;

    out = costmap.data[idx];
    return true;
}

bool costAtWorld(
    const nav_msgs::msg::OccupancyGrid &costmap,
    double wx, double wy,
    int8_t &cost)
{
    unsigned int mx, my;
    if (!worldToMap(costmap, wx, wy, mx, my)) return false;
    return getCost(costmap, mx, my, cost);
}

bool compute_feasible_standoff(
    double button_x, double button_y,
    double robot_x, double robot_y, // New: Current robot location
    const nav_msgs::msg::OccupancyGrid &costmap,
    double radius, int num_samples, int8_t max_allowed_cost,
    geometry_msgs::msg::Pose &best_pose,
    std::vector<geometry_msgs::msg::Pose> &all_feasible_poses)
{
    all_feasible_poses.clear();
    double best_score = std::numeric_limits<double>::infinity();
    bool found = false;

    // Weights: Adjust these to tune behavior
    const double W_cost = 1.0; 
    const double W_dist = 5.0; // Higher weight on distance prevents "teleporting" through walls

    for (int i = 0; i < num_samples; ++i) {
        const double theta = (2.0 * M_PI * i) / num_samples;
        const double gx = button_x + radius * std::cos(theta);
        const double gy = button_y + radius * std::sin(theta);

        int8_t c;
        if (!costAtWorld(costmap, gx, gy, c)) continue;
        if (c == -1 || c >= max_allowed_cost) continue;

        geometry_msgs::msg::Pose p;
        p.position.x = gx;
        p.position.y = gy;
        
        // Calculate Yaw to face the button
        double yaw = std::atan2(button_y - gy, button_x - gx);
        tf2::Quaternion q; q.setRPY(0, 0, yaw);
        p.orientation.x = q.x(); p.orientation.y = q.y();
        p.orientation.z = q.z(); p.orientation.w = q.w();

        all_feasible_poses.push_back(p); 

        // --- THE SCORING CALCULATION ---
        double dist_to_robot = std::hypot(gx - robot_x, gy - robot_y);
        double current_score = (double(c) * W_cost) + (dist_to_robot * W_dist);

        if (current_score < best_score) {
            best_score = current_score;
            best_pose = p;
            found = true;
        }
    }
    return found;
}

} // namespace standoff_utils