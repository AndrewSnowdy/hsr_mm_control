#include "hsr_mm_control/utils.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <cmath>
#include <limits>

namespace standoff_utils
{

bool worldToMap(
    const nav2_msgs::msg::Costmap &costmap,
    double wx, double wy,
    unsigned int &mx, unsigned int &my)
{
    double origin_x = costmap.metadata.origin.position.x;
    double origin_y = costmap.metadata.origin.position.y;
    double resolution = costmap.metadata.resolution;

    if (wx < origin_x || wy < origin_y) return false;

    mx = static_cast<unsigned int>((wx - origin_x) / resolution);
    my = static_cast<unsigned int>((wy - origin_y) / resolution);

    if (mx >= costmap.metadata.size_x || my >= costmap.metadata.size_y)
        return false;

    return true;
}

unsigned char getCost(
    const nav2_msgs::msg::Costmap &costmap,
    unsigned int mx, unsigned int my)
{
    unsigned int index = my * costmap.metadata.size_x + mx;
    return costmap.data[index];
}

geometry_msgs::msg::Pose compute_feasible_standoff(
    double button_x,
    double button_y,
    const nav2_msgs::msg::Costmap &costmap,
    double radius,
    int num_samples,
    unsigned char max_allowed_cost)
{
    geometry_msgs::msg::Pose best_pose;
    double best_cost = std::numeric_limits<double>::infinity();

    for (int i = 0; i < num_samples; ++i)
    {
        double theta = (2.0 * M_PI * i) / num_samples;

        double gx = button_x + radius * std::cos(theta);
        double gy = button_y + radius * std::sin(theta);

        unsigned int mx, my;
        if (!worldToMap(costmap, gx, gy, mx, my))
            continue;

        unsigned char cost = getCost(costmap, mx, my);

        // Reject lethal / inflated / unknown cells
        if (cost >= max_allowed_cost)
            continue;

        // Prefer lower-cost (safer) positions
        if (cost < best_cost)
        {
            best_cost = cost;
            best_pose.position.x = gx;
            best_pose.position.y = gy;
            best_pose.position.z = 0.0;

            // Face the button automatically (THIS gives you the yaw you wanted)
            double yaw = std::atan2(button_y - gy, button_x - gx);
            tf2::Quaternion q;
            q.setRPY(0, 0, yaw);

            best_pose.orientation.x = q.x();
            best_pose.orientation.y = q.y();
            best_pose.orientation.z = q.z();
            best_pose.orientation.w = q.w();
        }
    }

    return best_pose;
}

}
