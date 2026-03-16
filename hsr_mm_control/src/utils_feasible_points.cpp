#include "hsr_mm_control/utils_feasible_points.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <cmath>
#include <limits>

namespace feasible_standoff_utils
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

    // PrePress logic
    double compute_optimized_standoff(
        double bx, double by,
        double rx, double ry, // New: Current robot location
        const nav_msgs::msg::OccupancyGrid &costmap)
    {
        // --- Hardcoded Tuning Params ---
        const double radius = 0.80;        // Distance from button to stand
        const int num_samples = 72;        // 5-degree increments
        const int8_t max_allowed_cost = 15; // Stay away from walls/obstacles
        const double W_cost = 1.0; 
        const double W_dist = 5.0;

        double best_score = std::numeric_limits<double>::infinity();
        double best_yaw = -999.0;

    for (int i = 0; i < num_samples; ++i) {
        // 1. Calculate candidate coordinates on the circle
        const double theta = (2.0 * M_PI * i) / num_samples;
        const double gx = bx + radius * std::cos(theta);
        const double gy = by + radius * std::sin(theta);

        // 2. Check the costmap (is this point inside a wall?)
        int8_t c;
        if (!costAtWorld(costmap, gx, gy, c)) continue;
        if (c == -1 || c >= max_allowed_cost) continue;

        // 3. Score the point based on cost and distance to robot
        double dist_to_robot = std::hypot(gx - rx, gy - ry);
        double current_score = (double(c) * W_cost) + (dist_to_robot * W_dist);

        if (current_score < best_score) {
            best_score = current_score;
            // The yaw needed to face the button from this point
            best_yaw = std::atan2(by - gy, bx - gx);
        }
    }

    return best_yaw; 
}

    // Approach Logic
    geometry_msgs::msg::Pose compute_circular_target(
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

        // // 4) face the button
        // double yaw = std::atan2(by - goal.position.y, bx - goal.position.x);

        // tf2::Quaternion q;
        // q.setRPY(0, 0, yaw);
        // goal.orientation.x = q.x();
        // goal.orientation.y = q.y();
        // goal.orientation.z = q.z();
        // goal.orientation.w = q.w();

        set_pose_yaw(goal, std::atan2(by - goal.position.y, bx - goal.position.x));

        return goal;
    }

    geometry_msgs::msg::Pose compute_ee_target(
    double bx, double by, double bz, double yaw, double offset) 
    {
        geometry_msgs::msg::Pose target;
        // We move BACKWARDS from the button along the yaw axis
        target.position.x = bx - offset * std::cos(yaw);
        target.position.y = by - offset * std::sin(yaw);
        target.position.z = bz;
        
        set_pose_yaw(target, yaw);
        return target;
    }

    double get_pose_yaw(const geometry_msgs::msg::Pose &pose) {
        tf2::Quaternion q(
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w);
        double r, p, y;
        tf2::Matrix3x3(q).getRPY(r, p, y);
        return y;
    }

    void set_pose_yaw(geometry_msgs::msg::Pose &pose, double yaw) {
        tf2::Quaternion q;
        q.setRPY(0, 0, yaw);
        pose.orientation.x = q.x();
        pose.orientation.y = q.y();
        pose.orientation.z = q.z();
        pose.orientation.w = q.w();
    }

} // namespace standoff_utils