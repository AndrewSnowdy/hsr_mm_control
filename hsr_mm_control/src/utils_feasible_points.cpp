#include "hsr_mm_control/utils_feasible_points.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <optional>
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
        const nav_msgs::msg::OccupancyGrid &costmap,
        std::vector<geometry_msgs::msg::Pose>* debug_poses)
    {
        // --- Hardcoded Tuning Params ---
        const double radius = 0.61;        // Distance from button to stand
        const int num_samples = 72;        // 5-degree increments
        const int8_t max_allowed_cost = 1; // Stay away from walls/obstacles
        const double W_cost = 1.0; 
        const double W_dist = 1.0;

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

        if (debug_poses) {
            geometry_msgs::msg::Pose p;
            p.position.x = gx;
            p.position.y = gy;
            p.position.z = 0.05;

            // Set orientation to face the button
            double face_button_yaw = std::atan2(by - gy, bx - gx);
            tf2::Quaternion q;
            q.setRPY(0, 0, face_button_yaw);

            // Manual assignment instead of toMsg
            p.orientation.x = q.x();
            p.orientation.y = q.y();
            p.orientation.z = q.z();
            p.orientation.w = q.w();

            debug_poses->push_back(p);
        }

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


    bool is_pose_safe_coords(double wx, double wy, 
                          const nav_msgs::msg::OccupancyGrid& costmap,
                          int8_t threshold)
    {
        int8_t cost;
        if (!costAtWorld(costmap, wx, wy, cost)) return true;
        if (cost == -1) return true;  // unknown = treat as free
        return cost < threshold;
    }

    // utils_feasible_points.cpp — add this alongside your other functions
    geometry_msgs::msg::Pose compute_next_waypoint(
        double rx, double ry,
        const geometry_msgs::msg::Pose& goal,
        const nav_msgs::msg::OccupancyGrid& costmap,
        double step,
        double lookahead)
    {
        double gx = goal.position.x, gy = goal.position.y;
        double dx = gx - rx, dy = gy - ry;
        double dist = std::hypot(dx, dy);
        if (dist < 1e-3) return goal;

        double ux = dx / dist, uy = dy / dist;

        // 1. Ray-cast toward goal
        bool blocked = false;
        double obs_x = 0.0, obs_y = 0.0;
        for (double d = 0.3; d < std::min(dist, lookahead); d += step) {
            double tx = rx + ux * d, ty = ry + uy * d;
            if (!is_pose_safe_coords(tx, ty, costmap)) {
                obs_x = tx - ux * step;
                obs_y = ty - uy * step;
                blocked = true;
                break;
            }
        }

        if (!blocked) return goal;

        // 2. Walk boundary in both directions, return first point with clear LOS to goal
        auto hug = [&](double sign) -> std::optional<geometry_msgs::msg::Pose> {
            double hx = obs_x, hy = obs_y;
            double tang_x = -sign * uy, tang_y = sign * ux;

            for (int i = 0; i < 40; i++) {
                double nx = hx + tang_x * step;
                double ny = hy + tang_y * step;
                if (!is_pose_safe_coords(nx, ny, costmap)) break;
                hx = nx; hy = ny;

                // Check LOS to goal from here
                double cdx = gx - hx, cdy = gy - hy;
                double cdist = std::hypot(cdx, cdy);
                bool clear = true;
                for (double s = step; s < cdist; s += step) {
                    if (!is_pose_safe_coords(hx + (cdx / cdist) * s,
                                            hy + (cdy / cdist) * s, costmap)) {
                        clear = false; break;
                    }
                }
                if (clear) {
                    geometry_msgs::msg::Pose p;
                    p.position.x = hx; p.position.y = hy; p.position.z = 0.0;
                    set_pose_yaw(p, std::atan2(gy - hy, gx - hx));
                    return p;
                }
            }
            return std::nullopt;
        };

        auto left  = hug(+1.0);
        auto right = hug(-1.0);

        auto dist_to_goal = [&](const geometry_msgs::msg::Pose& p) {
            return std::hypot(gx - p.position.x, gy - p.position.y);
        };

        if (left && right)
            return dist_to_goal(*left) < dist_to_goal(*right) ? *left : *right;
        if (left)  return *left;
        if (right) return *right;

        return goal;  // fallback
    }

    bool is_door_open(const double x1, const double y1, const double x2, const double y2,
                        const nav_msgs::msg::OccupancyGrid& costmap) {


        double dist = std::hypot(x2 - x1, y2 - y1);
        if (dist < 0.1) return false; // Sanity check for bad pillar data

        int8_t max_seen_cost = 0;
        int samples = 0;

        // Scan the center "valley" of the door (30% to 70% of the width)
        // This avoids the 'funky' cost dispersal near the actual pillars
        for (double d = 0.3 * dist; d < 0.7 * dist; d += 0.05) {
            double tx = x1 + (x2 - x1) * (d / dist);
            double ty = y1 + (y2 - y1) * (d / dist);

            int8_t cost;
            if (costAtWorld(costmap, tx, ty, cost)) {
                if (cost > max_seen_cost) max_seen_cost = cost;
                samples++;
            }
        }

        
        if (samples == 0) return false;
        if (max_seen_cost == -1) return false;

        // If the 'worst' point in the gap is low cost, the door is open.
        // Threshold 35 allows for some 'bleeding' from the walls.
        return (max_seen_cost < 100);
    }
} // namespace standoff_utils