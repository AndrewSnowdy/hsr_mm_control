import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Int32
from tf2_ros import TransformListener, Buffer
import math

class Behavior2VisionNode(Node):
    def __init__(self):
        super().__init__('behavior2_vision_node')
        self.publisher_ = self.create_publisher(MarkerArray, '/visualization_marker', 10)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Person is stationary beside the open door (same pose as SDF actor)
        self.person_waypoints = [
            (0.0,    6.5, -1.2, 2.14159),
            (1000.0, 6.5, -1.2, 2.14159),
        ]

        self.timer = self.create_timer(0.125, self.timer_callback)
        self.get_logger().info("Behavior2 Vision Node: Started")

    def get_person_pose_interpolated(self, sim_time_sec):
        # Person never moves so this always returns the same pose,
        # but keeping the same pattern as behavior_sim_vision_node for consistency
        total_cycle = self.person_waypoints[-1][0]
        t_lookup = sim_time_sec % total_cycle if total_cycle > 0 else sim_time_sec

        for i in range(len(self.person_waypoints) - 1):
            t1, x1, y1, yaw1 = self.person_waypoints[i]
            t2, x2, y2, yaw2 = self.person_waypoints[i+1]
            if t1 <= t_lookup <= t2:
                alpha = (t_lookup - t1) / (t2 - t1)
                x = x1 + alpha * (x2 - x1)
                y = y1 + alpha * (y2 - y1)
                return x, y, yaw1
        return self.person_waypoints[-1][1], self.person_waypoints[-1][2], self.person_waypoints[-1][3]

    def get_robot_pose(self):
        try:
            t = self.tf_buffer.lookup_transform('odom', 'base_link', rclpy.time.Time())
            return t.transform.translation
        except:
            return None

    def timer_callback(self):
        pos = self.get_robot_pose()
        if not pos: return

        now = self.get_clock().now().to_msg()
        sim_time_sec = now.sec + (now.nanosec / 1e9)
        full_array = MarkerArray()

        clear_marker = Marker()
        clear_marker.action = Marker.DELETEALL
        full_array.markers.append(clear_marker)

        # Front button at (4.94, -1.35)
        full_array.markers.append(self.create_button_marker(now, [4.94, -1.35, 1.0], tid=101, btn_type="push"))

        # Right door pillars (open, static) — hinge at y=-1.005, doorway center at y=-0.05
        full_array.markers.extend(self.create_door_markers(now, [5.0, -0.05, 0.0], [5.0, -1.005, 0.0], tid=101))

        # Left door pillars — hinge at y=1.005, doorway center at y=0.05
        full_array.markers.extend(self.create_door_markers(now, [5.0, 1.005, 0.0], [5.0,  0.05, 0.0], tid=102))

        person_markers = self.create_person_prediction_markers(now, sim_time_sec)
        full_array.markers.extend(person_markers)

        self.publisher_.publish(full_array)

    def create_button_marker(self, timestamp, pos, tid, btn_type="push"):
        m = Marker()
        m.header.frame_id = "odom"
        m.header.stamp = timestamp
        m.ns = f"button_{btn_type}_{tid}"
        m.id = 0
        m.type = Marker.SPHERE
        m.action = Marker.ADD
        m.pose.position.x, m.pose.position.y, m.pose.position.z = pos
        m.pose.orientation.w = 1.0
        m.scale.x = m.scale.y = m.scale.z = 0.12
        if btn_type == "push":
            m.color.r, m.color.g, m.color.b, m.color.a = 0.0, 1.0, 0.0, 0.8
        else:
            m.color.r, m.color.g, m.color.b, m.color.a = 0.0, 0.0, 1.0, 0.8
        return m

    def create_door_markers(self, timestamp, p_l, p_r, tid):
        markers = []

        for i, pt in enumerate([p_l, p_r]):
            m = Marker()
            m.header.frame_id = "odom"
            m.header.stamp = timestamp
            m.ns = f"door_{tid}_pillars"
            m.id = i
            m.type = Marker.CYLINDER
            m.pose.position.x, m.pose.position.y, m.pose.position.z = pt[0], pt[1], 1.0
            m.scale.x = m.scale.y = 0.08
            m.scale.z = 2.0
            m.color.r, m.color.g, m.color.b, m.color.a = 1.0, 0.0, 0.0, 0.8
            markers.append(m)

        dx, dy = p_r[0] - p_l[0], p_r[1] - p_l[1]
        nx, ny = -dy, dx
        mag = math.hypot(nx, ny)
        if mag > 0: nx /= mag; ny /= mag

        arrow = Marker()
        arrow.header.frame_id = "odom"
        arrow.header.stamp = timestamp
        arrow.ns = f"door_{tid}_arrow"
        arrow.id = 0
        arrow.type = Marker.ARROW
        mx, my = (p_l[0] + p_r[0]) / 2.0, (p_l[1] + p_r[1]) / 2.0
        arrow.pose.position.x, arrow.pose.position.y, arrow.pose.position.z = mx, my, 0.1
        yaw = math.atan2(ny, nx)
        arrow.pose.orientation.z = math.sin(yaw / 2.0)
        arrow.pose.orientation.w = math.cos(yaw / 2.0)
        arrow.scale.x, arrow.scale.y, arrow.scale.z = 0.6, 0.1, 0.1
        arrow.color.r, arrow.color.g, arrow.color.b, arrow.color.a = 1.0, 1.0, 1.0, 1.0
        markers.append(arrow)

        return markers

    def create_person_prediction_markers(self, timestamp, sim_time_sec):
        markers = []
        pred_steps = 4
        pred_dt = 0.3

        for step in range(pred_steps):
            future_t = sim_time_sec + (step * pred_dt)
            px, py, pyaw = self.get_person_pose_interpolated(future_t)

            m = Marker()
            m.header.frame_id = "odom"
            m.header.stamp = timestamp
            m.ns = "people"
            m.id = step
            m.type = Marker.CYLINDER
            m.action = Marker.ADD

            m.pose.position.x = px
            m.pose.position.y = py
            m.pose.position.z = 0.01

            m.pose.orientation.z = math.sin(pyaw / 2.0)
            m.pose.orientation.w = math.cos(pyaw / 2.0)

            scale_factor = 1.0 - (step / (pred_steps + 1))
            m.scale.x = m.scale.y = 0.7 * scale_factor
            m.scale.z = 0.02

            m.color.r, m.color.g, m.color.b = 0.2, 0.4, 1.0
            m.color.a = float(max(0.1, 0.7 - 0.1 * step))

            markers.append(m)
        return markers


def main(args=None):
    rclpy.init(args=args)
    node = Behavior2VisionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()