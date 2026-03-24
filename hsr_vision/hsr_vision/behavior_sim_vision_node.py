import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Int32
from tf2_ros import TransformListener, Buffer
import math

class BehaviorVisionNode(Node):
    def __init__(self):
        super().__init__('behavior_vision_node')
        self.publisher_ = self.create_publisher(MarkerArray, '/visualization_marker', 10)

        self.count_sub = self.create_subscription(Int32, '/completed_mission_count', self.mission_cb, 10)
        self.mission_index = 0

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.timer = self.create_timer(0.125, self.timer_callback)
        self.get_logger().info(f"Behavior Vision Node: Starting at mission_index={self.mission_index}")

    def mission_cb(self, msg):
        self.mission_index = msg.data
        self.get_logger().info(f"Swapping to Mission Index: {self.mission_index}")

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
        full_array = MarkerArray()

        clear_marker = Marker()
        clear_marker.action = Marker.DELETEALL
        full_array.markers.append(clear_marker)

        if self.mission_index == 0:
            # Entering: front-right button, doors open inward
            full_array.markers.append(self.create_button_marker(now, [2.94, -1.35, 1.0], tid=101, btn_type="push"))
            full_array.markers.extend(self.create_door_markers(now, [3.0, -0.05, 0.0], [3.0, -1.005, 0.0], tid=101))
            full_array.markers.extend(self.create_door_markers(now, [3.0, 1.005, 0.0], [3.0,  0.05, 0.0], tid=102))

        elif self.mission_index == 1:
            # Exiting: back-left button, doors flipped
            full_array.markers.append(self.create_button_marker(now, [3.06, 1.35, 1.0], tid=103, btn_type="push"))
            full_array.markers.extend(self.create_door_markers(now, [3.0, -1.005, 0.0], [3.0, -0.05, 0.0], tid=101))
            full_array.markers.extend(self.create_door_markers(now, [3.0,  0.05, 0.0], [3.0,  1.005, 0.0], tid=102))

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


def main(args=None):
    rclpy.init(args=args)
    node = BehaviorVisionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()