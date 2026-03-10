import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
import math

class SimVisionNode(Node):
    def __init__(self):
        super().__init__('sim_vision_node')
        self.publisher_ = self.create_publisher(MarkerArray, '/visualization_marker', 10)
        
        # 8Hz match for hardware vision
        self.timer = self.create_timer(0.125, self.timer_callback)
        self.get_logger().info("Sim Vision Node: Publishing 4 Pillars (2 Doors) and 1 Button.")

    def timer_callback(self):
        now = self.get_clock().now().to_msg()
        full_array = MarkerArray()

        # 1. Housekeeping: Reset RViz display
        clear_marker = Marker()
        clear_marker.action = Marker.DELETEALL
        full_array.markers.append(clear_marker)

        # 2. Add ADA Button (Green)
        # Position: [2.94, -1.35, 1.0]
        button = self.create_button_marker(now, [2.94, -1.35, 1.0], tid=0)
        full_array.markers.append(button)

        # 3. Door 1: Handicap Door Right (Orange)
        # Hinge is at [3.0, -1.005], Edge is at [3.0, 0.0]
        door_r = self.create_door_markers(now, [3.0, -0.05, 0.0], [3.0, -1.005, 0.0], tid=101)
        full_array.markers.extend(door_r)

        # 4. Door 2: Handicap Door Left (Orange)
        # Hinge is at [3.0, 1.005], Edge is at [3.0, 0.0]
        door_l = self.create_door_markers(now, [3.0, 1.005, 0.0], [3.0, 0.05, 0.0], tid=102)
        full_array.markers.extend(door_l)

        self.publisher_.publish(full_array)

    def create_button_marker(self, timestamp, pos, tid):
        m = Marker()
        m.header.frame_id = "odom"
        m.header.stamp = timestamp
        m.ns = f"button_{tid}"
        m.id = 0
        m.type = Marker.SPHERE
        m.action = Marker.ADD
        m.pose.position.x, m.pose.position.y, m.pose.position.z = pos
        m.pose.orientation.w = 1.0
        m.scale.x = m.scale.y = m.scale.z = 0.12
        # Color: Green (0.0, 1.0, 0.0)
        m.color.r, m.color.g, m.color.b, m.color.a = 0.0, 1.0, 0.0, 0.8
        return m

    def create_door_markers(self, timestamp, p_l, p_r, tid):
        markers = []
        # Color: Orange (1.0, 0.5, 0.0) from your class_configs
        color_orange = (1.0, 0.5, 0.0)
        
        # Two Pillars per door
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

        # Normal Arrow per door
        dx, dy = p_r[0] - p_l[0], p_r[1] - p_l[1]
        nx, ny = -dy, dx # Perpendicular
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
        arrow.color.r, arrow.color.g, arrow.color.b, arrow.color.a = 1.0, 1.0, 1.0, 1.0 # White
        markers.append(arrow)
        
        return markers

def main(args=None):
    rclpy.init(args=args)
    node = SimVisionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()