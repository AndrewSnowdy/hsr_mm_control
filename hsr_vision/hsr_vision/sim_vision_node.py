import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Int32
from tf2_ros import TransformListener, Buffer
import math

class SimVisionNode(Node):
    def __init__(self):
        super().__init__('sim_vision_node')
        self.publisher_ = self.create_publisher(MarkerArray, '/visualization_marker', 10)
        

        self.count_sub = self.create_subscription(Int32, '/completed_mission_count', self.mission_cb, 10)
        self.mission_index = 0

        # TF tracking
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.timer = self.create_timer(0.125, self.timer_callback)
        
        # State: DOOR_1_IN -> DOOR_2_IN -> DOOR_2_OUT -> DOOR_1_OUT
        self.current_goal = "DOOR_1_IN"
        self.get_logger().info(f"Sim Vision Director: Starting at {self.current_goal}")

    def mission_cb(self, msg):
        self.mission_index = msg.data
        self.get_logger().info(f"Swapping to Mission Index: {self.mission_index}")

    def get_robot_pose(self):
        try:
            # Check robot position in odom
            t = self.tf_buffer.lookup_transform('odom', 'base_link', rclpy.time.Time())
            return t.transform.translation
        except:
            return None

    def timer_callback(self):
        pos = self.get_robot_pose()
        if not pos: return

        now = self.get_clock().now().to_msg()
        full_array = MarkerArray()

        # 1. Housekeeping: Reset RViz
        clear_marker = Marker()
        clear_marker.action = Marker.DELETEALL
        full_array.markers.append(clear_marker)

        # 3. Publish Active Markers based on mission_index
        if self.mission_index == 0:
            # Main Double Doors - Entering
            full_array.markers.append(self.create_button_marker(now, [2.94, -1.35, 1.0], tid=101, btn_type="push"))
            full_array.markers.extend(self.create_door_markers(now, [3.0, -0.05, 0.0], [3.0, -1.005, 0.0], tid=101))
            full_array.markers.extend(self.create_door_markers(now, [3.0, 1.005, 0.0], [3.0, 0.05, 0.0], tid=102))

        elif self.mission_index == 1: # Fixed variable name
            # Side Room Door - Entering
            full_array.markers.append(self.create_button_marker(now, [6.75, -1.44, 1.0], tid=201, btn_type="push"))
            full_array.markers.extend(self.create_door_markers(now, [7.95, -1.5, 0.0], [7.05, -1.5, 0.0], tid=201))

        elif self.mission_index == 2:
            can_pose = [5.3, -4.35, 0.811]
            full_array.markers.append(self.create_button_marker(now, can_pose, tid=202, btn_type="coke_can"))

        elif self.mission_index == 3: # Fixed variable name
            # Side Room Door - Exiting
            full_array.markers.append(self.create_button_marker(now, [6.75, -1.56, 1.0], tid=202, btn_type="push"))
            full_array.markers.extend(self.create_door_markers(now, [7.05, -1.5, 0.0], [7.95, -1.5, 0.0], tid=202))

        elif self.mission_index == 4: # Fixed variable name
            # Main Double Doors - Exiting
            full_array.markers.append(self.create_button_marker(now, [4.8, 1.44, 1.0], tid=103, btn_type="push"))
            full_array.markers.extend(self.create_door_markers(now, [3.0, -1.005, 0.0], [3.0, -0.05, 0.0], tid=101))
            full_array.markers.extend(self.create_door_markers(now, [3.0, 0.05, 0.0], [3.0, 1.005, 0.0], tid=102))

        elif self.mission_index == 5:
            can_pose = [-0.83, 0.9, 0.811]
            full_array.markers.append(self.create_button_marker(now, can_pose, tid=202, btn_type="coke_can"))

        self.publisher_.publish(full_array)

    def create_button_marker(self, timestamp, pos, tid, btn_type="push"):
        m = Marker()
        m.header.frame_id = "odom"
        m.header.stamp = timestamp
        # Encode 'push' or 'prox' into the namespace so C++ can decipher it
        m.ns = f"button_{btn_type}_{tid}"
        m.id = 0
        m.type = Marker.SPHERE
        m.action = Marker.ADD
        m.pose.position.x, m.pose.position.y, m.pose.position.z = pos
        m.pose.orientation.w = 1.0
        m.scale.x = m.scale.y = m.scale.z = 0.12
        # Push = Green, Prox = Blue
        if btn_type == "push":
            m.color.r, m.color.g, m.color.b, m.color.a = 0.0, 1.0, 0.0, 0.8
        else:
            m.color.r, m.color.g, m.color.b, m.color.a = 0.0, 0.0, 1.0, 0.8
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

    def create_object_marker(self, timestamp, pos, tid, obj_type="can"):
        m = Marker()
        m.header.frame_id = "odom"
        m.header.stamp = timestamp
        m.ns = f"object_{obj_type}_{tid}"
        m.id = tid
        m.type = Marker.CYLINDER
        m.action = Marker.ADD
        m.pose.position.x, m.pose.position.y, m.pose.position.z = pos
        m.pose.orientation.w = 1.0
        # Dimensions from your Gazebo XML
        m.scale.x = 0.033 * 2 # Diameter
        m.scale.y = 0.033 * 2
        m.scale.z = 0.122
        # Red color for the "Coke" can
        m.color.r, m.color.g, m.color.b, m.color.a = 0.8, 0.0, 0.0, 1.0
        return m

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