# import rclpy
# from sensor_msgs.msg import LaserScan
# from rclpy.node import Node
# from vision_msgs.msg import Detection2DArray
# from visualization_msgs.msg import Marker, MarkerArray
# from sensor_msgs.msg import Image
# from geometry_msgs.msg import PoseStamped, PointStamped, Point
# from cv_bridge import CvBridge
# import numpy as np
# import tf2_ros
# from tf2_geometry_msgs import do_transform_point

# class HSRSpatialLocalizer(Node):
#     def __init__(self):
#         super().__init__('hsr_spatial_localizer')
#         self.bridge = CvBridge()
#         self.latest_depth = None
        
#         # Camera Intrinsics
#         self.fx, self.fy = 525.0, 525.0
#         self.cx, self.cy = 320.0, 240.0

#         # TF2 Setup
#         self.tf_buffer = tf2_ros.Buffer()
#         self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

#         self.create_subscription(Image, '/rgb_PS1080_PrimeSense/depth_registered/image_raw', self.depth_cb, 10)
#         self.create_subscription(Detection2DArray, '/detected_objects', self.detection_cb, 10)
#         self.latest_scan = None
#         self.create_subscription(LaserScan, '/hsrb/base_scan', self.scan_cb, 10)

#         self.door_locked = False
#         self.create_subscription(Bool, '/door_lock_trigger', self.lock_cb, 10)

#         self.door_anchors = None
#         # Remove self.goal_pub = self.create_publisher(PoseStamped, '/goal_button', 10)
#         self.world_pub = self.create_publisher(MarkerArray, '/vision/world_model', 10)
#         # self.goal_pub = self.create_publisher(PoseStamped, '/goal_button', 10)
#         # # We use MarkerArray for all visualizations now
#         # self.marker_pub = self.create_publisher(MarkerArray, '/visualization_marker', 10)  
#         self.registry = {}
#         self.tty = 20.0

#         self.confirmed_objects = {}
#         self.stability_threshold = 5 
#         self.distance_threshold = 0.1

#         # --- Tracking State ---
#         self.active_people = {} 
#         self.person_id_counter = 0
#         self.max_disappearance_time = 1.0
#         self.alpha = 0.3 


#     def timer_callback(self):
#         now = self.get_clock().now().to_msg().sec
#         master_array = MarkerArray()
        
#         # List of IDs to prune
#         to_delete = []

#         for obj_id, data in self.registry.items():
#             # Only check TTL if we aren't locked
#             if not self.door_locked:
#                 if (now - data['last_seen']) > self.ttl:
#                     to_delete.append(obj_id)
#                     continue
            
#             # Add active/locked markers to the outbound message
#             master_array.markers.extend(data['markers'])

#         # Clean up the registry
#         for obj_id in to_delete:
#             del self.registry[obj_id]
#             self.get_logger().info(f"Forgot stale object: {obj_id}")

#         if master_array.markers:
#             self.world_pub.publish(master_array)

#     def lock_cb(self, msg):
#         self.door_locked = msg.data
#         self.get_logger().info(f"Door Lock Status: {self.door_locked}")
        
#     def scan_cb(self, msg):
#         self.latest_scan = msg

#     def depth_cb(self, msg):
#         cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
#         if cv_img.dtype == np.uint16:
#             self.latest_depth = cv_img.astype(np.float32) / 1000.0
#         else:
#             self.latest_depth = cv_img.astype(np.float32)

#     def detection_cb(self, msg):
#         if self.latest_depth is None: return
#         detection_time = msg.header.stamp

#         for det in msg.detections:
#             u, v = int(det.bbox.center.position.x), int(det.bbox.center.position.y)
#             label = det.results[0].hypothesis.class_id.lower()

#             # --- 1. Get 3D Point from Depth Camera ---
#             h, w = self.latest_depth.shape[:2]
#             win = 2 
#             depth_patch = self.latest_depth[max(0,v-win):min(h,v+win+1), max(0,u-win):min(w,u+win+1)]
#             valid_depths = depth_patch[np.isfinite(depth_patch) & (depth_patch > 0)]
            
#             if len(valid_depths) == 0: continue
#             z = float(np.median(valid_depths))

#             x_c = (u - self.cx) * z / self.fx
#             y_c = (v - self.cy) * z / self.fy
#             z_c = z

#             # --- 2. Routing Logic (FIXED INDENTATION) ---
#             # Group static items (door/buttons) for stability check
#             if "button" in label or "door" in label:
#                 self.track_object_stability(x_c, y_c, z_c, label, msg.header.frame_id, detection_time, det)
#             elif "person" in label:
#                 # Persons move; transform directly to robot frame without averaging
#                 self.transform_to_robot_frame(x_c, y_c, z_c, "person", msg.header.frame_id, detection_time)

#     def track_object_stability(self, x, y, z, label, frame_id, timestamp, det=None):
#         if label not in self.confirmed_objects:
#             self.confirmed_objects[label] = {'pos': [x, y, z], 'count': 1}
#             return

#         prev = self.confirmed_objects[label]
#         dist = np.linalg.norm(np.array([x, y, z]) - np.array(prev['pos']))
        
#         # If the new detection is close to our 'stable' point
#         if dist < self.distance_threshold:
#             prev['count'] += 1
#             # Smoothly update the internal position (80% old, 20% new)
#             prev['pos'] = (np.array(prev['pos']) * 0.8 + np.array([x, y, z]) * 0.2).tolist()
            
#             # CHANGE: Use '>=' instead of '==' 
#             # This means after frame 5, EVERY new frame updates the goalposts
#             if prev['count'] >= self.stability_threshold:
#                 if "button" in label:
#                     self.transform_to_robot_frame(prev['pos'][0], prev['pos'][1], prev['pos'][2], label, frame_id, timestamp)
#                 elif "door" in label: # and self.latest_scan is not None:
#                     # Goalposts will now 'track' the door as you approach
#                     if not self.door_locked:
#                         self.process_door_goalposts(det, frame_id, timestamp)
#         else:
#             # If the object 'jumps' (drift or new object), reset the count
#             # but don't reset immediately—maybe it was just one bad frame?
#             # For now, a hard reset is safest for your mission.
#             self.confirmed_objects[label] = {'pos': [x, y, z], 'count': 1}


#     def update_person_tracking(self, rx, ry, timestamp):
#         """Euclidean matching to handle multiple people and velocity."""
#         now = timestamp.sec + timestamp.nanosec * 1e-9
#         best_id = None
#         min_dist = 0.8 # Threshold for matching the same person

#         for p_id, data in self.active_people.items():
#             dist = np.linalg.norm(np.array([rx, ry]) - np.array(data['pos']))
#             if dist < min_dist:
#                 min_dist = dist
#                 best_id = p_id

#         if best_id is not None:
#             data = self.active_people[best_id]
#             dt = now - data['last_seen']
            
#             # --- APPLY ALPHA FILTER (Smoothing) ---
#             # Instead of jumping to [rx, ry], we move toward it slowly
#             smooth_x = (self.alpha * rx) + ((1.0 - self.alpha) * data['pos'][0])
#             smooth_y = (self.alpha * ry) + ((1.0 - self.alpha) * data['pos'][1])
            
#             if dt > 0:
#                 # Calculate velocity based on SMOOTHED positions for a stable trail
#                 vx = (smooth_x - data['pos'][0]) / dt
#                 vy = (smooth_y - data['pos'][1]) / dt
                
#                 # Smooth the velocity too (it's often the jumpiest part)
#                 prev_vx, prev_vy = data.get('vel', [0.0, 0.0])
#                 data['vel'] = [
#                     (self.alpha * vx) + ((1.0 - self.alpha) * prev_vx),
#                     (self.alpha * vy) + ((1.0 - self.alpha) * prev_vy)
#                 ]

#                 speed = np.linalg.norm(data['vel'])
#                 if speed < 0.1: # If moving slower than 10cm/s, assume they are still
#                     data['vel'] = [0.0, 0.0]

#             data['pos'] = [smooth_x, smooth_y]
#             data['last_seen'] = now
#         else:
#             best_id = self.person_id_counter
#             self.active_people[best_id] = {'pos': [rx, ry], 'last_seen': now, 'vel': [0.0, 0.0]}
#             self.person_id_counter += 1

#         # Clean up lost tracks
#         self.active_people = {i: d for i, d in self.active_people.items() 
#                              if now - d['last_seen'] < self.max_disappearance_time}
        
#         return best_id, self.active_people[best_id]['vel']


#     def transform_to_robot_frame(self, x, y, z, label, camera_frame, timestamp):
#         try:
#             point_camera = PointStamped()
#             point_camera.header.frame_id = camera_frame
#             point_camera.header.stamp = timestamp
#             point_camera.point.x, point_camera.point.y, point_camera.point.z = float(x), float(y), float(z)

#             transform = self.tf_buffer.lookup_transform('map', camera_frame, timestamp, 
#                                                         timeout=rclpy.duration.Duration(seconds=0.1))
#             point_robot = do_transform_point(point_camera, transform)
#             rx, ry = float(point_robot.point.x), float(point_robot.point.y)

#             if "button" in label:
#                 self.publish_button_marker(point_robot)
#             elif "door" in label:
#                 # Orange disc for doors
#                 self.publish_door_marker(rx, ry)

#         except Exception as e:
#             pass 


#     def publish_door_marker(self, x, y):
#         marker_array = MarkerArray()
#         marker = Marker()
#         marker.header.frame_id = "base_link"
#         marker.header.stamp = self.get_clock().now().to_msg()
#         marker.ns = "doors"
#         marker.id = 0
#         marker.type = Marker.CYLINDER
#         marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = x, y, 0.0
#         marker.scale.x, marker.scale.y, marker.scale.z = 0.8, 0.8, 0.02
#         # Orange Color for doors
#         marker.color.r, marker.color.g, marker.color.b, marker.color.a = 1.0, 0.5, 0.0, 0.5
#         marker.lifetime = rclpy.duration.Duration(seconds=0.5).to_msg()
#         marker_array.markers.append(marker)
#         self.marker_pub.publish(marker_array)


#     def publish_person_trails(self, x, y, label, person_id, velocity=[0.0, 0.0]):
#         marker_array = MarkerArray()
#         speed = np.linalg.norm(velocity)
#         num_prediction_steps = 5 
#         base_color = (0.2, 0.4, 1.0) # Blue
#         unique_ns = f"person_{person_id}"

#         for step in range(num_prediction_steps):
#             marker = Marker()
#             marker.header.frame_id = "base_link"
#             marker.header.stamp = self.get_clock().now().to_msg()
#             marker.ns = unique_ns
#             marker.id = step
#             marker.type = Marker.CYLINDER
#             marker.action = Marker.ADD
            
#             vanishing_scale = 1.0 - (step / (num_prediction_steps + 2)) 
#             vanishing_alpha = 0.6 - (step * 0.1) 
            
#             t_offset = step * 0.2
#             px = x + (velocity[0] * t_offset)
#             py = y + (velocity[1] * t_offset)
            
#             marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = float(px), float(py), 0.01
#             marker.pose.orientation.w = 1.0
            
#             radius = 0.3 * vanishing_scale
#             marker.scale.x, marker.scale.y, marker.scale.z = radius * 2, radius * 2, 0.02
            
#             marker.color.r, marker.color.g, marker.color.b = base_color
#             marker.color.a = float(max(0.1, vanishing_alpha)) # Ensure it doesn't go below 0.1
#             marker.lifetime = rclpy.duration.Duration(seconds=0.3).to_msg()
#             marker_array.markers.append(marker)
            
#         self.marker_pub.publish(marker_array)


#     def publish_button_marker(self, point_robot):
#         #FUTURE:
#         # Add normal to button marker so it know how to push the button
#         msg = MarkerArray()
#         m = Marker()
#         m.header.frame_id = "map"
#         m.header.stamp = self.get_clock().now().to_msg()
#         m.ns = "button_target"
#         m.id = 0
#         m.type = Marker.SPHERE
#         m.pose.position = point_robot.point
#         m.scale.x = m.scale.y = m.scale.z = 0.1
#         m.color.r, m.color.g, m.color.b, m.color.a = 1.0, 1.0, 0.0, 1.0 # Yellow
#         m.lifetime = rclpy.duration.Duration(seconds=0.5).to_msg()
#         msg.markers.append(m)
#         self.world_pub.publish(msg)


#     def process_door_goalposts(self, det, camera_frame, timestamp):
#         # 1. Get Image Edges (Goalpost pixels)
#         u_l = det.bbox.center.position.x - (det.bbox.size_x / 2.0)
#         u_r = det.bbox.center.position.x + (det.bbox.size_x / 2.0)
        
#         # 2. Convert Pixels to Angles (Radians)
#         angle_l = -np.arctan2((u_l - self.cx), self.fx)
#         angle_r = -np.arctan2((u_r - self.cx), self.fx)

#         # 3. Get Physical Distance from LiDAR (Glass-Proof)
#         dist_l = self.get_scan_dist(angle_l)
#         dist_r = self.get_scan_dist(angle_r)

#         # 4. Project and Transform to Map Frame
#         # We transform to 'map' so if the robot turns, the posts stay in place
#         map_l = self.project_and_transform(dist_l, angle_l, camera_frame, timestamp)
#         map_r = self.project_and_transform(dist_r, angle_r, camera_frame, timestamp)

#         if map_l and map_r:
#             self.door_anchors = {'left': map_l, 'right': map_r}
#             self.publish_door_visuals(map_l, map_r)


#     def get_scan_dist(self, target_angle):
#         """Finds the LiDAR distance at a specific relative angle."""
#         if self.latest_scan is None: return 1.5
#         idx = int((target_angle - self.latest_scan.angle_min) / self.latest_scan.angle_increment)
#         idx = max(0, min(idx, len(self.latest_scan.ranges) - 1))
#         dist = self.latest_scan.ranges[idx]
#         return dist if (np.isfinite(dist) and dist > 0.1) else 2.0


#     def project_and_transform(self, dist, angle, from_frame, timestamp):
#         """Projects a polar coordinate to 3D and transforms to map frame."""
#         try:
#             p = PointStamped()
#             p.header.frame_id = from_frame
#             p.header.stamp = timestamp
#             p.point.x = dist * np.cos(angle)
#             p.point.y = dist * np.sin(angle)
#             p.point.z = 0.0
            
#             # Transform to 'map' for global persistence
#             transform = self.tf_buffer.lookup_transform('map', from_frame, timestamp, 
#                                                         rclpy.duration.Duration(seconds=0.1))
#             return do_transform_point(p, transform).point
#         except:
#             return None


#     def publish_door_visuals(self, p_l, p_r):
#         marker_array = MarkerArray()
#         now = self.get_clock().now().to_msg()
        
#         # 1. Door Frames (Skinny Red Pillars)
#         for i, pt in enumerate([p_l, p_r]):
#             m = Marker()
#             m.header.frame_id = "map"
#             m.header.stamp = now
#             m.ns = "door_frame"
#             m.id = i
#             m.type = Marker.CYLINDER
#             m.pose.position = pt
#             m.pose.position.z = 1.0 # 2m tall, centered at 1m height
#             m.scale.x = m.scale.y = 0.05 # Skinny
#             m.scale.z = 2.0
#             m.color.r, m.color.g, m.color.b, m.color.a = 1.0, 0.0, 0.0, 0.8
#             marker_array.markers.append(m)

#         # 2. Calculate Midpoint and Normal
#         mx = (p_l.x + p_r.x) / 2.0
#         my = (p_l.y + p_r.y) / 2.0
#         dx, dy = p_r.x - p_l.x, p_r.y - p_l.y
#         dist = np.sqrt(dx**2 + dy**2)
#         nx, ny = -dy / dist, dx / dist # Perpendicular unit vector

#         # 3. Green Center (Disc form, like people tracking)
#         center_m = Marker()
#         center_m.header.frame_id = "map"
#         center_m.header.stamp = now
#         center_m.ns = "safe_passage"
#         center_m.id = 0
#         center_m.type = Marker.CYLINDER
#         center_m.pose.position.x, center_m.pose.position.y, center_m.pose.position.z = mx, my, 0.01
#         center_m.scale.x = center_m.scale.y = 0.6 # Width of the disc
#         center_m.scale.z = 0.02 # Flat disc
#         center_m.color.r, center_m.color.g, center_m.color.b, center_m.color.a = 0.0, 1.0, 0.0, 0.8
#         marker_array.markers.append(center_m)

#         # 4. Exit Waypoint (Optional but helpful for FSM)
#         exit_m = Marker()
#         exit_m.header.frame_id = "map"
#         exit_m.ns = "exit_waypoint"
#         exit_m.id = 0
#         exit_m.type = Marker.ARROW
#         exit_m.pose.position.x, exit_m.pose.position.y = mx + nx*1.5, my + ny*1.5
#         # Logic to point the arrow toward the exit
#         yaw = np.atan2(ny, nx)
#         exit_m.pose.orientation.z = np.sin(yaw/2.0)
#         exit_m.pose.orientation.w = np.cos(yaw/2.0)
#         exit_m.scale.x, exit_m.scale.y, exit_m.scale.z = 1.0, 0.1, 0.1
#         exit_m.color.r, exit_m.color.g, exit_m.color.b, exit_m.color.a = 0.0, 0.5, 1.0, 1.0
#         marker_array.markers.append(exit_m)

#         self.world_pub.publish(marker_array)

# def main(args=None):
#     rclpy.init(args=args)
#     node = HSRSpatialLocalizer()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()

# if __name__ == '__main__':
#     main()
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Image
from vision_msgs.msg import Detection2DArray
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import PointStamped, PoseStamped, Point
from std_msgs.msg import Bool
from cv_bridge import CvBridge
import numpy as np
import tf2_ros
from tf2_geometry_msgs import do_transform_point

class HSRSpatialLocalizer(Node):
    def __init__(self):
        super().__init__('hsr_spatial_localizer')
        self.bridge = CvBridge()
        self.latest_depth = None
        self.latest_scan = None
        
        # Camera Intrinsics
        self.fx, self.fy = 525.0, 525.0
        self.cx, self.cy = 320.0, 240.0

        # TF2 Setup
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Subscriptions
        self.create_subscription(Image, '/rgb_PS1080_PrimeSense/depth_registered/image_raw', self.depth_cb, 10)
        self.create_subscription(Detection2DArray, '/detected_objects', self.detection_cb, 10)
        self.create_subscription(LaserScan, '/hsrb/base_scan', self.scan_cb, 10)
        self.create_subscription(Bool, '/door_lock_trigger', self.lock_cb, 10)

        # World Model Persistence
        self.world_pub = self.create_publisher(MarkerArray, '/vision/world_model', 10)
        self.registry = {}  
        self.door_id_counter = 0
        self.button_id_counter = 0
        self.person_id_counter = 0
        
        self.door_locked = False
        self.ttl_static = 20.0  
        self.ttl_person = 1.5   
        
        # Stability & Tracking Params
        self.confirmed_objects = {} 
        self.stability_threshold = 5
        self.dist_threshold = 0.8 
        self.alpha = 0.3 

        # 10Hz Heartbeat Timer
        self.timer = self.create_timer(0.1, self.timer_callback)

    # --- Callbacks ---

    def lock_cb(self, msg):
        self.door_locked = msg.data
        self.get_logger().info(f"World State Locked: {self.door_locked}")

    def scan_cb(self, msg):
        self.latest_scan = msg

    def depth_cb(self, msg):
        cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        if cv_img.dtype == np.uint16:
            self.latest_depth = cv_img.astype(np.float32) / 1000.0
        else:
            self.latest_depth = cv_img.astype(np.float32)

    # --- Core Logic ---

    def timer_callback(self):
        """Heartbeat: Publishes everything in the registry with current clock time."""
        now = self.get_clock().now().to_msg()
        master_array = MarkerArray()
        to_delete = []

        for obj_key, data in self.registry.items():
            ttl = self.ttl_person if "person" in obj_key else self.ttl_static
            if not self.door_locked and (now.sec - data['last_seen']) > ttl:
                to_delete.append(obj_key)
                continue
            
            # Update markers with latest header timestamp to prevent RVIZ flickering
            for m in data['markers']:
                m.header.stamp = now
            master_array.markers.extend(data['markers'])

        for key in to_delete:
            del self.registry[key]
        if master_array.markers:
            self.world_pub.publish(master_array)

    def detection_cb(self, msg):
        if self.latest_depth is None: return
        # Capture the sensor "Head" timestamp
        sensor_stamp = msg.header.stamp

        for det in msg.detections:
            label = det.results[0].hypothesis.class_id.lower()
            u, v = int(det.bbox.center.position.x), int(det.bbox.center.position.y)
            z = self.get_stable_depth(u, v)
            if z is None: continue

            # Transform to MAP frame using the exact sensor stamp
            x_c = (u - self.cx) * z / self.fx
            y_c = (v - self.cy) * z / self.fy
            p_map = self.transform_point(x_c, y_c, z, msg.header.frame_id, sensor_stamp)
            if p_map is None: continue

            if "button" in label or "door" in label:
                self.process_static_object(p_map, label, det, msg.header.frame_id, sensor_stamp)
            elif "person" in label:
                self.process_moving_person(p_map, sensor_stamp)

    # --- Processing Engines ---

    def process_static_object(self, p, label, det, camera_frame, stamp):
        best_id = self.find_in_registry(p, label)
        stab_key = best_id if best_id else f"new_{label}_{p.x:.1f}"
        if stab_key not in self.confirmed_objects:
            self.confirmed_objects[stab_key] = 0
        self.confirmed_objects[stab_key] += 1
        
        if self.confirmed_objects[stab_key] >= self.stability_threshold:
            if "button" in label:
                self.update_button_registry(p, best_id, stamp)
            elif "door" in label and not self.door_locked:
                self.update_door_registry(det, camera_frame, stamp, best_id, p)

    def process_moving_person(self, p, stamp):
        person_key = self.find_in_registry(p, "person")
        now_wall = stamp.sec + stamp.nanosec * 1e-9
        
        if person_key is None:
            person_key = f"person_{self.person_id_counter}"
            self.person_id_counter += 1
            smooth_pos = [p.x, p.y, p.z]
            velocity = [0.0, 0.0]
        else:
            data = self.registry[person_key]
            dt = now_wall - data['wall_t']
            smooth_x = (self.alpha * p.x) + ((1.0 - self.alpha) * data['pos'][0])
            smooth_y = (self.alpha * p.y) + ((1.0 - self.alpha) * data['pos'][1])
            smooth_pos = [smooth_x, smooth_y, p.z]

            if dt > 0:
                vx = (smooth_x - data['pos'][0]) / dt
                vy = (smooth_y - data['pos'][1]) / dt
                prev_vel = data.get('vel', [0.0, 0.0])
                velocity = [(self.alpha * vx) + ((1.0 - self.alpha) * prev_vel[0]),
                            (self.alpha * vy) + ((1.0 - self.alpha) * prev_vel[1])]
            else:
                velocity = data.get('vel', [0.0, 0.0])

        self.registry[person_key] = {
            'pos': smooth_pos,
            'vel': velocity,
            'last_seen': stamp.sec,
            'wall_t': now_wall,
            'markers': self.create_person_trail_markers(smooth_pos, velocity, person_key, stamp)
        }

    # --- Registry Helpers ---

    def find_in_registry(self, p, label):
        for key, data in self.registry.items():
            if label in key:
                dist = np.linalg.norm(np.array([p.x, p.y]) - np.array(data['pos'][:2]))
                if dist < self.dist_threshold:
                    return key
        return None

    def update_button_registry(self, p, existing_key, stamp):
        bid = existing_key if existing_key else f"button_{self.button_id_counter}"
        if not existing_key: self.button_id_counter += 1
        
        m = Marker()
        m.header.frame_id, m.header.stamp = "map", stamp
        m.ns, m.id = "buttons", int(bid.split('_')[1])
        m.type = Marker.SPHERE
        m.pose.position = p
        m.scale.x = m.scale.y = m.scale.z = 0.12
        m.color.r, m.color.g, m.color.b, m.color.a = 1.0, 1.0, 0.0, 1.0
        
        self.registry[bid] = {
            'pos': [p.x, p.y, p.z],
            'last_seen': stamp.sec,
            'markers': [m]
        }

    def update_door_registry(self, det, frame, stamp, existing_key, map_center):
        u_l = det.bbox.center.position.x - (det.bbox.size_x / 2.0)
        u_r = det.bbox.center.position.x + (det.bbox.size_x / 2.0)
        a_l, a_r = -np.arctan2((u_l-self.cx), self.fx), -np.arctan2((u_r-self.cx), self.fx)
        
        map_l = self.project_and_transform(self.get_scan_dist(a_l), a_l, frame, stamp)
        map_r = self.project_and_transform(self.get_scan_dist(a_r), a_r, frame, stamp)
        
        if not map_l or not map_r: return

        did_str = existing_key if existing_key else f"door_{self.door_id_counter}"
        if not existing_key: self.door_id_counter += 1
        did = int(did_str.split('_')[1])

        self.registry[did_str] = {
            'pos': [map_center.x, map_center.y, map_center.z],
            'last_seen': stamp.sec,
            'markers': self.create_door_marker_set(map_l, map_r, did, stamp)
        }

    # --- Visual Generators ---

    def create_door_marker_set(self, p_l, p_r, did, stamp):
        markers = []
        # Pillars (Tall Red Cylinders)
        for i, pt in enumerate([p_l, p_r]):
            m = Marker()
            m.header.frame_id, m.header.stamp = "map", stamp
            m.ns, m.id = f"door_{did}_pillars", i
            m.type = Marker.CYLINDER
            m.pose.position = pt
            m.pose.position.z = 1.0
            m.scale.x = m.scale.y = 0.06
            m.scale.z = 2.0
            m.color.r, m.color.g, m.color.b, m.color.a = 1.0, 0.0, 0.0, 0.8
            markers.append(m)

        # Gap Disc (Floor Green Disc)
        mx, my = (p_l.x + p_r.x)/2.0, (p_l.y + p_r.y)/2.0
        gap = Marker()
        gap.header.frame_id, gap.header.stamp = "map", stamp
        gap.ns, gap.id = f"door_{did}_gap", 0
        gap.type = Marker.CYLINDER
        gap.pose.position.x, gap.pose.position.y, gap.pose.position.z = mx, my, 0.01
        gap.scale.x = gap.scale.y = 0.6
        gap.scale.z = 0.02
        gap.color.g, gap.color.a = 1.0, 0.7
        markers.append(gap)

        # Exit Normal (Blue Arrow)
        dx, dy = p_r.x - p_l.x, p_r.y - p_l.y
        dist = np.sqrt(dx**2 + dy**2)
        nx, ny = -dy/dist, dx/dist
        exit_m = Marker()
        exit_m.header.frame_id, exit_m.header.stamp = "map", stamp
        exit_m.ns, exit_m.id = f"door_{did}_exit", 0
        exit_m.type = Marker.ARROW
        exit_m.pose.position.x, exit_m.pose.position.y = mx + nx*1.5, my + ny*1.5
        yaw = np.atan2(ny, nx)
        exit_m.pose.orientation.z, exit_m.pose.orientation.w = np.sin(yaw/2.0), np.cos(yaw/2.0)
        exit_m.scale.x, exit_m.scale.y, exit_m.scale.z = 0.8, 0.1, 0.1
        exit_m.color.b, exit_m.color.a = 1.0, 1.0
        markers.append(exit_m)
        return markers

    def create_person_trail_markers(self, pos, vel, key, stamp):
        markers = []
        num_steps = 5 
        for step in range(num_steps):
            m = Marker()
            m.header.frame_id, m.header.stamp = "map", stamp
            m.ns, m.id = key, step
            m.type = Marker.CYLINDER
            t_offset = step * 0.2
            m.pose.position.x = pos[0] + (vel[0] * t_offset)
            m.pose.position.y = pos[1] + (vel[1] * t_offset)
            m.pose.position.z = 0.01
            v_scale = 1.0 - (step / (num_steps + 1))
            m.scale.x = m.scale.y = 0.6 * v_scale
            m.scale.z = 0.02
            m.color.r, m.color.g, m.color.b, m.color.a = 0.2, 0.4, 1.0, max(0.1, 0.6 - (step * 0.1))
            markers.append(m)
        return markers

    # --- Math Helpers ---

    def get_scan_dist(self, target_angle):
        if self.latest_scan is None:
            return 1.5
        inc = self.latest_scan.angle_increment
        if inc <= 0.0: return 2.0
        n = len(self.latest_scan.ranges)
        idx = int((target_angle - self.latest_scan.angle_min) / inc)
        idx = max(0, min(idx, n - 1))
        # Median window (7 beams for stability)
        window = 3
        start, end = max(0, idx - window), min(n - 1, idx + window)
        beams = self.latest_scan.ranges[start:end+1]
        valid = [d for d in beams if np.isfinite(d) and d > 0.1]
        return float(np.median(valid)) if valid else 2.0

    def get_stable_depth(self, u, v):
        h, w = self.latest_depth.shape[:2]
        win = 2
        patch = self.latest_depth[max(0,v-win):min(h,v+win+1), max(0,u-win):min(w,u+win+1)]
        valid = patch[np.isfinite(patch) & (patch > 0.1)]
        return float(np.median(valid)) if len(valid) > 0 else None

    def transform_point(self, x, y, z, from_frame, stamp):
        try:
            p = PointStamped()
            p.header.frame_id, p.header.stamp = from_frame, stamp
            p.point.x, p.point.y, p.point.z = x, y, z
            # Coordinate lookup in map frame using sensor timestamp
            return do_transform_point(p, self.tf_buffer.lookup_transform('map', from_frame, stamp, rclpy.duration.Duration(seconds=0.1))).point
        except: return None

    def project_and_transform(self, d, a, frame, stamp):
        return self.transform_point(d*np.cos(a), d*np.sin(a), 0.0, frame, stamp)

def main(args=None):
    rclpy.init(args=args)
    node = HSRSpatialLocalizer()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()