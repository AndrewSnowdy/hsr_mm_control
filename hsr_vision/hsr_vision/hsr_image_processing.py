import rclpy
from sensor_msgs.msg import LaserScan
from rclpy.node import Node
from vision_msgs.msg import Detection2DArray
from visualization_msgs.msg import Marker, MarkerArray
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped, PointStamped, Point
from cv_bridge import CvBridge
import numpy as np
import tf2_ros
from tf2_geometry_msgs import do_transform_point

class HSRSpatialLocalizer(Node):
    def __init__(self):
        super().__init__('hsr_spatial_localizer')
        self.bridge = CvBridge()
        self.latest_depth = None
        
        # Camera Intrinsics
        self.fx, self.fy = 525.0, 525.0
        self.cx, self.cy = 320.0, 240.0

        # TF2 Setup
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.create_subscription(Image, '/rgb_PS1080_PrimeSense/depth_registered/image_raw', self.depth_cb, 10)
        self.create_subscription(Detection2DArray, '/detected_objects', self.detection_cb, 10)
        self.latest_scan = None
        self.create_subscription(LaserScan, '/hsrb/base_scan', self.scan_cb, 10)

        self.door_locked = False
        self.create_subscription(Bool, '/door_lock_trigger', self.lock_cb, 10)

        self.door_anchors = None
        
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        # We use MarkerArray for all visualizations now
        self.marker_pub = self.create_publisher(MarkerArray, '/visualization_marker', 10)  

        self.confirmed_objects = {}
        self.stability_threshold = 5 
        self.distance_threshold = 0.1

        # --- Tracking State ---
        self.active_people = {} 
        self.person_id_counter = 0
        self.max_disappearance_time = 1.0
        self.alpha = 0.3 

    def lock_cb(self, msg):
        self.door_locked = msg.data
        self.get_logger().info(f"Door Lock Status: {self.door_locked}")
        
    def scan_cb(self, msg):
        self.latest_scan = msg

    def depth_cb(self, msg):
        cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        if cv_img.dtype == np.uint16:
            self.latest_depth = cv_img.astype(np.float32) / 1000.0
        else:
            self.latest_depth = cv_img.astype(np.float32)

    def detection_cb(self, msg):
        if self.latest_depth is None: return
        detection_time = msg.header.stamp

        for det in msg.detections:
            u, v = int(det.bbox.center.position.x), int(det.bbox.center.position.y)
            label = det.results[0].hypothesis.class_id.lower()

            # --- 1. Get 3D Point from Depth Camera ---
            h, w = self.latest_depth.shape[:2]
            win = 2 
            depth_patch = self.latest_depth[max(0,v-win):min(h,v+win+1), max(0,u-win):min(w,u+win+1)]
            valid_depths = depth_patch[np.isfinite(depth_patch) & (depth_patch > 0)]
            
            if len(valid_depths) == 0: continue
            z = float(np.median(valid_depths))

            x_c = (u - self.cx) * z / self.fx
            y_c = (v - self.cy) * z / self.fy
            z_c = z

            # --- 2. Routing Logic (FIXED INDENTATION) ---
            # Group static items (door/buttons) for stability check
            if "button" in label or "door" in label:
                self.track_object_stability(x_c, y_c, z_c, label, msg.header.frame_id, detection_time, det)
            elif "person" in label:
                # Persons move; transform directly to robot frame without averaging
                self.transform_to_robot_frame(x_c, y_c, z_c, "person", msg.header.frame_id, detection_time)

    def track_object_stability(self, x, y, z, label, frame_id, timestamp, det=None):
        if label not in self.confirmed_objects:
            self.confirmed_objects[label] = {'pos': [x, y, z], 'count': 1}
            return

        prev = self.confirmed_objects[label]
        dist = np.linalg.norm(np.array([x, y, z]) - np.array(prev['pos']))
        
        # If the new detection is close to our 'stable' point
        if dist < self.distance_threshold:
            prev['count'] += 1
            # Smoothly update the internal position (80% old, 20% new)
            prev['pos'] = (np.array(prev['pos']) * 0.8 + np.array([x, y, z]) * 0.2).tolist()
            
            # CHANGE: Use '>=' instead of '==' 
            # This means after frame 5, EVERY new frame updates the goalposts
            if prev['count'] >= self.stability_threshold:
                if "button" in label:
                    self.transform_to_robot_frame(prev['pos'][0], prev['pos'][1], prev['pos'][2], label, frame_id, timestamp)
                elif "door" in label: # and self.latest_scan is not None:
                    # Goalposts will now 'track' the door as you approach
                    if not self.door_locked:
                        self.process_door_goalposts(det, frame_id, timestamp)
        else:
            # If the object 'jumps' (drift or new object), reset the count
            # but don't reset immediately—maybe it was just one bad frame?
            # For now, a hard reset is safest for your mission.
            self.confirmed_objects[label] = {'pos': [x, y, z], 'count': 1}


    def update_person_tracking(self, rx, ry, timestamp):
        """Euclidean matching to handle multiple people and velocity."""
        now = timestamp.sec + timestamp.nanosec * 1e-9
        best_id = None
        min_dist = 0.8 # Threshold for matching the same person

        for p_id, data in self.active_people.items():
            dist = np.linalg.norm(np.array([rx, ry]) - np.array(data['pos']))
            if dist < min_dist:
                min_dist = dist
                best_id = p_id

        if best_id is not None:
            data = self.active_people[best_id]
            dt = now - data['last_seen']
            
            # --- APPLY ALPHA FILTER (Smoothing) ---
            # Instead of jumping to [rx, ry], we move toward it slowly
            smooth_x = (self.alpha * rx) + ((1.0 - self.alpha) * data['pos'][0])
            smooth_y = (self.alpha * ry) + ((1.0 - self.alpha) * data['pos'][1])
            
            if dt > 0:
                # Calculate velocity based on SMOOTHED positions for a stable trail
                vx = (smooth_x - data['pos'][0]) / dt
                vy = (smooth_y - data['pos'][1]) / dt
                
                # Smooth the velocity too (it's often the jumpiest part)
                prev_vx, prev_vy = data.get('vel', [0.0, 0.0])
                data['vel'] = [
                    (self.alpha * vx) + ((1.0 - self.alpha) * prev_vx),
                    (self.alpha * vy) + ((1.0 - self.alpha) * prev_vy)
                ]

                speed = np.linalg.norm(data['vel'])
                if speed < 0.1: # If moving slower than 10cm/s, assume they are still
                    data['vel'] = [0.0, 0.0]

            data['pos'] = [smooth_x, smooth_y]
            data['last_seen'] = now
        else:
            best_id = self.person_id_counter
            self.active_people[best_id] = {'pos': [rx, ry], 'last_seen': now, 'vel': [0.0, 0.0]}
            self.person_id_counter += 1

        # Clean up lost tracks
        self.active_people = {i: d for i, d in self.active_people.items() 
                             if now - d['last_seen'] < self.max_disappearance_time}
        
        return best_id, self.active_people[best_id]['vel']


    def transform_to_robot_frame(self, x, y, z, label, camera_frame, timestamp):
        try:
            point_camera = PointStamped()
            point_camera.header.frame_id = camera_frame
            point_camera.header.stamp = timestamp
            point_camera.point.x, point_camera.point.y, point_camera.point.z = float(x), float(y), float(z)

            transform = self.tf_buffer.lookup_transform('base_link', camera_frame, timestamp, 
                                                        timeout=rclpy.duration.Duration(seconds=0.1))
            point_robot = do_transform_point(point_camera, transform)
            rx, ry = float(point_robot.point.x), float(point_robot.point.y)

            if "button" in label:
                self.publish_goal(point_robot)
            elif "door" in label:
                # Orange disc for doors
                self.publish_door_marker(rx, ry)

        except Exception as e:
            pass 


    def publish_door_marker(self, x, y):
        marker_array = MarkerArray()
        marker = Marker()
        marker.header.frame_id = "base_link"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "doors"
        marker.id = 0
        marker.type = Marker.CYLINDER
        marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = x, y, 0.0
        marker.scale.x, marker.scale.y, marker.scale.z = 0.8, 0.8, 0.02
        # Orange Color for doors
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = 1.0, 0.5, 0.0, 0.5
        marker.lifetime = rclpy.duration.Duration(seconds=0.5).to_msg()
        marker_array.markers.append(marker)
        self.marker_pub.publish(marker_array)


    def publish_person_trails(self, x, y, label, person_id, velocity=[0.0, 0.0]):
        marker_array = MarkerArray()
        speed = np.linalg.norm(velocity)
        num_prediction_steps = 5 
        base_color = (0.2, 0.4, 1.0) # Blue
        unique_ns = f"person_{person_id}"

        for step in range(num_prediction_steps):
            marker = Marker()
            marker.header.frame_id = "base_link"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = unique_ns
            marker.id = step
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD
            
            vanishing_scale = 1.0 - (step / (num_prediction_steps + 2)) 
            vanishing_alpha = 0.6 - (step * 0.1) 
            
            t_offset = step * 0.2
            px = x + (velocity[0] * t_offset)
            py = y + (velocity[1] * t_offset)
            
            marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = float(px), float(py), 0.01
            marker.pose.orientation.w = 1.0
            
            radius = 0.3 * vanishing_scale
            marker.scale.x, marker.scale.y, marker.scale.z = radius * 2, radius * 2, 0.02
            
            marker.color.r, marker.color.g, marker.color.b = base_color
            marker.color.a = float(max(0.1, vanishing_alpha)) # Ensure it doesn't go below 0.1
            marker.lifetime = rclpy.duration.Duration(seconds=0.3).to_msg()
            marker_array.markers.append(marker)
            
        self.marker_pub.publish(marker_array)


    def publish_goal(self, point_robot):
        goal = PoseStamped()
        goal.header = point_robot.header
        goal.pose.position = point_robot.point
        goal.pose.orientation.w = 1.0
        self.goal_pub.publish(goal)


    def process_door_goalposts(self, det, camera_frame, timestamp):
        # 1. Get Image Edges (Goalpost pixels)
        u_l = det.bbox.center.position.x - (det.bbox.size_x / 2.0)
        u_r = det.bbox.center.position.x + (det.bbox.size_x / 2.0)
        
        # 2. Convert Pixels to Angles (Radians)
        angle_l = -np.arctan2((u_l - self.cx), self.fx)
        angle_r = -np.arctan2((u_r - self.cx), self.fx)

        # 3. Get Physical Distance from LiDAR (Glass-Proof)
        dist_l = self.get_scan_dist(angle_l)
        dist_r = self.get_scan_dist(angle_r)

        # 4. Project and Transform to Map Frame
        # We transform to 'map' so if the robot turns, the posts stay in place
        map_l = self.project_and_transform(dist_l, angle_l, camera_frame, timestamp)
        map_r = self.project_and_transform(dist_r, angle_r, camera_frame, timestamp)

        if map_l and map_r:
            self.door_anchors = {'left': map_l, 'right': map_r}
            self.publish_door_visuals(map_l, map_r)


    def get_scan_dist(self, target_angle):
        """Finds the LiDAR distance at a specific relative angle."""
        if self.latest_scan is None: return 1.5
        idx = int((target_angle - self.latest_scan.angle_min) / self.latest_scan.angle_increment)
        idx = max(0, min(idx, len(self.latest_scan.ranges) - 1))
        dist = self.latest_scan.ranges[idx]
        return dist if (np.isfinite(dist) and dist > 0.1) else 2.0


    def project_and_transform(self, dist, angle, from_frame, timestamp):
        """Projects a polar coordinate to 3D and transforms to map frame."""
        try:
            p = PointStamped()
            p.header.frame_id = from_frame
            p.header.stamp = timestamp
            p.point.x = dist * np.cos(angle)
            p.point.y = dist * np.sin(angle)
            p.point.z = 0.0
            
            # Transform to 'map' for global persistence
            transform = self.tf_buffer.lookup_transform('map', from_frame, timestamp, 
                                                        rclpy.duration.Duration(seconds=0.1))
            return do_transform_point(p, transform).point
        except:
            return None


    def publish_door_visuals(self, p_l, p_r):
        marker_array = MarkerArray()
        
        # Two tall Pillars (Red)
        for i, pt in enumerate([p_l, p_r]):
            m = Marker()
            m.header.frame_id = "map"
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = "door_frame"
            m.id = i
            m.type = Marker.CYLINDER
            m.pose.position = pt
            m.pose.position.z = 1.0 # 2m tall, centered at 1m
            m.scale.x = m.scale.y = 0.1
            m.scale.z = 2.0
            m.color.r, m.color.g, m.color.b, m.color.a = 1.0, 0.0, 0.0, 0.8
            marker_array.markers.append(m)

        # Safe Passage Center (Green Sphere)
        center_marker = Marker()
        center_marker.header.frame_id = "map"
        center_marker.header.stamp = self.get_clock().now().to_msg()
        center_marker.ns = "safe_passage"
        center_marker.id = 0
        center_marker.type = Marker.SPHERE
        center_marker.pose.position.x = (p_l.x + p_r.x) / 2.0
        center_marker.pose.position.y = (p_l.y + p_r.y) / 2.0
        center_marker.pose.position.z = 0.0
        center_marker.scale.x = center_marker.scale.y = center_marker.scale.z = 0.2
        center_marker.color.r, center_marker.color.g, center_marker.color.b, center_marker.color.a = 0.0, 1.0, 0.0, 1.0
        marker_array.markers.append(center_marker)

        self.marker_pub.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)
    node = HSRSpatialLocalizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()