import rclpy
from rclpy.node import Node
from vision_msgs.msg import Detection2DArray
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped, PointStamped
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
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)

        self.confirmed_objects = {}
        self.stability_threshold = 5  # Must see it 5 times
        self.distance_threshold = 0.1

    def depth_cb(self, msg):
        # Convert the message to an OpenCV image first
        cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")

        # Now handle the scaling based on the data type
        if cv_img.dtype == np.uint16:
            # Millimeters to Meters
            self.latest_depth = cv_img.astype(np.float32) / 1000.0
        else:
            # Already in Meters (float32)
            self.latest_depth = cv_img.astype(np.float32)

    def detection_cb(self, msg):
        if self.latest_depth is None: return
        
        # Grab the ORIGINAL camera timestamp from the Jetson message header
        detection_time = msg.header.stamp

        for det in msg.detections:
            u = int(det.bbox.center.position.x)
            v = int(det.bbox.center.position.y)
            label = det.results[0].hypothesis.class_id

            # --- ROBUST DEPTH SAMPLING ---
            h, w = self.latest_depth.shape[:2]
            win = 2 
            v_min, v_max = max(0, v - win), min(h, v + win + 1)
            u_min, u_max = max(0, u - win), min(w, u + win + 1)
            depth_patch = self.latest_depth[v_min:v_max, u_min:u_max]
            valid_depths = depth_patch[np.isfinite(depth_patch) & (depth_patch > 0)]
            
            if len(valid_depths) == 0: continue
            z = float(np.median(valid_depths))

            # Project to 3D (Camera Frame)
            x_c = (u - self.cx) * z / self.fx
            y_c = (v - self.cy) * z / self.fy
            z_c = z

            # --- UPDATED STABILITY CHECK ---
            # Using .lower() ensures 'Push_Button' or 'prox_button' are both caught
            if "button" in label.lower():
                self.track_object_stability(x_c, y_c, z_c, label, msg.header.frame_id, detection_time)
            else:
                self.transform_to_robot_frame(x_c, y_c, z_c, label, msg.header.frame_id, detection_time)

    def track_object_stability(self, x, y, z, label, frame_id, timestamp):
        if label not in self.confirmed_objects:
            self.confirmed_objects[label] = {'pos': [x, y, z], 'count': 1}
            return

        prev = self.confirmed_objects[label]
        dist = np.linalg.norm(np.array([x, y, z]) - np.array(prev['pos']))

        if dist < self.distance_threshold:
            prev['count'] += 1
            prev['pos'] = (np.array(prev['pos']) * 0.8 + np.array([x, y, z]) * 0.2).tolist()
            
            if prev['count'] == self.stability_threshold:
                self.get_logger().info(f"STABLE {label} confirmed!")
                # Pass the timestamp through to the transform
                self.transform_to_robot_frame(prev['pos'][0], prev['pos'][1], prev['pos'][2], label, frame_id, timestamp)
        else:
            self.confirmed_objects[label] = {'pos': [x, y, z], 'count': 1}

    def transform_to_robot_frame(self, x, y, z, label, camera_frame, timestamp):
        try:
            point_camera = PointStamped()
            point_camera.header.frame_id = camera_frame
            point_camera.header.stamp = timestamp # <--- Use the original capture time
            
            point_camera.point.x = float(x)
            point_camera.point.y = float(y)
            point_camera.point.z = float(z)

            # Look up transform exactly at the moment the image was captured
            transform = self.tf_buffer.lookup_transform(
                'base_link', 
                camera_frame, 
                timestamp, 
                timeout=rclpy.duration.Duration(seconds=0.1)
            )
            
            point_robot = do_transform_point(point_camera, transform)

            rx, ry, rz = float(point_robot.point.x), float(point_robot.point.y), float(point_robot.point.z)
            self.get_logger().info(f"{label} in BASE_LINK: x={rx:.2f}, y={ry:.2f}, z={rz:.2f}")
            
            if "button" in label.lower():
                self.publish_goal(point_robot)

        except Exception as e:
            self.get_logger().warn(f"TF Transform failed for {label}: {e}")

    def publish_goal(self, point_robot):
        goal = PoseStamped()
        goal.header = point_robot.header
        goal.pose.position = point_robot.point
        # Set a neutral orientation
        goal.pose.orientation.w = 1.0
        self.goal_pub.publish(goal)


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