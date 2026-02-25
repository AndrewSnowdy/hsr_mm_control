import rclpy
from rclpy.node import Node
from vision_msgs.msg import Detection2DArray
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge
import numpy as np

class HSRSpatialLocalizer(Node):
    def __init__(self):
        super().__init__('hsr_spatial_localizer')
        self.bridge = CvBridge()
        self.latest_depth = None
        
        # Camera Intrinsics (from your previous node)
        self.fx, self.fy = 525.0, 525.0
        self.cx, self.cy = 320.0, 240.0

        # Subscriptions
        self.create_subscription(Image, '/rgb_PS1080_PrimeSense/depth_registered/image_raw', self.depth_cb, 10)
        self.create_subscription(Detection2DArray, '/detected_objects', self.detection_cb, 10)
        
        # Publisher for the navigation system
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)

    def depth_cb(self, msg):
        self.latest_depth = self.bridge.imgmsg_to_cv2(msg, '32FC1')

    def detection_cb(self, msg):
        if self.latest_depth is None: return

        for det in msg.detections:
            # 1. Get 2D pixel center
            u = int(det.bbox.center.position.x)
            v = int(det.bbox.center.position.y)
            label = det.results[0].hypothesis.class_id

            # 2. Sample Depth (Z)
            z = self.latest_depth[v, u]
            if np.isnan(z) or z <= 0: continue

            # 3. Project to 3D (Camera Frame)
            x_cam = (u - self.cx) * z / self.fx
            y_cam = (v - self.cy) * z / self.fy
            z_cam = z

            self.get_logger().info(f"Found {label} at 3D: {x_cam:.2f}, {y_cam:.2f}, {z_cam:.2f}")

            # 4. Trigger Movement logic here (e.g., publish a goal)
            if label == "button":
                self.publish_goal(x_cam, y_cam, z_cam)