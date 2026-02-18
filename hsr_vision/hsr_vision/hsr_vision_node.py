import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose
from cv_bridge import CvBridge
from ultralytics import YOLO
from ament_index_python.packages import get_package_share_directory
import os
import cv2
import numpy as np
import message_filters # Requires: sudo apt install ros-$ROS_DISTRO-message-filters

class HSRVisionNode(Node):
    def __init__(self):
        super().__init__('hsr_vision_node')
        
        # 1. Setup Model & Bridge
        package_share_dir = get_package_share_directory('hsr_vision')
        model_path = os.path.join(package_share_dir, 'models', 'cardboard.pt')
        self.model = YOLO(model_path) 
        self.bridge = CvBridge()
        self.class_names = self.model.names 
        
        # 2. Camera Intrinsics (HSR PrimeSense standard approx)
        # For better accuracy, subscribe to /rgb_PS1080_PrimeSense/rgb/camera_info
        self.fx, self.fy = 525.0, 525.0
        self.cx, self.cy = 320.0, 240.0
        
        # 3. Synchronized Subscriptions
        self.rgb_sub = message_filters.Subscriber(self, Image, '/rgb_PS1080_PrimeSense/rgb/image_rect_color')
        self.depth_sub = message_filters.Subscriber(self, Image, '/rgb_PS1080_PrimeSense/depth_registered/image_raw')
        
        # Syncs topics within 0.1s of each other
        self.ts = message_filters.ApproximateTimeSynchronizer([self.rgb_sub, self.depth_sub], 10, 0.1)
        self.ts.registerCallback(self.synchronized_callback)
        
        self.detection_pub = self.create_publisher(Detection2DArray, '/detected_buttons', 10)
        self.debug_pub = self.create_publisher(Image, '/detected_image_debug', 10)
        
        self.frame_counter = 0
        self.process_every_n_frames = 10 

    def synchronized_callback(self, rgb_msg, depth_msg):
        self.frame_counter += 1
        if self.frame_counter % self.process_every_n_frames != 0:
            return

        cv_image = self.bridge.imgmsg_to_cv2(rgb_msg, "bgr8")
        depth_image = self.bridge.imgmsg_to_cv2(depth_msg, "32FC1") # Depth in meters
        
        results = self.model(cv_image, conf=0.25, verbose=False)
        detection_array = Detection2DArray()
        detection_array.header = rgb_msg.header 
        
        found_any = False
        for result in results:
            for box in result.boxes:
                found_any = True
                
                # 1. Get pixel center (u, v)
                u, v = int(box.xywh[0][0]), int(box.xywh[0][1])
                
                # 2. Get Depth Z (use median of 5x5 area to avoid noise)
                z = np.nanmedian(depth_image[v-2:v+3, u-2:u+3])
                
                if not np.isnan(z) and z > 0:
                    # 3. Calculate 3D Coordinates
                    x = (u - self.cx) * z / self.fx
                    y = (v - self.cy) * z / self.fy
                    self.get_logger().info(f"FOUND: {self.class_names[int(box.cls[0])]} at X:{x:.2f} Y:{y:.2f} Z:{z:.2f}")

                # Create ROS Message and draw debug info
                det = self.create_detection_msg(box)
                detection_array.detections.append(det)
                
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                cv2.rectangle(cv_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(cv_image, f"Z: {z:.2f}m", (x1, y1-10), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        if found_any:
            self.detection_pub.publish(detection_array)
        
        debug_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding="bgr8")
        debug_msg.header = rgb_msg.header
        self.debug_pub.publish(debug_msg)

    def create_detection_msg(self, box):
        det = Detection2D()
        det.bbox.center.position.x = float(box.xywh[0][0])
        det.bbox.center.position.y = float(box.xywh[0][1])
        det.bbox.size_x = float(box.xywh[0][2])
        det.bbox.size_y = float(box.xywh[0][3])
        hyp = ObjectHypothesisWithPose()
        hyp.hypothesis.class_id = str(int(box.cls[0]))
        hyp.hypothesis.score = float(box.conf[0])
        det.results.append(hyp)
        return det

def main(args=None):
    rclpy.init(args=args)
    node = HSRVisionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()