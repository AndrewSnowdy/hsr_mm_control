import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose
from cv_bridge import CvBridge
from ultralytics import YOLO
from ament_index_python.packages import get_package_share_directory
import os

class HSRVisionNode(Node):
    def __init__(self):
        super().__init__('hsr_vision_node')
        
        # 1. Dynamic Pathing
        package_share_dir = get_package_share_directory('hsr_vision')
        model_path = os.path.join(package_share_dir, 'models', 'best.pt')
        
        # 2. Model Initialization
        self.model = YOLO(model_path) 
        self.bridge = CvBridge()
        
        # Use the model's internal names to guarantee consistency
        self.class_names = self.model.names 
        self.get_logger().info(f"Model loaded with classes: {self.class_names}")
        
        # 3. Throttling Configuration
        self.frame_counter = 0
        self.process_every_n_frames = 10  # Reduces CPU load by skipping frames
        
        self.detection_pub = self.create_publisher(Detection2DArray, '/detected_buttons', 10)
        
        self.subscription = self.create_subscription(
            Image,
            '/rgb_PS1080_PrimeSense/rgb/image_rect_color',
            self.image_callback,
            10)
        
        self.get_logger().info("HSR Vision Node initialized and throttled to 5 FPS.")

    def image_callback(self, msg):
        # Throttle logic: Only process Nth frame to save CPU
        self.frame_counter += 1
        if self.frame_counter % self.process_every_n_frames != 0:
            return

        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        
        # Optimized Inference
        results = self.model(cv_image, conf=0.25, verbose=False)
        
        detection_array = Detection2DArray()
        detection_array.header = msg.header 
        
        found_any = False
        for result in results:
            for box in result.boxes:
                det = self.create_detection_msg(box)
                detection_array.detections.append(det)
                
                label = self.class_names.get(int(box.cls[0]), "unknown")
                # Log only on detection to keep terminal clean
                self.get_logger().info(f"🎯 FOUND: {label} (Conf: {float(box.conf[0]):.2f})")
                found_any = True

        if found_any:
            self.detection_pub.publish(detection_array)

    def create_detection_msg(self, box):
        """Helper to keep the callback clean."""
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