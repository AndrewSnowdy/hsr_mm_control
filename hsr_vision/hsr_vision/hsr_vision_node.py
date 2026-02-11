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
        
        # INDUSTRY STANDARD: Get the model path dynamically from the 'share' folder
        package_share_dir = get_package_share_directory('hsr_vision')
        model_path = os.path.join(package_share_dir, 'models', 'best.pt')
        
        self.model = YOLO(model_path) 
        self.bridge = CvBridge()
        self.class_names = {0: 'dual_button', 1: 'prox_button', 2: 'single_button'}
        
        # INDUSTRY STANDARD: Use Detection2DArray instead of Point
        self.detection_pub = self.create_publisher(Detection2DArray, '/detected_buttons', 10)
        
        self.subscription = self.create_subscription(
            Image,
            '/rgb_PS1080_PrimeSense/rgb/image_rect_color', # Use the rectified topic we found
            self.image_callback,
            10)
        
        self.get_logger().info("HSR Vision Node initialized using package share directory.")

    def image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        results = self.model(cv_image, conf=0.6, verbose=False)
        
        detection_array = Detection2DArray()
        detection_array.header = msg.header # Keep the timestamp for depth syncing later
        
        for result in results:
            for box in result.boxes:
                # Create standard ROS detection message
                det = Detection2D()
                
                # Bounding Box info
                det.bbox.center.position.x = float(box.xywh[0][0])
                det.bbox.center.position.y = float(box.xywh[0][1])
                det.bbox.size_x = float(box.xywh[0][2])
                det.bbox.size_y = float(box.xywh[0][3])
                
                # Class and Confidence info
                hyp = ObjectHypothesisWithPose()
                hyp.hypothesis.class_id = str(int(box.cls[0]))
                hyp.hypothesis.score = float(box.conf[0])
                det.results.append(hyp)
                
                detection_array.detections.append(det)
                
                label = self.class_names.get(int(box.cls[0]), "unknown")
                self.get_logger().info(f"Detected {label} at [{det.bbox.center.position.x}, {det.bbox.center.position.y}]")

        self.detection_pub.publish(detection_array)

def main(args=None):
    rclpy.init(args=args)
    node = HSRVisionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()