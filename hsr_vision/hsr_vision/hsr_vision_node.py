import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from cv_bridge import CvBridge
from ultralytics import YOLO
import os

class HSRVisionNode(Node):
    def __init__(self):
        super().__init__('hsr_vision_node')
        
        # 1. Improved Pathing: Look in the 'models' folder of your package
        # Update this to your actual best.pt location in hsr_ros2_ws
        model_path = os.path.expanduser('~/hsr_ros2_ws/src/hsr_vision/models/best.pt')
        self.model = YOLO(model_path) 
        self.bridge = CvBridge()
        
        # Map IDs to names based on your hsr_run_2 results
        self.class_names = {0: 'dual_button', 1: 'prox_button', 2: 'single_button'}
        
        self.coord_pub = self.create_publisher(Point, '/detected_button_coords', 10)
        
        self.subscription = self.create_subscription(
            Image,
            '/hsr/head_rgbd_sensor/rgb/image_raw',
            self.image_callback,
            10)
        
        self.get_logger().info("HSR Multi-Class Vision Node Started.")

    def image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        
        # Run Inference with your new 3-class model
        results = self.model(cv_image, conf=0.6, stream=True)
        
        for result in results:
            for box in result.boxes:
                x_center = float(box.xywh[0][0])
                y_center = float(box.xywh[0][1])
                class_id = int(box.cls[0])
                label = self.class_names.get(class_id, "unknown")
                
                btn_point = Point()
                btn_point.x = x_center
                btn_point.y = y_center
                btn_point.z = float(class_id) # Z stores the class ID for the controller
                
                self.coord_pub.publish(btn_point)
                self.get_logger().info(f"Detected {label} at: x={x_center:.1f}, y={y_center:.1f}")

def main(args=None):
    rclpy.init(args=args)
    node = HSRVisionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()