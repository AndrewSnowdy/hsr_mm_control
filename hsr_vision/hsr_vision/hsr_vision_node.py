import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from cv_bridge import CvBridge
from ultralytics import YOLO

class HSRVisionNode(Node):
    def __init__(self):
        super().__init__('hsr_vision_node')
        
        # 1. Load the model (Provide the path to your best.pt)
        # Use device='cpu' if the robot doesn't have a GPU
        self.model = YOLO("best.pt") 
        self.bridge = CvBridge()
        
        # 2. Publisher for button coordinates
        self.coord_pub = self.create_publisher(Point, '/detected_button_coords', 10)
        
        # 3. Subscriber for HSR Head Camera
        self.subscription = self.create_subscription(
            Image,
            '/hsr/head_rgbd_sensor/rgb/image_raw',
            self.image_callback,
            10)
        
        self.get_logger().info("HSR Vision Node has started and is looking for ADA buttons...")

    def image_callback(self, msg):
        # Convert ROS image to OpenCV
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        
        # 4. Run Inference
        # 'stream=True' is more memory-efficient for continuous video
        results = self.model(cv_image, conf=0.6, stream=True)
        
        for result in results:
            for box in result.boxes:
                # Get center coordinates (x, y)
                x_center = float(box.xywh[0][0])
                y_center = float(box.xywh[0][1])
                
                # Publish the Point (z can be used for the class ID or depth)
                btn_point = Point()
                btn_point.x = x_center
                btn_point.y = y_center
                btn_point.z = float(box.cls[0]) # Class ID (prox, dual, or single)
                
                self.coord_pub.publish(btn_point)
                self.get_logger().info(f"Published Button: x={x_center}, y={y_center}")

def main(args=None):
    rclpy.init(args=args)
    node = HSRVisionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()