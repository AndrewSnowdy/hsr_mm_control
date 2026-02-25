import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image, CompressedImage
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from ultralytics import YOLO
import os
import cv2
import numpy as np
import time

class HSRDetailedProfileNode(Node):
    def __init__(self):
        super().__init__('hsr_detailed_profile_node')


        package_share_dir = get_package_share_directory('hsr_vision')
        human_path = os.path.join(package_share_dir, 'models', 'human.engine')
        button_path = os.path.join(package_share_dir, 'models', 'door_button.engine')
        
        self.get_logger().info(f"Loading model from: {human_path}")
        self.human_model = YOLO(human_path, task='detect') 
        self.get_logger().info(f"HUMAN Model Classes: {self.human_model.names}")
        
        self.get_logger().info(f"Loading Button Model: {button_path}")
        self.button_model = YOLO(button_path, task='detect')
        self.get_logger().info(f"BUTTON Model Classes: {self.button_model.names}")

        self.bridge = CvBridge()
        self.latest_depth = None 

        self.frame_count = 0
        self.button_interval = 1

        self.latest_button = None
        
        # Camera Intrinsics
        self.fx, self.fy = 525.0, 525.0
        self.cx, self.cy = 320.0, 240.0

        fast_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.rgb_sub = self.create_subscription(
            CompressedImage,
            '/rgb_PS1080_PrimeSense/rgb/image_rect_color/compressed',
            self.rgb_callback,
            fast_qos)

        self.detection_pub = self.create_publisher(Detection2DArray, '/detected_objects', 10)
        self.debug_pub = self.create_publisher(Image, '/people_debug_image', 10)

        self.get_logger().info("Node Started: Detailed Timing Active")

    def rgb_callback(self, msg):
        # --- TIMER START ---
        self.frame_count += 1
        t_start_proc = time.perf_counter()
        
        # 1. NETWORK LATENCY
        t_now_ros = self.get_clock().now()
        t_msg_header = rclpy.time.Time.from_msg(msg.header.stamp)
        net_ms = (t_now_ros - t_msg_header).nanoseconds / 1e6

        # 2. DECODE
        t_pre_decode = time.perf_counter()
        np_arr = np.frombuffer(msg.data, dtype=np.uint8)
        cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        t_post_decode = time.perf_counter()

        if cv_image is None: return

        # 3. INFERENCE
        t_pre_infer = time.perf_counter()
        # half=True is essential for your FP16 engine speed
        # results = self.model.track(cv_image, persist=True, classes=[0], verbose=False, half=True)
        human_results = self.human_model.predict(cv_image, classes=[0], verbose=False, half=True)

        if self.frame_count % self.button_interval == 0:
            # We save the results to a class variable so they persist
            self.latest_button = self.button_model.predict(cv_image, verbose=False, half=True)
            
        t_post_infer = time.perf_counter()

        # 4. DEPTH & VISUALS
        t_pre_vis = time.perf_counter()
        detection_array = Detection2DArray()
        detection_array.header = msg.header

        # Draw Humans
        self.process_results(human_results, "person", (0, 255, 0), cv_image, detection_array)

        # Draw Buttons from the last time we found them (No flicker!)
        if self.latest_button is not None:
            self.process_results(self.latest_button, "button", (255, 0, 0), cv_image, detection_array)

        self.detection_pub.publish(detection_array)
        t_post_vis = time.perf_counter()

        # 5. PUBLISH DEBUG
        t_pre_pub = time.perf_counter()
        debug_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding="bgr8")
        debug_msg.header = msg.header
        self.debug_pub.publish(debug_msg)
        t_post_pub = time.perf_counter()

        # --- TIMER END ---
        t_end_proc = time.perf_counter()
        
        # Calculate individual step durations in ms
        decode_ms = (t_post_decode - t_pre_decode) * 1000
        infer_ms = (t_post_infer - t_pre_infer) * 1000
        vis_ms = (t_post_vis - t_pre_vis) * 1000
        pub_ms = (t_post_pub - t_pre_pub) * 1000
        total_proc_ms = (t_end_proc - t_start_proc) * 1000
        
        self.get_logger().info(
            f"FPS: {1.0/(t_end_proc - t_start_proc):.1f} | "
            f"NET: {net_ms:.1f}ms | DEC: {decode_ms:.1f}ms | INF: {infer_ms:.1f}ms | "
            f"VIS: {vis_ms:.1f}ms | PUB: {pub_ms:.1f}ms | TOTAL_LATENCY: {net_ms + total_proc_ms:.1f}ms"
        )

    def create_detection_msg(self, class_id, conf, box, header):
        det = Detection2D()
        det.header = header
        x1, y1, x2, y2 = box
        det.bbox.center.position.x = float((x1+x2)/2)
        det.bbox.center.position.y = float((y1+y2)/2)
        det.bbox.size_x = float(x2-x1)
        det.bbox.size_y = float(y2-y1)

        hyp = ObjectHypothesisWithPose()
        hyp.hypothesis.class_id = str(class_id)   # "person"
        hyp.hypothesis.score = float(conf)
        det.results.append(hyp)
        return det

    def process_results(self, results, label_prefix, color, img, det_array):
        boxes = results[0].boxes
        if boxes is not None:
            for box in boxes:
                x1, y1, x2, y2 = map(int, box.xyxy[0].cpu().numpy())
                conf = float(box.conf[0].cpu().numpy())
                
                # Draw on image
                cv2.rectangle(img, (x1, y1), (x2, y2), color, 2)
                cv2.putText(img, f"{label_prefix} {conf:.2f}", (x1, y1 - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

                # Add to ROS message
                det = self.create_detection_msg(label_prefix, conf, [x1, y1, x2, y2], det_array.header)
                det_array.detections.append(det)

def main(args=None):
    rclpy.init(args=args)
    node = HSRDetailedProfileNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        rclpy.shutdown()