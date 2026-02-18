import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image, CameraInfo, CompressedImage
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose

from cv_bridge import CvBridge
from ultralytics import YOLO
from ament_index_python.packages import get_package_share_directory

import os
import cv2
import numpy as np
import message_filters  # sudo apt install ros-$ROS_DISTRO-message-filters


class HSRVisionNode(Node):
    def __init__(self):
        super().__init__('hsr_vision_node')

        # 1. Setup Model
        package_share_dir = get_package_share_directory('hsr_vision')
        model_path = os.path.join(package_share_dir, 'models', 'cardboard.pt')
        self.model = YOLO(model_path)
        self.class_names = self.model.names

        # CvBridge still used for depth + debug publishing
        self.bridge = CvBridge()

        # 2. Camera Intrinsics (better: subscribe to camera_info)
        self.fx, self.fy = 525.0, 525.0
        self.cx, self.cy = 320.0, 240.0

        # 3. Synchronized Subscriptions
        # RGB is COMPRESSED:
        self.rgb_sub = message_filters.Subscriber(
            self,
            CompressedImage,
            '/rgb_PS1080_PrimeSense/rgb/image_rect_color/compressed'
        )

        # Depth stays RAW (32FC1):
        self.depth_sub = message_filters.Subscriber(
            self,
            Image,
            '/rgb_PS1080_PrimeSense/depth_registered/image_raw'
        )

        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.rgb_sub, self.depth_sub],
            queue_size=10,
            slop=0.1
        )
        self.ts.registerCallback(self.synchronized_callback)

        self.detection_pub = self.create_publisher(Detection2DArray, '/detected_buttons', 10)
        self.debug_pub = self.create_publisher(Image, '/detected_image_debug', 10)

        self.frame_counter = 0
        self.process_every_n_frames = 10

        self.get_logger().info("HSR Vision Node using COMPRESSED RGB + raw depth.")

    def decode_compressed_bgr(self, rgb_msg: CompressedImage):
        # rgb_msg.format often "jpeg" or "png"
        np_arr = np.frombuffer(rgb_msg.data, dtype=np.uint8)
        bgr = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        return bgr

    def synchronized_callback(self, rgb_msg: CompressedImage, depth_msg: Image):
        self.frame_counter += 1
        if self.frame_counter % self.process_every_n_frames != 0:
            return

        cv_image = self.decode_compressed_bgr(rgb_msg)
        if cv_image is None:
            self.get_logger().warn("Failed to decode compressed RGB image.")
            return

        depth_image = self.bridge.imgmsg_to_cv2(depth_msg, "32FC1")  # meters

        results = self.model(cv_image, conf=0.25, verbose=False)

        detection_array = Detection2DArray()
        detection_array.header = rgb_msg.header  # CompressedImage has header

        found_any = False
        for result in results:
            for box in result.boxes:
                found_any = True

                u, v = int(box.xywh[0][0]), int(box.xywh[0][1])

                # Clamp ROI to image bounds (prevents edge crashes)
                h, w = depth_image.shape[:2]
                u0, u1 = max(u - 2, 0), min(u + 3, w)
                v0, v1 = max(v - 2, 0), min(v + 3, h)

                z = np.nanmedian(depth_image[v0:v1, u0:u1])

                if np.isfinite(z) and z > 0:
                    x = (u - self.cx) * z / self.fx
                    y = (v - self.cy) * z / self.fy
                    self.get_logger().info(
                        f"FOUND: {self.class_names[int(box.cls[0])]} at X:{x:.2f} Y:{y:.2f} Z:{z:.2f}"
                    )

                det = self.create_detection_msg(box)
                detection_array.detections.append(det)

                x1, y1, x2, y2 = map(int, box.xyxy[0])
                cv2.rectangle(cv_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
                if np.isfinite(z):
                    cv2.putText(
                        cv_image, f"Z: {z:.2f}m", (x1, max(0, y1 - 10)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2
                    )

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
