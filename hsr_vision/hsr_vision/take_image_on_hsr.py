import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os
import re
import sys
import termios
import tty
import threading

class ImageSnapper(Node):
    def __init__(self):
        super().__init__('image_snapper')
        self.bridge = CvBridge()
        self.latest_msg = None
        
        self.save_dir = os.path.expanduser('~/button_dataset')
        if not os.path.exists(self.save_dir):
            os.makedirs(self.save_dir)

        self.subscription = self.create_subscription(
            Image,
            '/rgb_PS1080_PrimeSense/rgb/image_rect_color',
            self.image_callback,
            10)
        
        self.get_logger().info('--- SNAPPER READY ---')
        self.get_logger().info('Press SPACE to capture, or ESC to quit.')

    def image_callback(self, msg):
        # Just store the latest frame in memory
        self.latest_msg = msg

    def get_next_filename(self):
        prefix = "push_hsr_"
        extension = ".jpg"
        files = os.listdir(self.save_dir)
        indices = [int(re.search(r'(\d+)', f).group()) for f in files 
                   if f.startswith(prefix) and f.endswith(extension)]
        next_idx = max(indices) + 1 if indices else 1
        return os.path.join(self.save_dir, f"{prefix}{next_idx:03d}{extension}")

    def capture_image(self):
        if self.latest_msg is None:
            self.get_logger().warn('No frame received yet!')
            return

        try:
            cv_image = self.bridge.imgmsg_to_cv2(self.latest_msg, "bgr8")
            full_path = self.get_next_filename()
            cv2.imwrite(full_path, cv_image)
            self.get_logger().info(f'📸 CAPTURED: {os.path.basename(full_path)}')
        except Exception as e:
            self.get_logger().error(f'Capture failed: {e}')

def get_key():
    """Reads a single keypress from the terminal."""
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch

def main():
    rclpy.init()
    node = ImageSnapper()
    
    # Spin the node in a separate thread so it keeps updating latest_msg
    thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    thread.start()

    try:
        while rclpy.ok():
            key = get_key()
            if key == ' ':  # Spacebar
                node.capture_image()
            elif key == '\x1b':  # ESC key
                print("\nExiting...")
                break
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()