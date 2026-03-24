#!/usr/bin/env python3
"""
Bridge for behavior world: double doors + simulated person.
- Monitors EE proximity to both ADA buttons
- On button press: opens doors, then after a delay walks the sim_person through
"""

import subprocess
import threading
import math
import time
import rclpy
from rclpy.node import Node
from tf2_ros import TransformListener, Buffer


double_door_opened = False

BUTTONS = [
    {"pos": [2.94, -1.35, 1.0], "name": "front-right button"},
    {"pos": [3.06,  1.35, 1.0], "name": "back-left button"},
]
PROXIMITY_THRESHOLD = 0.15

def send_command(topic, value):
    subprocess.run(
        ["ign", "topic", "-t", topic, "-m", "ignition.msgs.Double", "-p", value],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL
    )



def open_then_close_double():
    global double_door_opened
    print("Opening double doors...")
    t1 = threading.Thread(target=send_command, args=("/model/handicap_door_left/door_cmd",  "data: 1.57"))
    t2 = threading.Thread(target=send_command, args=("/model/handicap_door_right/door_cmd", "data: -1.57"))
    t1.start(); t2.start(); t1.join(); t2.join()
    print("Double doors open! Auto-closing in 15 seconds...")

    threading.Event().wait(15.0)
    print("Closing double doors...")
    t1 = threading.Thread(target=send_command, args=("/model/handicap_door_left/door_cmd",  "data: 0.0"))
    t2 = threading.Thread(target=send_command, args=("/model/handicap_door_right/door_cmd", "data: 0.0"))
    t1.start(); t2.start(); t1.join(); t2.join()
    print("Double doors closed.")
    double_door_opened = False


def handle_double_door(source):
    global double_door_opened
    if double_door_opened:
        print(f"[{source}] Double doors already open, ignoring.")
        return
    double_door_opened = True
    print(f"[{source}] Button pressed!")
    threading.Thread(target=open_then_close_double, daemon=True).start()


class ProximityBridge(Node):
    def __init__(self):
        super().__init__('proximity_bridge')
        self.tf_buffer   = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.timer = self.create_timer(0.05, self.check_proximity)
        self.get_logger().info("Behavior proximity bridge started. Monitoring EE position...")

    def check_proximity(self):
        try:
            tf = self.tf_buffer.lookup_transform('odom', 'hand_palm_link', rclpy.time.Time())
            ee = tf.transform.translation
        except Exception:
            return

        for btn in BUTTONS:
            dist = math.sqrt(
                (ee.x - btn["pos"][0])**2 +
                (ee.y - btn["pos"][1])**2 +
                (ee.z - btn["pos"][2])**2
            )
            if dist < PROXIMITY_THRESHOLD:
                self.get_logger().info(
                    f"EE near [{btn['name']}]: dist={dist:.3f}m — TRIGGERED",
                    throttle_duration_sec=2.0
                )
                handle_double_door(btn["name"])


def main():
    rclpy.init()
    node = ProximityBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nStopping...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()