#!/usr/bin/env python3
"""
Bridge for all ADA buttons and doors using EE proximity detection.
No TouchPlugin or physics contact required.
"""

import subprocess
import sys
import threading
import math
import rclpy
from rclpy.node import Node
from tf2_ros import TransformListener, Buffer

double_door_opened = False
side_door_opened   = False

# Button positions [x, y, z] and their proximity threshold
BUTTONS = [
    {"pos": [2.94, -1.35, 1.0], "type": "double", "name": "room-side double door"},
    {"pos": [3.06, -1.35, 1.0], "type": "double", "name": "interior double door"},
    {"pos": [4.8,   1.44, 1.0], "type": "double", "name": "hallway double door"},
    {"pos": [6.75, -1.44, 1.0], "type": "side",   "name": "hallway side room"},
    {"pos": [6.75, -1.56, 1.0], "type": "side",   "name": "interior side room"},
]
PROXIMITY_THRESHOLD = 0.15  # 8cm — tight enough to require intent


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


def open_then_close_side():
    global side_door_opened
    print("Opening side room door...")
    send_command("/model/side_room_door/door_cmd", "data: 1.57")
    print("Side door open! Auto-closing in 15 seconds...")
    threading.Event().wait(15.0)
    print("Closing side room door...")
    send_command("/model/side_room_door/door_cmd", "data: 0.0")
    print("Side door closed.")
    side_door_opened = False


def handle_double_door(source):
    global double_door_opened
    if double_door_opened:
        print(f"[{source}] Double doors already open, ignoring.")
        return
    double_door_opened = True
    print(f"[{source}] Button pressed!")
    threading.Thread(target=open_then_close_double, daemon=True).start()


def handle_side_door(source):
    global side_door_opened
    if side_door_opened:
        print(f"[{source}] Side door already open, ignoring.")
        return
    side_door_opened = True
    print(f"[{source}] Button pressed!")
    threading.Thread(target=open_then_close_side, daemon=True).start()


class ProximityBridge(Node):
    def __init__(self):
        super().__init__('proximity_bridge')
        self.tf_buffer   = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.timer = self.create_timer(0.05, self.check_proximity)  # 20Hz
        self.get_logger().info("Proximity bridge started. Monitoring EE position...")

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
                if btn["type"] == "double":
                    handle_double_door(btn["name"])
                else:
                    handle_side_door(btn["name"])


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