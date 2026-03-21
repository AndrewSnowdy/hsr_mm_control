#!/usr/bin/env python3
"""
Bridge for all ADA buttons and doors.

Double door (3 buttons):
  - /button_logic/touched               (room-side)
  - /button_hallway_logic/touched        (hallway north wall)

Side room door (2 buttons):
  - /button_side_logic/touched           (hallway-side)
  - /button_side_interior_logic/touched  (room-side)
"""

import subprocess
import sys
import threading

double_door_opened = False
side_door_opened = False

def send_command(topic, value):
    subprocess.run(
        ["ign", "topic", "-t", topic, "-m", "ignition.msgs.Double", "-p", value],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL
    )

# ── Double door ──────────────────────────────────────────────────────────────

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

# ── Side room door ───────────────────────────────────────────────────────────

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

def handle_side_door(source):
    global side_door_opened
    if side_door_opened:
        print(f"[{source}] Side door already open, ignoring.")
        return
    side_door_opened = True
    print(f"[{source}] Button pressed!")
    threading.Thread(target=open_then_close_side, daemon=True).start()

# ── Listener ─────────────────────────────────────────────────────────────────

def listen_topic(topic, handler, source):
    process = subprocess.Popen(
        ["ign", "topic", "-e", "-t", topic],
        stdout=subprocess.PIPE, text=True, bufsize=1
    )
    try:
        for line in process.stdout:
            if "true" in line.lower():
                handler(source)
    finally:
        process.terminate()

# ── Main ─────────────────────────────────────────────────────────────────────

def main():
    print("Listening for all ADA button presses...")
    print("Press Ctrl+C to stop.\n")

    listeners = [
        ("/button_logic/touched",              handle_double_door, "room-side double door button"),
        ("/button_hallway_logic/touched",       handle_double_door, "hallway double door button"),
        ("/button_side_logic/touched",          handle_side_door,   "hallway side room button"),
        ("/button_side_interior_logic/touched", handle_side_door,   "interior side room button"),
    ]

    threads = [
        threading.Thread(target=listen_topic, args=args, daemon=True)
        for args in listeners
    ]

    for t in threads:
        t.start()

    try:
        for t in threads:
            t.join()
    except KeyboardInterrupt:
        print("\nStopping...")
        sys.exit(0)

if __name__ == "__main__":
    main()