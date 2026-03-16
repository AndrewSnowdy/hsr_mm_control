#!/usr/bin/env python3
"""
Bridge to open both door leaves when button is touched.
Doors open simultaneously, then auto-close after 5 seconds.
"""

import subprocess
import sys
import threading

door_opened = False

def send_command(topic, value):
    """Send a single door command."""
    subprocess.run(
        ["ign", "topic", "-t", topic, "-m", "ignition.msgs.Double", "-p", value],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL
    )

def send_door_commands(open_value_left, open_value_right):
    """Send commands to both doors simultaneously using threads."""
    t1 = threading.Thread(target=send_command, args=("/model/handicap_door_left/door_cmd", open_value_left))
    t2 = threading.Thread(target=send_command, args=("/model/handicap_door_right/door_cmd", open_value_right))
    t1.start()
    t2.start()
    t1.join()
    t2.join()

def open_then_close():
    global door_opened

    # Open both doors simultaneously
    print("Opening doors...")
    send_door_commands("data: 1.57", "data: -1.57")
    print("Doors open! Auto-closing in 5 seconds...")

    # Wait for doors to fully open (~2s) + hold open for 5s
    threading.Event().wait(15.0)

    # Close both doors simultaneously
    print("Closing doors...")
    send_door_commands("data: 0.0", "data: 0.0")
    print("Doors closed.")

    # Reset so button can trigger again
    door_opened = False

def handle_button_press():
    global door_opened
    if door_opened:
        print("Doors already opening/open!")
        return
    door_opened = True
    threading.Thread(target=open_then_close, daemon=True).start()

def main():
    print("Listening for button presses on /button_logic/touched...")
    print("Press Ctrl+C to stop.")

    process = subprocess.Popen(
        ["ign", "topic", "-e", "-t", "/button_logic/touched"],
        stdout=subprocess.PIPE, text=True, bufsize=1
    )

    try:
        for line in process.stdout:
            if "true" in line.lower():
                handle_button_press()
    except KeyboardInterrupt:
        print("\nStopping...")
        process.terminate()
        sys.exit(0)

if __name__ == "__main__":
    main()