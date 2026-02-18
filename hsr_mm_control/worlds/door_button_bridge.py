#!/usr/bin/env python3
"""
Bridge to open door when button is touched.
The door will open smoothly due to the velocity limit in the joint.
"""

import subprocess
import sys

door_opened = False

def send_door_command():
    """Send command to open the door to -1.57 radians (opens away)"""
    global door_opened
    if door_opened:
        print("Door already opened!")
        return
        
    cmd = [
        "ign", "topic", "-t", "/model/handicap_door/door_cmd",
        "-m", "ignition.msgs.Double", "-p", "data: -1.57"
    ]
    subprocess.run(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    print("Door opening command sent! (will open smoothly over ~2 seconds)")
    door_opened = True

def main():
    print("Listening for button presses on /button_logic/touched...")
    print("Press Ctrl+C to stop.")
    
    # Listen to button topic
    cmd = ["ign", "topic", "-e", "-t", "/button_logic/touched"]
    process = subprocess.Popen(cmd, stdout=subprocess.PIPE, text=True, bufsize=1)
    
    try:
        for line in process.stdout:
            if "true" in line.lower():
                send_door_command()
    except KeyboardInterrupt:
        print("\nStopping...")
        process.terminate()
        sys.exit(0)

if __name__ == "__main__":
    main()