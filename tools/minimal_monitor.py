#!/usr/bin/env python3
"""
R2D2 Minimal System Monitor
Shows: Time | Status | Person | Gesture | Faces
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32
import json
from datetime import datetime
import sys

class MinimalMonitor(Node):
    def __init__(self):
        super().__init__('minimal_monitor')
        
        # State
        self.status = "BLUE"
        self.person = "none"
        self.gesture = "--"
        self.faces = 0
        self.last_gesture_time = None
        
        # Subscriptions
        self.create_subscription(String, '/r2d2/audio/person_status', 
                                self.status_callback, 10)
        self.create_subscription(String, '/r2d2/perception/gesture_event', 
                                self.gesture_callback, 10)
        self.create_subscription(Int32, '/r2d2/perception/face_count', 
                                self.face_callback, 10)
        
        # Timer to refresh display (even if no updates)
        self.create_timer(0.5, self.display)
        
        print("\n" + "="*80)
        print("R2D2 MINIMAL MONITOR")
        print("="*80)
        print("Format: TIME | STATUS | Person | Gesture | Faces")
        print("="*80 + "\n")
    
    def status_callback(self, msg):
        try:
            data = json.loads(msg.data)
            self.status = data.get('status', 'unknown').upper()
            # person_identity is a string directly: "severin" | "unknown" | "no_person"
            self.person = data.get('person_identity', 'unknown')
        except:
            pass
    
    def gesture_callback(self, msg):
        self.gesture = msg.data
        self.last_gesture_time = datetime.now()
    
    def face_callback(self, msg):
        self.faces = msg.data
    
    def display(self):
        # Clear gesture if it's more than 2 seconds old
        if self.last_gesture_time:
            age = (datetime.now() - self.last_gesture_time).total_seconds()
            if age > 2.0:
                self.gesture = "--"
        
        # Color codes
        status_display = {
            'RED': '\033[1;31m🔴 RED  \033[0m',
            'GREEN': '\033[1;32m🟢 GREEN\033[0m',
            'BLUE': '\033[1;34m🔵 BLUE \033[0m'
        }.get(self.status, self.status)
        
        # Format output
        timestamp = datetime.now().strftime("%H:%M:%S")
        person_str = f"Person: {self.person:10s}"
        gesture_str = f"Gesture: {self.gesture:20s}"
        faces_str = f"Faces: {self.faces}"
        
        # Print with carriage return (overwrites line)
        output = f"{timestamp} | {status_display} | {person_str} | {gesture_str} | {faces_str}"
        print(f"\r{output}", end='', flush=True)

def main():
    rclpy.init()
    monitor = MinimalMonitor()
    
    try:
        rclpy.spin(monitor)
    except KeyboardInterrupt:
        print("\n\nMonitor stopped.")
    finally:
        monitor.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

