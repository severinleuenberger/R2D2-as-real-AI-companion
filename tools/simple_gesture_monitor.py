#!/usr/bin/env python3
"""
Simple Index Finger Gesture Monitor
Just shows when index_finger_up is detected on the topic
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from datetime import datetime

class SimpleGestureMonitor(Node):
    def __init__(self):
        super().__init__('simple_gesture_monitor')
        
        self.sub = self.create_subscription(
            String,
            '/r2d2/perception/gesture_event',
            self.callback,
            10
        )
        
        print("\n" + "="*60)
        print("SIMPLE GESTURE MONITOR")
        print("="*60)
        print("Listening for gestures on /r2d2/perception/gesture_event")
        print("Make index finger up gesture now...")
        print("="*60 + "\n")
    
    def callback(self, msg):
        now = datetime.now().strftime('%H:%M:%S.%f')[:-3]
        gesture = msg.data
        
        if gesture == "index_finger_up":
            print(f"👆 INDEX FINGER UP detected at {now}")
        elif gesture == "fist":
            print(f"✊ FIST detected at {now}")
        else:
            print(f"🤚 {gesture} detected at {now}")

def main():
    rclpy.init()
    node = SimpleGestureMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nStopped")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

