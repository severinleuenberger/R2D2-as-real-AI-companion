#!/usr/bin/env python3
"""
R2D2 Minimal System Monitor
Shows: Time | Status | Person | Gesture | Faces | Speech | Phase
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32
import json
from datetime import datetime

class MinimalMonitor(Node):
    def __init__(self):
        super().__init__('minimal_monitor')
        
        # State
        self.status = "BLUE"
        self.person = "no_person"
        self.gesture = "--"
        self.faces = 0
        self.fast_mode_active = False  # Index finger mode
        self.r2d2_mode_active = False  # Open hand mode
        self.last_gesture_time = None
        
        # Subscriptions
        self.create_subscription(String, '/r2d2/audio/person_status', 
                                self.status_callback, 10)
        self.create_subscription(String, '/r2d2/perception/gesture_event', 
                                self.gesture_callback, 10)
        self.create_subscription(Int32, '/r2d2/perception/face_count', 
                                self.face_callback, 10)
        self.create_subscription(String, '/r2d2/speech/session_status', 
                                self.fast_mode_callback, 10)
        self.create_subscription(String, '/r2d2/speech/intelligent/session_status', 
                                self.r2d2_mode_callback, 10)
        
        # Timer to refresh display
        self.create_timer(0.5, self.display)
        
        print("\n" + "="*105)
        print("R2D2 MINIMAL MONITOR - Dual Mode Speech System")
        print("="*105)
        print("TIME     | STATUS  | Person     | Gest | Faces | Speech Mode      | Phase")
        print("="*105 + "\n")
    
    def status_callback(self, msg):
        try:
            data = json.loads(msg.data)
            self.status = data.get('status', 'unknown').upper()
            # person_identity is a string directly: "severin" | "unknown" | "no_person"
            self.person = data.get('person_identity', 'unknown')
        except:
            pass
    
    def gesture_callback(self, msg):
        # Convert gesture names to symbols
        gesture_map = {
            'index_finger_up': '☝️',   # Fast Mode
            'fist': '✊',              # Stop
            'open_hand': '🖐️'          # Intelligent Mode
        }
        self.gesture = gesture_map.get(msg.data, msg.data)
        self.last_gesture_time = datetime.now()
    
    def face_callback(self, msg):
        self.faces = msg.data
    
    def fast_mode_callback(self, msg):
        try:
            data = json.loads(msg.data)
            # "connected" = active Fast Mode conversation (index finger)
            self.fast_mode_active = (data.get('status', '') == 'connected')
        except:
            pass
    
    def r2d2_mode_callback(self, msg):
        try:
            data = json.loads(msg.data)
            # "session_active" = active R2-D2 Mode conversation (open hand)
            self.r2d2_mode_active = (data.get('session_active', False) == True)
        except:
            pass
    
    def get_phase(self):
        """Determine current system phase based on state."""
        if self.status == "BLUE":
            return "Phase 1: Waiting"
        elif self.status == "GREEN":
            return "Phase 3: Unknown"
        elif self.status == "RED":
            if self.fast_mode_active:
                return "Phase 5-7: Fast Mode"
            elif self.r2d2_mode_active:
                return "Phase 5-7: R2-D2 Mode"
            else:
                return "Phase 4: Ready"
        return "Phase ?: --"
    
    def get_speech_mode(self):
        """Get current speech mode with emoji."""
        if self.fast_mode_active:
            return "☝️  Fast (RT)"
        elif self.r2d2_mode_active:
            return "🖐️  R2-D2 (REST)"
        else:
            return "🔇 OFF"
    
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
        person_str = f"{self.person:10s}"
        gesture_str = f"{self.gesture:4s}"
        speech_mode_str = f"{self.get_speech_mode():16s}"
        phase_str = self.get_phase()
        
        # Print with carriage return (overwrites line)
        output = f"{timestamp} | {status_display} | {person_str} | {gesture_str} | {self.faces}     | {speech_mode_str} | {phase_str}"
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
