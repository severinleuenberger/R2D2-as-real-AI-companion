#!/usr/bin/env python3
"""
R2D2 Gesture & Speech Monitor
Real-time monitoring of gestures, person status, and speech session
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32, Float32, Bool
import json
from datetime import datetime
import sys


class GestureMonitor(Node):
    def __init__(self):
        super().__init__('gesture_monitor')
        
        # State tracking
        self.person_status_data = None
        self.gesture_event = None
        self.session_status_data = None
        self.person_id = None
        self.face_count = None
        self.last_gesture_time = None
        self.last_status_time = None
        self.last_session_time = None
        
        # Create subscriptions
        self.status_sub = self.create_subscription(
            String, '/r2d2/audio/person_status', self.status_callback, 10)
        
        self.gesture_sub = self.create_subscription(
            String, '/r2d2/perception/gesture_event', self.gesture_callback, 10)
        
        self.session_sub = self.create_subscription(
            String, '/r2d2/speech/session_status', self.session_callback, 10)
        
        self.person_id_sub = self.create_subscription(
            String, '/r2d2/perception/person_id', self.person_id_callback, 10)
        
        self.face_count_sub = self.create_subscription(
            Int32, '/r2d2/perception/face_count', self.face_count_callback, 10)
        
        # Timer for display refresh
        self.create_timer(0.5, self.display_status)
        
        print("\n" + "="*80)
        print("R2D2 GESTURE & SPEECH MONITOR")
        print("="*80)
        print("Monitoring: Gestures, Person Status (RED/GREEN/BLUE), Speech Session")
        print("="*80 + "\n")
    
    def status_callback(self, msg):
        try:
            self.person_status_data = json.loads(msg.data)
            self.last_status_time = datetime.now()
        except:
            self.person_status_data = {'status': 'error'}
    
    def gesture_callback(self, msg):
        self.gesture_event = msg.data
        self.last_gesture_time = datetime.now()
        print(f"\n🤚 GESTURE DETECTED: {msg.data} at {self.last_gesture_time.strftime('%H:%M:%S.%f')[:-3]}")
        sys.stdout.flush()
    
    def session_callback(self, msg):
        try:
            self.session_status_data = json.loads(msg.data)
            self.last_session_time = datetime.now()
            status = self.session_status_data.get('status', 'unknown')
            print(f"\n🎤 SPEECH SESSION: {status.upper()} at {self.last_session_time.strftime('%H:%M:%S.%f')[:-3]}")
            sys.stdout.flush()
        except:
            self.session_status_data = {'status': 'error'}
    
    def person_id_callback(self, msg):
        self.person_id = msg.data
    
    def face_count_callback(self, msg):
        self.face_count = msg.data
    
    def display_status(self):
        # Clear screen
        print("\033[H\033[J", end='')
        
        now = datetime.now()
        print("="*80)
        print(f"R2D2 GESTURE & SPEECH MONITOR - {now.strftime('%H:%M:%S')}")
        print("="*80)
        
        # Person Status (RED/GREEN/BLUE)
        if self.person_status_data:
            status = self.person_status_data.get('status', 'unknown')
            person = self.person_status_data.get('person_identity', 'unknown')
            duration = self.person_status_data.get('duration_in_state', 0.0)
            
            if status == 'red':
                status_display = f"🔴 RED (Recognized: {person})"
                status_color = "\033[91m"  # Red
            elif status == 'green':
                status_display = f"🟢 GREEN (Unknown person)"
                status_color = "\033[92m"  # Green
            elif status == 'blue':
                status_display = f"🔵 BLUE (No person)"
                status_color = "\033[94m"  # Blue
            else:
                status_display = f"⚪ {status.upper()}"
                status_color = "\033[0m"
            
            print(f"\n{status_color}PERSON STATUS: {status_display}\033[0m")
            print(f"  Duration: {duration:.1f}s")
            if self.last_status_time:
                age = (now - self.last_status_time).total_seconds()
                print(f"  Last update: {age:.1f}s ago")
        else:
            print(f"\n⚪ PERSON STATUS: No data")
        
        # Face Detection
        print(f"\n👤 FACE DETECTION:")
        print(f"  Face count: {self.face_count if self.face_count is not None else 'N/A'}")
        print(f"  Person ID: {self.person_id if self.person_id else 'N/A'}")
        
        # Last Gesture
        print(f"\n🤚 LAST GESTURE:")
        if self.gesture_event and self.last_gesture_time:
            age = (now - self.last_gesture_time).total_seconds()
            print(f"  Gesture: {self.gesture_event}")
            print(f"  Detected: {age:.1f}s ago")
        else:
            print(f"  None detected yet")
        
        # Speech Session Status
        print(f"\n🎤 SPEECH SESSION:")
        if self.session_status_data:
            session_state = self.session_status_data.get('status', 'unknown')
            if session_state == 'connected':
                print(f"  Status: \033[92m✅ CONNECTED (Active)\033[0m")
            elif session_state == 'active':
                print(f"  Status: ⚡ ACTIVE (Ready)")
            elif session_state == 'disconnected':
                print(f"  Status: ⭕ DISCONNECTED")
            else:
                print(f"  Status: {session_state.upper()}")
            
            if self.last_session_time:
                age = (now - self.last_session_time).total_seconds()
                print(f"  Last update: {age:.1f}s ago")
        else:
            print(f"  Status: No data")
        
        # Gesture Requirements
        print(f"\n📋 GESTURE REQUIREMENTS:")
        print(f"  ✓ Person must be RED (recognized)")
        print(f"  ✓ Index finger up = Start conversation")
        print(f"  ✓ Fist = Stop conversation")
        print(f"  ✓ Session must be inactive to start")
        print(f"  ✓ Session must be active to stop")
        
        print("\n" + "="*80)
        print("Press Ctrl+C to exit | Waiting for gestures...")
        print("="*80)
        
        sys.stdout.flush()


def main(args=None):
    rclpy.init(args=args)
    monitor = GestureMonitor()
    
    try:
        rclpy.spin(monitor)
    except KeyboardInterrupt:
        print("\n\nMonitor stopped by user")
    finally:
        monitor.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

