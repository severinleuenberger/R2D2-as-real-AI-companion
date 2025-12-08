#!/usr/bin/env python3
"""
R2D2 Face Recognition & Audio Notification Monitor
Real-time status display with 5-second state machine logic
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import sys
from datetime import datetime
import time
import subprocess

class FaceStateManager:
    """Manages face recognition state with jitter tolerance and loss confirmation"""
    
    def __init__(self):
        # States: UNKNOWN, RECOGNIZED, LOSS_CONFIRMING, LOST
        self.state = "UNKNOWN"
        self.last_recognition_time = None
        self.last_loss_start_time = None
        self.jitter_tolerance_sec = 5.0
        self.loss_confirmation_sec = 5.0
        self.recognition_cooldown_sec = 2.0
        self.last_beep_time = None
        
    def update(self, raw_face_data):
        """
        Update state based on raw face detection
        Returns: (state, display_text, should_beep, beep_type)
        """
        current_time = time.time()
        should_beep = False
        beep_type = None
        
        # PHASE 1: Recognition Detection
        if raw_face_data == "severin":
            if self.state == "UNKNOWN":
                # Transition: UNKNOWN → RECOGNIZED
                self.state = "RECOGNIZED"
                self.last_recognition_time = current_time
                self.last_loss_start_time = None
                should_beep = True
                beep_type = "recognition"
                display_text = "✅ severin RECOGNIZED"
                
            elif self.state == "LOST":
                # Transition: LOST → RECOGNIZED
                self.state = "RECOGNIZED"
                self.last_recognition_time = current_time
                self.last_loss_start_time = None
                should_beep = True
                beep_type = "recognition"
                display_text = "✅ severin RECOGNIZED (recovered)"
                
            elif self.state == "LOSS_CONFIRMING":
                # Cancel loss, return to RECOGNIZED
                self.state = "RECOGNIZED"
                self.last_recognition_time = current_time
                self.last_loss_start_time = None
                display_text = "✅ severin RECOGNIZED (jitter recovered)"
                
            else:  # Already RECOGNIZED
                display_text = "✅ severin RECOGNIZED"
        
        # PHASE 2: No recognition detected
        else:  # raw_face_data == "unknown"
            if self.state == "RECOGNIZED":
                # Start jitter tolerance window
                if self.last_loss_start_time is None:
                    self.last_loss_start_time = current_time
                
                time_since_loss = current_time - self.last_loss_start_time
                
                if time_since_loss < self.jitter_tolerance_sec:
                    # Within jitter window - stay RECOGNIZED
                    display_text = f"✅ severin RECOGNIZED (jitter: {self.jitter_tolerance_sec - time_since_loss:.1f}s)"
                else:
                    # Jitter window expired, move to LOSS_CONFIRMING
                    self.state = "LOSS_CONFIRMING"
                    display_text = "⚠️  severin LOSS_CONFIRMING (5s timeout)"
            
            elif self.state == "LOSS_CONFIRMING":
                # In confirmation window
                time_since_loss = current_time - self.last_loss_start_time
                
                if time_since_loss < (self.jitter_tolerance_sec + self.loss_confirmation_sec):
                    # Still in confirmation window
                    remaining = self.jitter_tolerance_sec + self.loss_confirmation_sec - time_since_loss
                    display_text = f"⚠️  severin LOSS_CONFIRMING (confirm: {remaining:.1f}s)"
                else:
                    # Confirmation complete - move to LOST
                    self.state = "LOST"
                    should_beep = True
                    beep_type = "loss"
                    display_text = "❌ severin LOST (alert triggered)"
            
            elif self.state == "LOST":
                display_text = "❌ severin LOST"
            
            else:  # UNKNOWN
                display_text = "❓ unknown (waiting for face)"
        
        return self.state, display_text, should_beep, beep_type


class MonitorNode(Node):
    def __init__(self):
        super().__init__('face_monitor')
        
        # Subscribe to face recognition topic
        self.face_sub = self.create_subscription(
            String,
            '/r2d2/perception/person_id',
            self.face_callback,
            10
        )
        
        # Subscribe to audio notification events
        self.audio_sub = self.create_subscription(
            String,
            '/r2d2/audio/notification_event',
            self.audio_callback,
            10
        )
        
        self.state_manager = FaceStateManager()
        self.last_face = None
        self.last_audio = None
        self.face_count = 0
        self.audio_count = 0
        
        print("\n" + "="*70)
        print("🔍 R2D2 FACE RECOGNITION & AUDIO MONITOR (with State Machine)")
        print("="*70)
        print("\n📊 Real-time Status:\n")
        print("Raw Detection: Waiting...")
        print("State (5s logic): Waiting...")
        print("Audio Events: Waiting...\n")
        print("="*70 + "\n")
        
    def play_beep(self, beep_type):
        """Play beep sound via audio_beep.py utility"""
        try:
            if beep_type == "recognition":
                # Single beep: 400 Hz, 0.5 sec, 25% volume
                subprocess.run([
                    'python3', '/home/severin/dev/r2d2/audio_beep.py',
                    '--frequency', '400',
                    '--duration', '0.5',
                    '--volume', '0.25'
                ], timeout=2, capture_output=True)
            elif beep_type == "loss":
                # Double beep: 400 Hz, 0.3 sec x2, 25% volume
                for _ in range(2):
                    subprocess.run([
                        'python3', '/home/severin/dev/r2d2/audio_beep.py',
                        '--frequency', '400',
                        '--duration', '0.3',
                        '--volume', '0.25'
                    ], timeout=2, capture_output=True)
                    time.sleep(0.1)  # Small gap between beeps
        except Exception as e:
            self.get_logger().warn(f"Beep error: {e}")
        
    def face_callback(self, msg):
        """Handle face recognition updates with state machine logic"""
        self.face_count += 1
        timestamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        raw_face_data = msg.data
        
        # Update state machine
        state, state_display, should_beep, beep_type = self.state_manager.update(raw_face_data)
        
        # Play beep if triggered
        if should_beep and beep_type:
            beep_emoji = "🔊" if beep_type == "recognition" else "🔔🔔"
            print(f"\n{timestamp} | 🎵 BEEPING: {beep_emoji} ({beep_type})")
            self.play_beep(beep_type)
        
        # Color codes for state display
        if "RECOGNIZED" in state_display:
            state_color = "\033[92m"  # Green
        elif "LOSS_CONFIRMING" in state_display:
            state_color = "\033[93m"  # Yellow
        elif "LOST" in state_display:
            state_color = "\033[91m"  # Red
        else:
            state_color = "\033[94m"  # Blue
        
        reset_color = "\033[0m"
        
        # Display raw detection and state
        raw_color = "\033[96m" if raw_face_data == "severin" else "\033[95m"
        display_line = f"{timestamp} | Raw: {raw_color}{raw_face_data}{reset_color} | State: {state_color}{state_display}{reset_color}"
        
        sys.stdout.write(f"\r{display_line}              ")
        sys.stdout.flush()
        
        self.last_face = (raw_face_data, state, timestamp)
    
    def audio_callback(self, msg):
        """Handle audio notification events"""
        self.audio_count += 1
        timestamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        event = msg.data
        
        # New line for audio events so they don't overwrite face status
        print(f"\n{timestamp} | 📢 Service Event: {event}")
        
        # Re-print current face status
        if self.last_face:
            raw_face_data, state, _ = self.last_face
            if "RECOGNIZED" in state:
                state_color = "\033[92m"
            elif "LOSS_CONFIRMING" in state:
                state_color = "\033[93m"
            elif "LOST" in state:
                state_color = "\033[91m"
            else:
                state_color = "\033[94m"
            
            reset_color = "\033[0m"
            raw_color = "\033[96m" if raw_face_data == "severin" else "\033[95m"
            
            ts = datetime.now().strftime("%H:%M:%S.%f")[:-3]
            sys.stdout.write(f"{ts} | Raw: {raw_color}{raw_face_data}{reset_color} | State: {state_color}{state}{reset_color}              ")
            sys.stdout.flush()
        
        self.last_audio = (event, timestamp)


def main(args=None):
    rclpy.init(args=args)
    
    try:
        monitor = MonitorNode()
        
        print("\n⏳ Monitoring started. Press Ctrl+C to stop.\n")
        print("📋 STATE MACHINE BEHAVIOR:")
        print("  ✅ RECOGNIZED    - Face detected, steady for 0-5 sec")
        print("  ⚠️  CONFIRMING   - Loss detected, waiting 5 sec confirmation")
        print("  ❌ LOST          - Loss confirmed (5+5 sec), 🔔🔔 double beep")
        print("  🔊 BEEP triggers on state transitions (recognition & loss)\n")
        print("-" * 70 + "\n")
        
        rclpy.spin(monitor)
        
    except KeyboardInterrupt:
        print("\n\n" + "="*70)
        print("📊 MONITORING SUMMARY")
        print("="*70)
        print(f"Face detections received: {monitor.face_count}")
        print(f"Audio events triggered: {monitor.audio_count}")
        if monitor.last_face:
            raw, state, ts = monitor.last_face
            print(f"Last face status: {state} (raw: {raw}) at {ts}")
        if monitor.last_audio:
            print(f"Last audio event: {monitor.last_audio[0]} ({monitor.last_audio[1]})")
        print("="*70 + "\n")
    finally:
        monitor.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
