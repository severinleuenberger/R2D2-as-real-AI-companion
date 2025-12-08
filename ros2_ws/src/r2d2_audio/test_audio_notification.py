#!/usr/bin/env python3
"""
Test script for R2D2 Audio Notification System

This script demonstrates how the audio notification node works:
1. Publishes fake face recognition data to /r2d2/perception/person_id
2. The audio_notification_node listens and triggers beeps when it recognizes you

Usage:
  # Terminal 1: Start the audio notification node
  ros2 launch r2d2_audio audio_notification.launch.py

  # Terminal 2: Run this test script
  cd ~/dev/r2d2/ros2_ws/src/r2d2_audio
  python3 test_audio_notification.py
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time
from typing import List, Tuple


class AudioNotificationTestNode(Node):
    """Test node that publishes fake recognition results."""
    
    def __init__(self):
        super().__init__('audio_notification_test_node')
        
        # Create publisher for person_id topic
        self.person_pub = self.create_publisher(
            String,
            '/r2d2/perception/person_id',
            qos_profile=rclpy.qos.QoSProfile(depth=10)
        )
        
        self.get_logger().info("Audio Notification Test Node ready")
        self.get_logger().info("Publishing fake recognition results to /r2d2/perception/person_id")
    
    def publish_person_id(self, person_id: str):
        """Publish a person_id message."""
        msg = String()
        msg.data = person_id
        self.person_pub.publish(msg)
        self.get_logger().info(f"Published: {person_id}")


def run_test_scenario():
    """Run a test scenario with face recognition events."""
    
    rclpy.init()
    node = AudioNotificationTestNode()
    
    # Test scenario: different face recognition events
    scenario: List[Tuple[str, float, str]] = [
        # (person_id, duration, description)
        ("unknown", 2.0, "Unknown person in frame"),
        ("severin", 3.0, "SEVERIN RECOGNIZED! 🎉"),
        ("severin", 2.0, "Still recognized..."),
        ("unknown", 2.0, "Lost Severin"),
        ("unknown", 2.0, "Someone else in frame"),
        ("severin", 3.0, "SEVERIN BACK! 🎉"),
        ("severin", 1.5, "Brief continuity..."),
    ]
    
    print("\n" + "="*70)
    print("R2D2 AUDIO NOTIFICATION TEST SCENARIO")
    print("="*70)
    print("\nTest timeline:")
    print("-" * 70)
    
    for person_id, duration, description in scenario:
        print(f"\n📢 {description}")
        print(f"   Publishing: {person_id}")
        
        node.publish_person_id(person_id)
        
        # Wait for the specified duration
        for i in range(int(duration * 10)):
            rclpy.spin_once(node, timeout_sec=0.1)
            time.sleep(0.1)
    
    print("\n" + "="*70)
    print("TEST COMPLETE")
    print("="*70)
    print("\nExpected behavior:")
    print("  ✓ First 'severin' → BEEP! (transition from unknown)")
    print("  ✓ Second 'severin' → No beep (still recognized)")
    print("  ✓ Back to 'unknown' → No beep (loss event)")
    print("  ✓ Third 'severin' → BEEP! (transition from unknown)")
    print("\nTotal beeps expected: 2")
    print("="*70)
    
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    try:
        run_test_scenario()
    except KeyboardInterrupt:
        print("\n\nTest interrupted by user")
    except Exception as e:
        print(f"\n\nTest failed with error: {e}")
