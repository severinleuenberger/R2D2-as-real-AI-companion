#!/usr/bin/env python3
"""
R2D2 Audio Notification Node - Enhanced Recognition State Management

Subscribes to face recognition results and triggers audio alerts when
specific people are recognized.

Subscribes to:
  - /r2d2/perception/person_id (String): Person name or "unknown"

Publishes:
  - /r2d2/audio/notification_event (String): Event descriptions for debugging

Features:
  - Single beep when target person is recognized (transition)
  - Tolerates brief gaps (< 5 seconds) in recognition (jitter tolerance)
  - Double beep when person is continuously lost for > 5 seconds
  - Configurable parameters
  - Background service ready
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import subprocess
import time
from pathlib import Path
from typing import Optional


class AudioNotificationNode(Node):
    """
    ROS 2 node for audio notifications based on face recognition.
    Enhanced with improved state tracking:
    - Tolerates brief recognition gaps (< 5 seconds)
    - Triggers double beep on confirmed loss (> 5 seconds)
    """
    
    def __init__(self):
        super().__init__('audio_notification_node')
        
        self.get_logger().info("R2D2 Audio Notification Node starting...")
        
        # Declare parameters
        self.declare_parameter('target_person', 'severin')
        self.declare_parameter('beep_frequency', 400.0)  # Hz (deep)
        self.declare_parameter('beep_duration', 0.5)      # seconds
        self.declare_parameter('beep_volume', 0.25)       # 0.0-1.0 (quieter)
        self.declare_parameter('loss_beep_frequency', 400.0)  # Hz (same deep tone)
        self.declare_parameter('loss_beep_duration', 0.3)     # seconds
        self.declare_parameter('jitter_tolerance_seconds', 5.0)  # Brief gap tolerance
        self.declare_parameter('loss_confirmation_seconds', 5.0) # Loss confirmation time
        self.declare_parameter('cooldown_seconds', 2.0)   # Min between recognition beeps
        self.declare_parameter('enabled', True)
        
        # Get parameters
        self.target_person = self.get_parameter('target_person').value
        self.beep_frequency = self.get_parameter('beep_frequency').value
        self.beep_duration = self.get_parameter('beep_duration').value
        self.beep_volume = self.get_parameter('beep_volume').value
        self.loss_beep_frequency = self.get_parameter('loss_beep_frequency').value
        self.loss_beep_duration = self.get_parameter('loss_beep_duration').value
        self.jitter_tolerance = self.get_parameter('jitter_tolerance_seconds').value
        self.loss_confirmation = self.get_parameter('loss_confirmation_seconds').value
        self.cooldown_seconds = self.get_parameter('cooldown_seconds').value
        self.enabled = self.get_parameter('enabled').value
        
        # Enhanced state tracking
        self.is_currently_recognized = False  # True if logically "recognized" (tolerates brief gaps)
        self.last_recognition_time: Optional[float] = None  # Last time saw target person
        self.last_loss_notification_time: Optional[float] = None  # Last time notified of loss
        self.last_recognition_beep_time: Optional[float] = None  # Cooldown for recognition beeps
        self.last_known_state = "unknown"  # Track last known state
        
        # Find audio_beep.py
        self.audio_beep_path = Path.home() / 'dev' / 'r2d2' / 'audio_beep.py'
        if not self.audio_beep_path.exists():
            self.get_logger().warn(
                f"audio_beep.py not found at {self.audio_beep_path}. "
                "Notifications will be disabled."
            )
            self.audio_beep_path = None
        
        # Create subscription to person_id topic
        self.person_sub = self.create_subscription(
            String,
            '/r2d2/perception/person_id',
            self.person_callback,
            qos_profile=rclpy.qos.QoSProfile(depth=10)
        )
        
        # Create publisher for notification events (for debugging/monitoring)
        self.event_pub = self.create_publisher(
            String,
            '/r2d2/audio/notification_event',
            qos_profile=rclpy.qos.QoSProfile(depth=10)
        )
        
        # Create timer for loss detection (check every 500ms)
        self.create_timer(0.5, self.check_loss_state)
        
        self.get_logger().info(
            f"Audio Notification Node initialized (Enhanced):\n"
            f"  Target person: {self.target_person}\n"
            f"  Recognition beep: {self.beep_frequency} Hz, {self.beep_duration}s, "
            f"volume {self.beep_volume*100:.0f}%\n"
            f"  Loss beep: {self.loss_beep_frequency} Hz, 2x{self.loss_beep_duration}s\n"
            f"  Jitter tolerance: {self.jitter_tolerance}s (brief gap tolerance)\n"
            f"  Loss confirmation: {self.loss_confirmation}s (before loss beep)\n"
            f"  Recognition cooldown: {self.cooldown_seconds}s\n"
            f"  Enabled: {self.enabled}\n"
            f"  Audio utility: {self.audio_beep_path}"
        )
    
    def person_callback(self, msg: String):
        """
        Handle person_id messages from face recognition.
        
        Args:
            msg: String message containing person name or "unknown"
        """
        person_id = msg.data
        current_time = time.time()
        
        if not self.enabled:
            return
        
        is_target_recognized = (person_id == self.target_person)
        
        if is_target_recognized:
            # Target person detected
            self.last_recognition_time = current_time
            
            if not self.is_currently_recognized:
                # Transition: unknown -> recognized
                self.is_currently_recognized = True
                self.last_known_state = self.target_person
                self._trigger_recognition_beep()
                self._publish_event(f"🎉 Recognized {self.target_person}!")
                self.get_logger().info(f"✓ {self.target_person} recognized!")
        else:
            # Target person not detected, but don't immediately mark as lost
            # Let the timer handle the loss detection with jitter tolerance
            pass
    
    def check_loss_state(self):
        """
        Timer callback: Check if person has been continuously lost for > loss_confirmation seconds.
        Handles jitter tolerance for brief recognition gaps.
        """
        if not self.enabled or not self.is_currently_recognized:
            return
        
        current_time = time.time()
        time_since_last_recognition = current_time - (self.last_recognition_time or current_time)
        
        # If person not seen for longer than jitter tolerance
        if time_since_last_recognition > self.jitter_tolerance:
            # Check if we should notify about loss
            if self.last_loss_notification_time is None or \
               (current_time - self.last_loss_notification_time) > self.loss_confirmation:
                
                # Person was recognized but now confirmed lost
                self.is_currently_recognized = False
                self.last_loss_notification_time = current_time
                self._trigger_loss_beep()
                self._publish_event(f"❌ {self.target_person} lost (confirmed)")
                self.get_logger().info(
                    f"✗ {self.target_person} lost after {time_since_last_recognition:.1f}s"
                )
    
    def _trigger_recognition_beep(self):
        """
        Trigger single beep when target person is recognized (transition).
        Respects cooldown to prevent spam.
        """
        current_time = time.time()
        
        # Check cooldown
        if self.last_recognition_beep_time is not None:
            time_since_last_beep = current_time - self.last_recognition_beep_time
            if time_since_last_beep < self.cooldown_seconds:
                self.get_logger().debug(
                    f"Recognition beep suppressed (cooldown: {time_since_last_beep:.1f}s "
                    f"< {self.cooldown_seconds}s)"
                )
                return
        
        self._play_beep(
            frequency=self.beep_frequency,
            duration=self.beep_duration,
            volume=self.beep_volume,
            beep_type="RECOGNITION"
        )
        self.last_recognition_beep_time = current_time
    
    def _trigger_loss_beep(self):
        """
        Trigger double beep (loss alert) when person is confirmed lost.
        Two rapid beeps at lower frequency to indicate loss.
        """
        if self.audio_beep_path is None:
            self.get_logger().warn("Cannot play loss beep: audio_beep.py not found")
            return
        
        try:
            self.get_logger().info(
                f"🔔🔔 LOSS ALERT! {self.target_person} lost "
                f"({self.loss_beep_frequency}Hz, 2x{self.loss_beep_duration}s)"
            )
            
            # Play two beeps
            for beep_num in range(2):
                cmd = [
                    'python3',
                    str(self.audio_beep_path),
                    '--frequency', str(self.loss_beep_frequency),
                    '--duration', str(self.loss_beep_duration),
                    '--volume', str(self.beep_volume),
                ]
                
                subprocess.run(
                    cmd,
                    timeout=self.loss_beep_duration + 2,
                    capture_output=True,
                    cwd=str(Path.home() / 'dev' / 'r2d2')
                )
                
                # Small gap between beeps
                if beep_num == 0:
                    time.sleep(0.2)
            
        except subprocess.TimeoutExpired:
            self.get_logger().warn("Loss beep timed out")
        except Exception as e:
            self.get_logger().error(f"Error playing loss beep: {e}")
    
    def _play_beep(self, frequency: float, duration: float, volume: float, beep_type: str = "GENERIC"):
        """
        Generic beep player.
        
        Args:
            frequency: Frequency in Hz
            duration: Duration in seconds
            volume: Volume 0.0-1.0
            beep_type: Type name for logging (RECOGNITION, LOSS, etc.)
        """
        if self.audio_beep_path is None:
            self.get_logger().warn(f"Cannot play {beep_type} beep: audio_beep.py not found")
            return
        
        try:
            # Run audio_beep.py with specified parameters
            cmd = [
                'python3',
                str(self.audio_beep_path),
                '--frequency', str(frequency),
                '--duration', str(duration),
                '--volume', str(volume),
            ]
            
            subprocess.run(
                cmd,
                timeout=duration + 2,
                capture_output=True,
                cwd=str(Path.home() / 'dev' / 'r2d2')
            )
            
        except subprocess.TimeoutExpired:
            self.get_logger().warn(f"{beep_type} beep timed out")
        except Exception as e:
            self.get_logger().error(f"Error playing {beep_type} beep: {e}")
    
    def _publish_event(self, event_description: str):
        """
        Publish a notification event for debugging/monitoring.
        
        Args:
            event_description: Description of the event
        """
        msg = String()
        msg.data = event_description
        self.event_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = AudioNotificationNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Audio Notification Node shutting down...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
