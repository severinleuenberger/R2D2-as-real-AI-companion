#!/usr/bin/env python3
"""
R2D2 Audio Notification Node - Enhanced Recognition State Management

Subscribes to face recognition results and triggers MP3 audio alerts when
specific people are recognized or confirmed lost.

Subscribes to:
  - /r2d2/perception/person_id (String): Person name or "unknown"

Publishes:
  - /r2d2/audio/notification_event (String): Event descriptions for debugging

Features:
  - MP3 audio alert when target person is recognized (transition "Hello!")
  - Tolerates brief gaps (< 5 seconds) in recognition (jitter tolerance)
  - MP3 audio alert when person confirmed lost (> 5s jitter + 15s confirmation "Oh, I lost you!")
  - Fully parameterizable audio files, timing, and volume
  - Background service ready
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import subprocess
import time
from pathlib import Path
from typing import Optional
import sys
import os


class AudioNotificationNode(Node):
    """
    ROS 2 node for MP3 audio notifications based on face recognition.
    
    State machine:
    - Tolerates brief recognition gaps (< 5 seconds jitter tolerance)
    - Triggers loss alert after jitter exceeded + 15 second confirmation window
    - Plays parameterizable MP3 audio files for recognition and loss events
    """
    
    def __init__(self):
        super().__init__('audio_notification_node')
        
        self.get_logger().info("R2D2 Audio Notification Node starting...")
        
        # Declare parameters
        self.declare_parameter('target_person', 'severin')
        self.declare_parameter('audio_volume', 0.05)       # 0.0-1.0 (audio file volume)
        self.declare_parameter('jitter_tolerance_seconds', 5.0)  # Brief gap tolerance
        self.declare_parameter('loss_confirmation_seconds', 15.0) # Loss confirmation window duration
        self.declare_parameter('cooldown_seconds', 2.0)   # Min between recognition alerts
        self.declare_parameter('recognition_audio_file', 'Voicy_R2-D2 - 2.mp3')  # Recognition alert audio
        self.declare_parameter('loss_audio_file', 'Voicy_R2-D2 - 5.mp3')  # Loss alert audio
        self.declare_parameter('enabled', True)
        
        # Get parameters
        self.target_person = self.get_parameter('target_person').value
        self.audio_volume = self.get_parameter('audio_volume').value
        self.jitter_tolerance = self.get_parameter('jitter_tolerance_seconds').value
        self.loss_confirmation = self.get_parameter('loss_confirmation_seconds').value
        self.cooldown_seconds = self.get_parameter('cooldown_seconds').value
        recognition_audio_filename = self.get_parameter('recognition_audio_file').value
        loss_audio_filename = self.get_parameter('loss_audio_file').value
        self.enabled = self.get_parameter('enabled').value
        
        # Enhanced state tracking
        self.is_currently_recognized = False  # True if logically "recognized" (tolerates brief gaps)
        self.last_recognition_time: Optional[float] = None  # Last time saw target person
        self.loss_jitter_exceeded_time: Optional[float] = None  # When jitter tolerance was first exceeded
        self.last_recognition_beep_time: Optional[float] = None  # Cooldown for recognition beeps
        self.last_known_state = "unknown"  # Track last known state
        
        # Find audio player and audio files
        # Get the directory where this script is located
        script_dir = Path(__file__).parent
        
        # Try multiple paths for audio assets (development and installed)
        audio_paths = [
            script_dir / 'assets' / 'audio',  # Development path
            Path('/home/severin/dev/r2d2/ros2_ws/install/r2d2_audio/share/r2d2_audio/assets/audio'),  # Install path
        ]
        
        self.audio_assets_dir = None
        for path in audio_paths:
            if path.exists():
                self.audio_assets_dir = path
                break
        
        if self.audio_assets_dir is None:
            self.get_logger().warn(
                f"Audio assets directory not found. Tried: {', '.join(str(p) for p in audio_paths)}"
            )
            self.audio_assets_dir = audio_paths[0]  # Use first path as fallback
        
        self.recognition_audio = self.audio_assets_dir / recognition_audio_filename
        self.loss_audio = self.audio_assets_dir / loss_audio_filename
        self.audio_player_path = script_dir / 'audio_player.py'
        
        if not self.audio_assets_dir.exists():
            self.get_logger().warn(
                f"Audio assets directory not found at {self.audio_assets_dir}"
            )
        
        if not self.recognition_audio.exists():
            self.get_logger().warn(
                f"Recognition audio file not found at {self.recognition_audio}"
            )
        
        if not self.loss_audio.exists():
            self.get_logger().warn(
                f"Loss audio file not found at {self.loss_audio}"
            )
        
        if not self.audio_player_path.exists():
            self.get_logger().warn(
                f"Audio player script not found at {self.audio_player_path}"
            )
        
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
            f"Audio Notification Node initialized (Audio Files):\n"
            f"  Target person: {self.target_person}\n"
            f"  Recognition audio: {self.recognition_audio.name}\n"
            f"  Loss audio: {self.loss_audio.name}\n"
            f"  Audio volume: {self.audio_volume*100:.0f}%\n"
            f"  Jitter tolerance: {self.jitter_tolerance}s (brief gap tolerance)\n"
            f"  Loss confirmation: {self.loss_confirmation}s (confirmation window after jitter)\n"
            f"  Alert cooldown: {self.cooldown_seconds}s\n"
            f"  Enabled: {self.enabled}\n"
            f"  Audio player: {self.audio_player_path}"
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
                self.loss_jitter_exceeded_time = None  # Reset loss timer on re-recognition
                self.last_known_state = self.target_person
                self._trigger_recognition_alert()
                self._publish_event(f"🎉 Recognized {self.target_person}!")
                self.get_logger().info(f"✓ {self.target_person} recognized!")
        else:
            # Target person not detected, but don't immediately mark as lost
            # Let the timer handle the loss detection with jitter tolerance
            pass
    
    def check_loss_state(self):
        """
        Timer callback: Check if person has been continuously lost for > loss_confirmation seconds.
        
        Timing logic:
        1. Jitter tolerance window (0 to 5s absence): No action - brief gap tolerance
        2. Loss confirmation window (5s to 20s absence): Monitoring for confirmed loss
        3. At 20s+ continuous absence: Fire loss alert
        
        Resets when person is re-recognized.
        """
        if not self.enabled or not self.is_currently_recognized:
            return
        
        current_time = time.time()
        time_since_last_recognition = current_time - (self.last_recognition_time or current_time)
        
        # STEP 1: Check if jitter tolerance window has been exceeded (5 seconds)
        if time_since_last_recognition > self.jitter_tolerance:
            # Person has been absent for > 5 seconds
            
            # Track when jitter tolerance was first exceeded
            if self.loss_jitter_exceeded_time is None:
                self.loss_jitter_exceeded_time = current_time
            
            # STEP 2: Check if loss confirmation window has been met
            # (15 seconds of continuous absence AFTER jitter tolerance exceeded)
            time_in_loss_window = current_time - self.loss_jitter_exceeded_time
            
            if time_in_loss_window > self.loss_confirmation:
                # Confirmed loss: Jitter exceeded + 15 seconds confirmed
                self.is_currently_recognized = False
                self.loss_jitter_exceeded_time = None  # Reset for next cycle
                self._trigger_loss_alert()
                self._publish_event(f"❌ {self.target_person} lost (confirmed after {time_since_last_recognition:.1f}s absence)")
                self.get_logger().info(
                    f"✗ {self.target_person} lost (after {time_since_last_recognition:.1f}s absence, "
                    f"{time_in_loss_window:.1f}s in loss window)"
                )
    
    def _trigger_recognition_alert(self):
        """
        Play recognition audio when target person is recognized (transition).
        Respects cooldown to prevent spam.
        """
        current_time = time.time()
        
        # Check cooldown
        if self.last_recognition_beep_time is not None:
            time_since_last_alert = current_time - self.last_recognition_beep_time
            if time_since_last_alert < self.cooldown_seconds:
                self.get_logger().debug(
                    f"Recognition alert suppressed (cooldown: {time_since_last_alert:.1f}s "
                    f"< {self.cooldown_seconds}s)"
                )
                return
        
        self._play_audio_file(
            audio_file=self.recognition_audio,
            alert_type="RECOGNITION"
        )
        self.last_recognition_beep_time = current_time
    
    def _trigger_loss_alert(self):
        """
        Play loss audio when person is confirmed lost.
        """
        self._play_audio_file(
            audio_file=self.loss_audio,
            alert_type="LOSS"
        )
    
    def _play_audio_file(self, audio_file: Path, alert_type: str = "GENERIC"):
        """
        Play an audio file using the audio player.
        
        Args:
            audio_file: Path to the audio file to play
            alert_type: Type name for logging (RECOGNITION, LOSS, etc.)
        """
        if not audio_file.exists():
            self.get_logger().error(f"Audio file not found: {audio_file}")
            return
        
        if not self.audio_player_path.exists():
            self.get_logger().error(f"Audio player not found: {self.audio_player_path}")
            return
        
        try:
            self.get_logger().info(
                f"🔊 Playing {alert_type} audio: {audio_file.name} (volume {self.audio_volume*100:.0f}%)"
            )
            
            # Run audio player asynchronously (non-blocking)
            cmd = [
                'python3',
                str(self.audio_player_path),
                str(audio_file),
                str(self.audio_volume),
            ]
            
            subprocess.Popen(
                cmd,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                start_new_session=True
            )
            
        except Exception as e:
            self.get_logger().error(f"Error playing {alert_type} audio: {e}")
    
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
