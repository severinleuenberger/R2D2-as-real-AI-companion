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
from std_msgs.msg import String, Int32
import subprocess
import time
from pathlib import Path
from typing import Optional
from dataclasses import dataclass, asdict
import json

# #region agent log
DEBUG_LOG_PATH = '/home/severin/.cursor/debug.log'
def _debug_log(location, message, data, hypothesis_id=None):
    try:
        with open(DEBUG_LOG_PATH, 'a') as f:
            import json
            log_entry = {
                'timestamp': int(time.time() * 1000),
                'location': location,
                'message': message,
                'data': data,
                'sessionId': 'debug-session',
                'runId': 'run1'
            }
            if hypothesis_id:
                log_entry['hypothesisId'] = hypothesis_id
            f.write(json.dumps(log_entry) + '\n')
    except:
        pass
# #endregion


# Simple PersonStatus data class (alternative to ROS message)
@dataclass
class PersonStatusData:
    """Person recognition status data."""
    status: str  # "red" | "blue" | "green"
    person_identity: str  # "severin" | "unknown" | "no_person"
    timestamp_sec: int  # ROS 2 time seconds
    timestamp_nanosec: int  # ROS 2 time nanoseconds
    confidence: float  # 0.0-1.0
    duration_in_state: float  # seconds
    
    def to_json(self) -> str:
        """Convert to JSON string for publishing."""
        return json.dumps(asdict(self))


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
        self.declare_parameter('audio_volume', 0.3)       # 0.0-1.0 (audio file volume) - 30% volume
        self.declare_parameter('alsa_device', 'hw:1,0')    # ALSA device for audio output (e.g., hw:1,0)
        self.declare_parameter('red_status_timeout_seconds', 15.0)  # Simple 15s timeout, resets on recognition
        self.declare_parameter('cooldown_seconds', 2.0)   # Min between recognition alerts
        self.declare_parameter('recognition_cooldown_after_loss_seconds', 5.0)  # Quiet period after loss alert
        self.declare_parameter('recognition_audio_file', 'Voicy_R2-D2 - 2.mp3')  # Recognition alert audio
        self.declare_parameter('loss_audio_file', 'Voicy_R2-D2 - 5.mp3')  # Loss alert audio
        self.declare_parameter('enabled', True)
        
        # Get parameters
        self.target_person = self.get_parameter('target_person').value
        self.audio_volume = self.get_parameter('audio_volume').value
        self.alsa_device = self.get_parameter('alsa_device').value
        self.red_status_timeout = self.get_parameter('red_status_timeout_seconds').value
        self.cooldown_seconds = self.get_parameter('cooldown_seconds').value
        self.recognition_cooldown_after_loss = self.get_parameter('recognition_cooldown_after_loss_seconds').value
        recognition_audio_filename = self.get_parameter('recognition_audio_file').value
        loss_audio_filename = self.get_parameter('loss_audio_file').value
        self.enabled = self.get_parameter('enabled').value
        
        # Simplified state tracking - 15s reset timer
        self.last_recognition_time: Optional[float] = None  # Last time saw target person (resets 15s timer)
        self.last_recognition_beep_time: Optional[float] = None  # Cooldown for recognition beeps
        self.last_loss_beep_time: Optional[float] = None  # Cooldown for loss beeps
        
        # Status tracking (for LED, STT-LLM-TTS, database)
        self.current_status = "blue"  # "red" (recognized) | "blue" (lost) | "green" (unknown)
        self.current_person = "no_person"  # "severin" | "unknown" | "no_person"
        self.status_changed_time = time.time()
        self.unknown_person_detected = False
        self.last_known_state = "unknown"
        
        # Track face count to detect when no faces are visible
        self.last_face_count = None
        self.last_face_count_time = None
        
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
        
        # Create subscription to face_count topic to detect when no faces are visible
        self.face_count_sub = self.create_subscription(
            Int32,
            '/r2d2/perception/face_count',
            self.face_count_callback,
            qos_profile=rclpy.qos.QoSProfile(depth=10)
        )
        
        # Create subscription to face_count topic to detect when no faces are visible
        self.face_count_sub = self.create_subscription(
            Int32,
            '/r2d2/perception/face_count',
            self.face_count_callback,
            qos_profile=rclpy.qos.QoSProfile(depth=10)
        )
        
        # Track last face count to detect transitions
        self.last_face_count = None
        self.last_face_count_time = None
        
        # Create publisher for notification events (for debugging/monitoring)
        self.event_pub = self.create_publisher(
            String,
            '/r2d2/audio/notification_event',
            qos_profile=rclpy.qos.QoSProfile(depth=10)
        )
        
        # Create publisher for person status (for LED, STT-LLM-TTS, database logging)
        self.status_pub = self.create_publisher(
            String,
            '/r2d2/audio/person_status',
            qos_profile=rclpy.qos.QoSProfile(depth=10)
        )
        
        # Create timer for loss detection (check every 500ms)
        self.create_timer(0.5, self.check_loss_state)
        
        # Create timer for regular status publishing (10 Hz = every 100ms)
        # This ensures web page and other subscribers get regular updates even when person_id topic stops publishing
        self.create_timer(0.1, self.publish_current_status)
        
        self.get_logger().info(
            f"Audio Notification Node initialized (Audio Files):\n"
            f"  Target person: {self.target_person}\n"
            f"  Recognition audio: {self.recognition_audio.name}\n"
            f"  Loss audio: {self.loss_audio.name}\n"
            f"  Audio volume: {self.audio_volume*100:.0f}%\n"
            f"  ALSA device: {self.alsa_device}\n"
            f"  RED status timeout: {self.red_status_timeout}s (simple timer, resets on recognition)\n"
            f"  Alert cooldown: {self.cooldown_seconds}s (min between alerts)\n"
            f"  Recognition cooldown after loss: {self.recognition_cooldown_after_loss}s (quiet period after loss alert)\n"
            f"  Enabled: {self.enabled}\n"
            f"  Audio player: {self.audio_player_path}"
        )
    
    def person_callback(self, msg: String):
        """
        Handle person_id messages - SIMPLIFIED 15-second timeout logic.
        
        Rule: If target_person recognized, reset 15s timer.
        If timer expires (15s without recognition), switch to BLUE.
        """
        person_id = msg.data
        current_time = time.time()
        
        if not self.enabled:
            return
        
        if person_id == self.target_person:
            # Target person recognized - RESET 15s countdown timer
            was_red = (self.current_status == "red")
            self.last_recognition_time = current_time  # ← KEY: Reset timer
            
            if not was_red:
                # Transition to RED
                old_status = self.current_status
                self.current_status = "red"
                self.current_person = self.target_person
                self.status_changed_time = current_time
                self.unknown_person_detected = False
                
                # Publish status FIRST
                self._publish_status("red", self.target_person, confidence=0.95)
                
                # Play "Hello!" beep (with cooldown)
                if self.last_recognition_beep_time is None or \
                   (current_time - self.last_recognition_beep_time) >= self.cooldown_seconds:
                    self._play_audio_file(self.recognition_audio, alert_type="RECOGNITION")
                    self.last_recognition_beep_time = current_time
                    self._publish_event(f"🎉 Recognized {self.target_person}!")
                
                self.get_logger().info(f"✓ {self.target_person} recognized ({old_status} → RED)")
            else:
                # Already RED - just reset timer and update duration
                self._publish_status("red", self.target_person, confidence=0.95)
                self.get_logger().debug("RED: Timer reset (target_person seen)")
        
        elif person_id == "unknown":
            # Unknown person detected → GREEN status
            if self.current_status != "green":
                self.current_status = "green"
                self.current_person = "unknown"
                self.status_changed_time = current_time
                self.unknown_person_detected = True
                self._publish_status("green", "unknown", confidence=0.70)
                self.get_logger().info("🟢 Unknown person detected")
            else:
                self._publish_status("green", "unknown", confidence=0.70)
    
    def face_count_callback(self, msg: Int32):
        """
        Handle face_count messages to detect when no faces are visible.
        When face_count == 0 and we're in GREEN state, transition to BLUE.
        """
        face_count = msg.data
        current_time = time.time()
        
        # #region agent log
        _debug_log('audio_notification_node.py:335', 'face_count_callback', {
            'face_count': face_count,
            'current_status': self.current_status,
            'current_person': self.current_person,
            'unknown_person_detected': self.unknown_person_detected
        }, 'H')
        # #endregion
        
        if not self.enabled:
            return
        
        self.last_face_count = face_count
        self.last_face_count_time = current_time
        
        # If no faces are detected (face_count == 0)
        if face_count == 0:
            # CRITICAL: When no faces are visible, we should be in BLUE state (no person)
            # This handles the case where the status incorrectly shows GREEN when no one is visible
            
            if self.current_status == "green":
                # If we're in GREEN (unknown person), immediately transition to BLUE (no person)
                # This fixes the issue where status stays GREEN when no one is visible
                self.current_status = "blue"
                self.current_person = "no_person"
                self.status_changed_time = current_time
                self.unknown_person_detected = False
                
                # #region agent log
                _debug_log('audio_notification_node.py:371', 'No faces detected - transitioning GREEN to BLUE', {
                    'previous_status': 'green',
                    'new_status': 'blue'
                }, 'I')
                # #endregion
                
                self._publish_status("blue", "no_person", confidence=0.0)
                self.get_logger().info("🔵 No faces detected - transitioned from GREEN to BLUE")
            
            # If we're in RED state (target person recognized) and face_count == 0,
            # the timer will handle the transition to BLUE after the loss confirmation period (~20 seconds)
            # This allows for jitter tolerance (brief interruptions are ignored)
            
        # If faces are detected (face_count > 0), the person_id callback will handle
        # the state transitions (RED for target, GREEN for unknown)
    
    def check_loss_state(self):
        """
        Timer callback: SIMPLIFIED 15-second timeout check.
        
        If in RED state and no target_person recognized for 15 seconds, switch to BLUE.
        Runs every 500ms to check timeout.
        """
        if not self.enabled:
            return
        
        if self.last_recognition_time is None:
            return  # Never recognized yet, nothing to check
        
        if self.current_status != "red":
            return  # Not in RED state, nothing to timeout
        
        current_time = time.time()
        time_since_recognition = current_time - self.last_recognition_time
        
        if time_since_recognition > self.red_status_timeout:  # 15 seconds
            # Timer expired - switch to BLUE
            self.current_status = "blue"
            self.current_person = "no_person"
            self.status_changed_time = current_time
            self.unknown_person_detected = False
            
            # Publish BLUE status
            self._publish_status("blue", "no_person", confidence=0.0)
            
            # Play "Lost you!" beep (with cooldown)
            if self.last_loss_beep_time is None or \
               (current_time - self.last_loss_beep_time) >= self.cooldown_seconds:
                self._play_audio_file(self.loss_audio, alert_type="LOSS")
                self.last_loss_beep_time = current_time
                self._publish_event(f"❌ {self.target_person} lost (no recognition for {time_since_recognition:.1f}s)")
            
            self.get_logger().info(
                f"✗ {self.target_person} lost (timer expired: {time_since_recognition:.1f}s > {self.red_status_timeout}s)"
            )
    
    def publish_current_status(self):
        """
        Timer callback: Regularly publish current status (10 Hz) so web page and other subscribers
        get updates even when person_id topic stops publishing.
        """
        if not self.enabled:
            return
        
        # Map current state to confidence values
        confidence_map = {
            "red": 0.95,
            "green": 0.70,
            "blue": 0.0
        }
        
        confidence = confidence_map.get(self.current_status, 0.0)
        self._publish_status(self.current_status, self.current_person, confidence)
    
    def _publish_status(self, status: str, person_identity: str, confidence: float):
        """
        Publish current recognition status for LED, STT-LLM-TTS, and database logging.
        
        Publishes as JSON String message to /r2d2/audio/person_status
        
        Args:
            status: "red" (recognized) | "blue" (lost) | "green" (unknown)
            person_identity: "severin" | "unknown" | "no_person"
            confidence: 0.0-1.0 detection confidence
        """
        current_time = time.time()
        duration = current_time - self.status_changed_time
        
        # Create status data
        ros_now = self.get_clock().now()
        status_data = PersonStatusData(
            status=status,
            person_identity=person_identity,
            timestamp_sec=ros_now.seconds_nanoseconds()[0],
            timestamp_nanosec=ros_now.seconds_nanoseconds()[1],
            confidence=confidence,
            duration_in_state=duration
        )
        
        # Publish as JSON string
        msg = String()
        msg.data = status_data.to_json()
        self.status_pub.publish(msg)
        
        # #region agent log
        _debug_log('audio_notification_node.py:420', 'Status published', {
            'status': status,
            'person_identity': person_identity,
            'confidence': confidence,
            'duration': duration,
            'last_recognition_time': self.last_recognition_time
        }, 'G')
        # #endregion
        
        self.get_logger().debug(
            f"📊 Status: {status.upper()} | Person: {person_identity} | Duration: {duration:.1f}s"
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
                str(self.alsa_device),
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
