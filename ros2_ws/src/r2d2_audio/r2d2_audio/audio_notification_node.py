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
        self.declare_parameter('audio_volume', 0.05)       # 0.0-1.0 (audio file volume) - 5% volume (subtle)
        self.declare_parameter('alsa_device', 'hw:1,0')    # ALSA device for audio output (e.g., hw:1,0)
        self.declare_parameter('jitter_tolerance_seconds', 5.0)  # Brief gap tolerance
        self.declare_parameter('loss_confirmation_seconds', 15.0) # Loss confirmation window duration
        self.declare_parameter('cooldown_seconds', 2.0)   # Min between recognition alerts
        self.declare_parameter('recognition_cooldown_after_loss_seconds', 5.0)  # Quiet period after loss alert
        self.declare_parameter('recognition_audio_file', 'Voicy_R2-D2 - 2.mp3')  # Recognition alert audio
        self.declare_parameter('loss_audio_file', 'Voicy_R2-D2 - 5.mp3')  # Loss alert audio
        self.declare_parameter('enabled', True)
        
        # Get parameters
        self.target_person = self.get_parameter('target_person').value
        self.audio_volume = self.get_parameter('audio_volume').value
        self.alsa_device = self.get_parameter('alsa_device').value
        self.jitter_tolerance = self.get_parameter('jitter_tolerance_seconds').value
        self.loss_confirmation = self.get_parameter('loss_confirmation_seconds').value
        self.cooldown_seconds = self.get_parameter('cooldown_seconds').value
        self.recognition_cooldown_after_loss = self.get_parameter('recognition_cooldown_after_loss_seconds').value
        recognition_audio_filename = self.get_parameter('recognition_audio_file').value
        loss_audio_filename = self.get_parameter('loss_audio_file').value
        self.enabled = self.get_parameter('enabled').value
        
        # Enhanced state tracking
        self.is_currently_recognized = False  # True if logically "recognized" (tolerates brief gaps)
        self.last_recognition_time: Optional[float] = None  # Last time saw target person
        self.loss_jitter_exceeded_time: Optional[float] = None  # When jitter tolerance was first exceeded
        self.last_recognition_beep_time: Optional[float] = None  # Cooldown for recognition beeps
        self.last_loss_beep_time: Optional[float] = None  # Cooldown for loss beeps (after 15s confirmation)
        self.loss_alert_time: Optional[float] = None  # When loss alert was triggered (for quiet period)
        
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
            f"  Jitter tolerance: {self.jitter_tolerance}s (brief gap tolerance)\n"
            f"  Loss confirmation: {self.loss_confirmation}s (confirmation window after jitter)\n"
            f"  Alert cooldown: {self.cooldown_seconds}s (min between alerts)\n"
            f"  Recognition cooldown after loss: {self.recognition_cooldown_after_loss}s (quiet period after loss alert)\n"
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
        
        # #region agent log
        _debug_log('audio_notification_node.py:196', 'person_callback called', {
            'person_id': person_id,
            'current_time': current_time,
            'is_currently_recognized': self.is_currently_recognized,
            'last_recognition_time': self.last_recognition_time,
            'current_status': self.current_status,
            'current_person': self.current_person
        }, 'A')
        # #endregion
        
        if not self.enabled:
            return
        
        is_target_recognized = (person_id == self.target_person)
        
        if is_target_recognized:
            # Target person detected
            self.last_recognition_time = current_time
            
            # Check if we're in the quiet period after a loss alert
            in_quiet_period = False
            if self.loss_alert_time is not None:
                time_since_loss_alert = current_time - self.loss_alert_time
                if time_since_loss_alert < self.recognition_cooldown_after_loss:
                    in_quiet_period = True
            
            if not self.is_currently_recognized:
                # Only transition to RECOGNIZED if NOT in quiet period
                if not in_quiet_period:
                    # Transition: LOST/UNKNOWN → RECOGNIZED (RED)
                    self.is_currently_recognized = True
                    self.loss_jitter_exceeded_time = None  # Reset loss timer on re-recognition
                    self.current_status = "red"
                    self.current_person = self.target_person
                    self.status_changed_time = current_time
                    self.unknown_person_detected = False
                    self.last_known_state = self.target_person
                    
                    # Publish status FIRST (so LED lights up immediately)
                    self._publish_status("red", self.target_person, confidence=0.95)
                    
                    # Then trigger beep
                    self._trigger_recognition_alert()
                    self._publish_event(f"🎉 Recognized {self.target_person}!")
                    self.get_logger().info(f"✓ {self.target_person} recognized!")
                else:
                    # In quiet period - stay in BLUE state but update last_recognition_time
                    # This way if person stays visible after quiet period, we'll transition to RED
                    self.get_logger().debug(
                        f"In quiet period ({self.recognition_cooldown_after_loss}s): "
                        f"suppressing recognition alert for {self.target_person}"
                    )
            else:
                # Already in RECOGNIZED state - just reset timer
                self.last_recognition_time = current_time
                # Publish status update (duration keeps updating)
                self._publish_status("red", self.target_person, confidence=0.95)
        
        elif person_id == "unknown":
            # Unknown person detected (not the target)
            # #region agent log
            _debug_log('audio_notification_node.py:279', 'Unknown person detected', {
                'was_recognized': self.is_currently_recognized,
                'previous_status': self.current_status,
                'last_recognition_time': self.last_recognition_time,
                'time_since_recognition': current_time - (self.last_recognition_time or current_time) if self.last_recognition_time else None
            }, 'B')
            # #endregion
            
            # CRITICAL: If we were in RED state, we need to handle transition properly
            # The timer will handle loss detection, but we should NOT reset is_currently_recognized here
            # because we want the timer to detect when we've been away long enough
            
            if not self.unknown_person_detected:
                self.unknown_person_detected = True
                self.current_status = "green"
                self.current_person = "unknown"
                self.status_changed_time = current_time
                
                # Publish status
                self._publish_status("green", "unknown", confidence=0.70)
                self.get_logger().info("🟢 Unknown person detected")
            else:
                # Update duration while unknown person present
                self._publish_status("green", "unknown", confidence=0.70)
        else:
            # #region agent log
            _debug_log('audio_notification_node.py:324', 'Unexpected person_id value', {
                'person_id': person_id,
                'current_status': self.current_status,
                'is_currently_recognized': self.is_currently_recognized
            }, 'F')
            # #endregion
    
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
            'is_currently_recognized': self.is_currently_recognized,
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
        Timer callback: Check if person has been continuously lost for > loss_confirmation seconds.
        
        Timing logic:
        1. Jitter tolerance window (0 to 5s absence): No action - brief gap tolerance
        2. Loss confirmation window (5s to 20s absence): Monitoring for confirmed loss
        3. At 20s+ continuous absence: Fire loss alert (with cooldown to prevent spam)
        
        Resets when person is re-recognized.
        """
        if not self.enabled or not self.is_currently_recognized:
            # #region agent log
            _debug_log('audio_notification_node.py:273', 'check_loss_state early return', {
                'enabled': self.enabled,
                'is_currently_recognized': self.is_currently_recognized
            }, 'C')
            # #endregion
            return
        
        current_time = time.time()
        time_since_last_recognition = current_time - (self.last_recognition_time or current_time)
        
        # #region agent log
        _debug_log('audio_notification_node.py:277', 'check_loss_state calculation', {
            'current_time': current_time,
            'last_recognition_time': self.last_recognition_time,
            'time_since_last_recognition': time_since_last_recognition,
            'jitter_tolerance': self.jitter_tolerance,
            'loss_confirmation': self.loss_confirmation,
            'loss_jitter_exceeded_time': self.loss_jitter_exceeded_time,
            'current_status': self.current_status,
            'current_person': self.current_person
        }, 'D')
        # #endregion
        
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
                # Check cooldown to prevent spam
                if self.last_loss_beep_time is None or \
                   (current_time - self.last_loss_beep_time) >= self.cooldown_seconds:
                    # #region agent log
                    _debug_log('audio_notification_node.py:294', 'Loss confirmed - transitioning to BLUE', {
                        'time_since_last_recognition': time_since_last_recognition,
                        'time_in_loss_window': time_in_loss_window,
                        'previous_status': self.current_status,
                        'previous_person': self.current_person
                    }, 'E')
                    # #endregion
                    
                    self.is_currently_recognized = False
                    self.loss_jitter_exceeded_time = None  # Reset for next cycle
                    self.unknown_person_detected = False
                    
                    # Record when loss alert fires (start of quiet period)
                    self.loss_alert_time = current_time
                    
                    # UPDATE STATUS FIRST (so LED lights up immediately)
                    self.current_status = "blue"
                    self.current_person = "no_person"
                    self.status_changed_time = current_time
                    self._publish_status("blue", "no_person", confidence=0.0)
                    
                    # THEN trigger beep
                    self._trigger_loss_alert()
                    self.last_loss_beep_time = current_time  # Set loss alert cooldown
                    self._publish_event(f"❌ {self.target_person} lost (confirmed after {time_since_last_recognition:.1f}s absence)")
                    self.get_logger().info(
                        f"✗ {self.target_person} lost (after {time_since_last_recognition:.1f}s absence, "
                        f"{time_in_loss_window:.1f}s in loss window)"
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
            'is_currently_recognized': self.is_currently_recognized,
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
            self.get_logger().debug(
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
