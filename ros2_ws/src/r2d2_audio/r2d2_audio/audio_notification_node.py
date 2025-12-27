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
from std_msgs.msg import String, Int32, Float32
import subprocess
import time
from pathlib import Path
from typing import Optional
from dataclasses import dataclass, asdict
import json
from r2d2_common.person_config import PersonConfig

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
        self.declare_parameter('target_person', 'target_person')  # Default: generic 'target_person' which resolves via registry
        self.declare_parameter('audio_volume', 0.02)       # 0.0-1.0 (audio file volume) - 30% volume (from config/audio_params.yaml)
        self.declare_parameter('alsa_device', 'hw:1,0')    # ALSA device for audio output (e.g., hw:1,0)
        self.declare_parameter('red_status_timeout_seconds', 15.0)  # Simple 15s timeout, resets on recognition
        self.declare_parameter('cooldown_seconds', 2.0)   # Min between recognition alerts
        self.declare_parameter('recognition_cooldown_after_loss_seconds', 5.0)  # Quiet period after loss alert
        self.declare_parameter('recognition_audio_file', 'Voicy_R2-D2 - 2.mp3')  # Recognition alert audio
        self.declare_parameter('loss_audio_file', 'Voicy_R2-D2 - 5.mp3')  # Loss alert audio
        self.declare_parameter('enabled', True)
        
        # RED-first architecture: Rolling window parameters
        self.declare_parameter('red_entry_match_threshold', 3)  # Matches required in 1s window
        self.declare_parameter('red_entry_window_seconds', 1.0)  # Rolling window duration
        
        # Get parameters
        target_person_param = self.get_parameter('target_person').value
        # Resolve person name using registry if 'target_person' is used
        self.target_person = PersonConfig.get_person_name(target_person_param)
        self.get_logger().info(f"Target person resolved to: {self.target_person}")

        self.audio_volume = self.get_parameter('audio_volume').value
        self.alsa_device = self.get_parameter('alsa_device').value
        self.red_status_timeout = self.get_parameter('red_status_timeout_seconds').value
        self.cooldown_seconds = self.get_parameter('cooldown_seconds').value
        self.recognition_cooldown_after_loss = self.get_parameter('recognition_cooldown_after_loss_seconds').value
        recognition_audio_filename = self.get_parameter('recognition_audio_file').value
        loss_audio_filename = self.get_parameter('loss_audio_file').value
        self.enabled = self.get_parameter('enabled').value
        
        # RED-first architecture: Rolling window parameters
        self.red_entry_match_threshold = self.get_parameter('red_entry_match_threshold').value
        self.red_entry_window_seconds = self.get_parameter('red_entry_window_seconds').value
        
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
        
        # Rolling buffer for RED-first architecture (list of (timestamp, person_id) tuples)
        self.recognition_buffer = []
        
        # GREEN/BLUE smoothing state (hysteresis filter to prevent flickering)
        # Only applies AFTER RED ends - while RED, only target person matters
        self.face_detected_start_time: Optional[float] = None  # When face first detected (for BLUE→GREEN)
        self.face_absent_start_time: Optional[float] = None    # When face first lost (for GREEN→BLUE)
        self.green_entry_delay = 2.0   # Seconds of face before BLUE→GREEN (default: 2s)
        self.blue_entry_delay = 3.0    # Seconds of no face before GREEN→BLUE (default: 3s)
        
        # Master volume from physical volume knob (multiplier for audio_volume)
        self.master_volume = 1.0  # Default: no attenuation until volume_control_node publishes
        
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
        
        # Create subscription to master volume from volume_control_node
        self.master_volume_sub = self.create_subscription(
            Float32,
            '/r2d2/audio/master_volume',
            self.master_volume_callback,
            qos_profile=rclpy.qos.QoSProfile(
                depth=10,
                durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL  # Get last value on subscribe
            )
        )
        
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
            f"  Local audio volume: {self.audio_volume*100:.0f}%\n"
            f"  Master volume: subscribed to /r2d2/audio/master_volume\n"
            f"  ALSA device: {self.alsa_device}\n"
            f"  RED-FIRST Architecture: Recognition is PRIMARY\n"
            f"  RED entry threshold: {self.red_entry_match_threshold} matches in {self.red_entry_window_seconds}s window\n"
            f"  RED status timeout: {self.red_status_timeout}s (resets on recognition)\n"
            f"  GREEN entry delay: {self.green_entry_delay}s (BLUE→GREEN smoothing)\n"
            f"  BLUE entry delay: {self.blue_entry_delay}s (GREEN→BLUE smoothing)\n"
            f"  Alert cooldown: {self.cooldown_seconds}s (min between alerts)\n"
            f"  Recognition cooldown after loss: {self.recognition_cooldown_after_loss}s (quiet period after loss alert)\n"
            f"  Enabled: {self.enabled}\n"
            f"  Audio player: {self.audio_player_path}\n"
            f"  State Machine: RED is primary (ignores non-target while active)"
        )
    
    def person_callback(self, msg: String):
        """
        Handle person_id messages - RED-FIRST ARCHITECTURE.
        
        Recognition is PRIMARY. This callback:
        1. Adds trained person matches to rolling buffer
        2. Checks if rolling window threshold met -> RED
        3. If already RED, resets 15s timer
        4. Does NOT handle GREEN/BLUE (that's face_count_callback's job)
        """
        person_id = msg.data
        current_time = time.time()
        
        if not self.enabled:
            return
        
        # Only process trained person recognitions (not "unknown", not empty)
        if person_id and person_id != "unknown":
            # Add to rolling buffer
            self.recognition_buffer.append((current_time, person_id))
            
            # Clean buffer: remove entries older than window
            cutoff_time = current_time - self.red_entry_window_seconds
            self.recognition_buffer = [(t, p) for t, p in self.recognition_buffer if t >= cutoff_time]
            
            # Count matches for THIS person in the window
            match_count = sum(1 for t, p in self.recognition_buffer if p == person_id)
            
            was_red = (self.current_status == "red")
            
            if was_red:
                # Already RED - just reset 15s timer (threshold already met)
                self.last_recognition_time = current_time
                self._publish_status("red", self.current_person, confidence=0.95)
                self.get_logger().debug(f"RED: Timer reset ({person_id}, {match_count} in window)")
            
            elif match_count >= self.red_entry_match_threshold:
                # THRESHOLD MET - Transition to RED (PRIMARY STATUS)
                self.last_recognition_time = current_time
            
                # Reset smoothing timers
                self.face_detected_start_time = None
                self.face_absent_start_time = None
            
                # Transition to RED
                old_status = self.current_status
                self.current_status = "red"
                self.current_person = person_id
                self.status_changed_time = current_time
                self.unknown_person_detected = False
                
                # Publish status FIRST
                self._publish_status("red", person_id, confidence=0.95)
                
                # Play "Hello!" beep (with cooldown)
                if self.last_recognition_beep_time is None or \
                   (current_time - self.last_recognition_beep_time) >= self.cooldown_seconds:
                    self._play_audio_file(self.recognition_audio, alert_type="RECOGNITION")
                    self.last_recognition_beep_time = current_time
                    self._publish_event(f"Recognized {person_id}!")
                
                self.get_logger().info(
                    f"RED-FIRST: {person_id} recognized ({old_status} -> RED) "
                    f"[{match_count}/{self.red_entry_match_threshold} in {self.red_entry_window_seconds}s]"
                )
            else:
                # Threshold NOT met - just log (GREEN/BLUE handled by face_count_callback)
                self.get_logger().debug(
                    f"Recognition buffered: {person_id} ({match_count}/{self.red_entry_match_threshold})"
                )
        
        # NOTE: "unknown" person_id is NOT processed here
        # GREEN/BLUE states are handled by face_count_callback based on face_count
        # This ensures RED is truly PRIMARY - recognition is checked FIRST
    
    def face_count_callback(self, msg: Int32):
        """
        Handle face_count messages - SECONDARY to RED status.
        
        RED-FIRST ARCHITECTURE:
        - This callback ONLY manages GREEN/BLUE transitions
        - It is IGNORED when current_status == "red"
        - RED status is managed exclusively by person_callback rolling window
        """
        face_count = msg.data
        current_time = time.time()
        
        if not self.enabled:
            return
        
        self.last_face_count = face_count
        self.last_face_count_time = current_time
        
        # RED-FIRST: Ignore face_count when in RED state
        # RED is managed by person_callback rolling window, not face detection
        if self.current_status == "red":
            return
        
        # SECONDARY LOGIC: Handle GREEN <-> BLUE transitions
        # Only reached when RED threshold is NOT met
        
        if face_count == 0:
            # No faces - track for BLUE entry
            if self.face_absent_start_time is None:
                self.face_absent_start_time = current_time
            self.face_detected_start_time = None
            
            time_without_face = current_time - self.face_absent_start_time
            
            if self.current_status == "green" and time_without_face >= self.blue_entry_delay:
                # GREEN -> BLUE transition
                self.current_status = "blue"
                self.current_person = "no_person"
                self.status_changed_time = current_time
                self.unknown_person_detected = False
                self._publish_status("blue", "no_person", confidence=0.0)
                self.get_logger().info(f"SECONDARY: GREEN -> BLUE (no face for {time_without_face:.1f}s)")
        
        else:
            # Faces detected - track for GREEN entry
            if self.face_detected_start_time is None:
                self.face_detected_start_time = current_time
            self.face_absent_start_time = None
            
            time_with_face = current_time - self.face_detected_start_time
            
            if self.current_status == "blue" and time_with_face >= self.green_entry_delay:
                # BLUE -> GREEN transition (unknown person visible)
                self.current_status = "green"
                self.current_person = "unknown"
                self.status_changed_time = current_time
                self.unknown_person_detected = True
                self._publish_status("green", "unknown", confidence=0.70)
                self.get_logger().info(f"SECONDARY: BLUE -> GREEN (face for {time_with_face:.1f}s)")
    
    def master_volume_callback(self, msg: Float32):
        """
        Handle master volume updates from volume_control_node (physical knob).
        
        The master volume multiplies with the local audio_volume parameter.
        """
        old_volume = self.master_volume
        self.master_volume = max(0.0, min(1.0, msg.data))  # Clamp to 0.0-1.0
        
        if abs(old_volume - self.master_volume) > 0.01:
            self.get_logger().debug(
                f"🎚️ Master volume updated: {old_volume:.2f} -> {self.master_volume:.2f}"
            )
    
    def check_loss_state(self):
        """
        Timer callback: Handle RED timeout - RED-FIRST architecture.
        
        When 15s timer expires:
        1. Check rolling buffer - if still has matches, stay RED
        2. Only if buffer empty, transition to GREEN (face) or BLUE (no face)
        """
        if not self.enabled:
            return
        
        if self.last_recognition_time is None:
            return
        
        if self.current_status != "red":
            return
        
        current_time = time.time()
        time_since_recognition = current_time - self.last_recognition_time
        
        if time_since_recognition > self.red_status_timeout:
            # 15s timer expired - but check rolling buffer FIRST (RED-FIRST principle)
            
            # Clean buffer
            cutoff_time = current_time - self.red_entry_window_seconds
            self.recognition_buffer = [(t, p) for t, p in self.recognition_buffer if t >= cutoff_time]
            
            # Count recent matches
            match_count = len(self.recognition_buffer)
            
            if match_count >= self.red_entry_match_threshold:
                # Still have matches - stay RED, reset timer
                self.last_recognition_time = current_time
                self.get_logger().debug(f"RED timeout: Buffer still active ({match_count} matches), staying RED")
                return
            
            # Buffer empty or below threshold - NOW transition out of RED
            previous_person = self.current_person
            
            # Clear buffer on RED exit
            self.recognition_buffer = []
            
            # Check face_count to decide GREEN or BLUE
            face_recently_detected = (
                self.last_face_count is not None and 
                self.last_face_count > 0 and
                self.last_face_count_time is not None and
                (current_time - self.last_face_count_time) < 1.0
            )
            
            if face_recently_detected:
                # Face visible but not recognized -> GREEN
                self.current_status = "green"
                self.current_person = "unknown"
                self.status_changed_time = current_time
                self.unknown_person_detected = True
                self.face_detected_start_time = current_time
                self.face_absent_start_time = None
                
                self._publish_status("green", "unknown", confidence=0.70)
                
                if self.last_loss_beep_time is None or \
                   (current_time - self.last_loss_beep_time) >= self.cooldown_seconds:
                    self._play_audio_file(self.loss_audio, alert_type="LOSS")
                    self.last_loss_beep_time = current_time
                
                self.get_logger().info(f"RED-FIRST: {previous_person} lost -> GREEN (unknown present)")
            
            else:
                # No face -> BLUE
                self.current_status = "blue"
                self.current_person = "no_person"
                self.status_changed_time = current_time
                self.unknown_person_detected = False
                self.face_detected_start_time = None
                self.face_absent_start_time = None
                
                self._publish_status("blue", "no_person", confidence=0.0)
            
                if self.last_loss_beep_time is None or \
                   (current_time - self.last_loss_beep_time) >= self.cooldown_seconds:
                    self._play_audio_file(self.loss_audio, alert_type="LOSS")
                    self.last_loss_beep_time = current_time
            
                self.get_logger().info(f"RED-FIRST: {previous_person} lost -> BLUE (no person)")
    
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
        Play an audio file using ffplay directly (same as gesture_intent_node).
        
        The effective volume is: master_volume * audio_volume
        Where master_volume comes from the physical volume knob.
        
        Args:
            audio_file: Path to the audio file to play
            alert_type: Type name for logging (RECOGNITION, LOSS, etc.)
        """
        if not audio_file.exists():
            self.get_logger().error(f"Audio file not found: {audio_file}")
            return
        
        try:
            # Calculate effective volume: master_volume * audio_volume
            effective_volume = self.master_volume * self.audio_volume
            
            self.get_logger().info(
                f"🔊 Playing {alert_type} audio: {audio_file.name} "
                f"(volume {effective_volume*100:.1f}% = {self.master_volume*100:.0f}% master × {self.audio_volume*100:.0f}% local)"
            )
            
            # Call ffplay directly with effective volume
            subprocess.Popen(
                ['ffplay', '-nodisp', '-autoexit', '-loglevel', 'error', 
                 '-af', f'volume={effective_volume}', str(audio_file)],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL
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
