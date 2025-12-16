#!/usr/bin/env python3
"""
R2D2 Audio Notification Node - Fixed State Machine Implementation

Implements exact timing rules for person recognition state management:
- BLUE: Target person not recognized. LED blue. On entry play "Voicy R2-D2 - 5.mp3" (lost you).
- RED: Target person (Severin) recognized. LED red. On entry from BLUE play "Voicy R2-D2 - 2.mp3" (hello).
  Never replay hello while staying in RED.

Input: Continuous recognition boolean (recognized = true iff person_id == "severin").
Evaluated continuously at sub-second cadence.

Fixed timing rules:
- REACQUIRE_WINDOW = 5.0s: Continuous recognized==False required in RED before transition
- RED_HOLD_TIME = 15.0s: Minimum seconds RED state must be held

Core rules:
1. BLUE -> RED: Immediate transition when recognized becomes true, play hello, set red_enter_time
2. RED minimum hold: Must remain RED for at least 15 seconds regardless of recognition dropouts
3. 5s reacquire rule in RED: If recognized==False, start loss timer. If recognized==True, clear timer immediately
4. RED -> BLUE: Only when (now - red_enter_time) >= 15s AND (now - last_recognized_true_time) >= 5s
5. BLUE -> RED: Immediate re-recognition (no cooldown, no delay)

Subscribes to:
  - /r2d2/perception/person_id (String): Person name (e.g., "severin")

Publishes:
  - /r2d2/audio/person_status (String): JSON status (RED/BLUE)
  - /r2d2/audio/notification_event (String): Event descriptions for debugging
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import subprocess
import time
from pathlib import Path
from typing import Optional
from dataclasses import dataclass, asdict
import json



# Simple PersonStatus data class (alternative to ROS message)
@dataclass
class PersonStatusData:
    """Person recognition status data."""
    status: str  # "red" | "blue"
    person_identity: str  # "severin" | "no_person"
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
    
    State machine implements exact timing rules:
    - BLUE: Target person not recognized. LED blue. On entry play "lost you" audio.
    - RED: Target person recognized. LED red. On entry from BLUE play "hello" audio.
      Never replay hello while staying in RED.
    
    Fixed timing constants:
    - REACQUIRE_WINDOW = 5.0s: Continuous recognized==False required in RED before transition
    - RED_HOLD_TIME = 15.0s: Minimum seconds RED state must be held
    
    State transitions:
    - BLUE -> RED: Immediate when recognized==True, play hello, set red_enter_time
    - RED minimum hold: Must remain RED for at least 15 seconds
    - RED reacquire: If recognized==False, start loss timer. If recognized==True, clear timer.
    - RED -> BLUE: Only when 15s hold met AND 5s continuous loss
    - BLUE -> RED: Immediate re-recognition (no cooldown)
    """
    
    def __init__(self):
        super().__init__('audio_notification_node')
        
        self.get_logger().info("R2D2 Audio Notification Node starting...")
        
        # Declare parameters
        self.declare_parameter('target_person', 'severin')
        self.declare_parameter('audio_volume', 0.05)       # 0.0-1.0 (audio file volume) - 5% volume (subtle)
        self.declare_parameter('alsa_device', 'hw:1,0')    # ALSA device for audio output (e.g., hw:1,0)
        self.declare_parameter('jitter_tolerance_seconds', 5.0)  # Deprecated - kept for compatibility only
        self.declare_parameter('loss_confirmation_seconds', 15.0)  # Deprecated - kept for compatibility only
        self.declare_parameter('recognition_audio_file', 'Voicy_R2-D2 - 2.mp3')  # Recognition alert audio
        self.declare_parameter('loss_audio_file', 'Voicy_R2-D2 - 5.mp3')  # Loss alert audio
        self.declare_parameter('enabled', True)
        
        # Get parameters
        self.target_person = self.get_parameter('target_person').value
        self.audio_volume = self.get_parameter('audio_volume').value
        self.alsa_device = self.get_parameter('alsa_device').value
        # Deprecated parameters kept for compatibility (not used in new state machine)
        self.jitter_tolerance = self.get_parameter('jitter_tolerance_seconds').value
        self.loss_confirmation = self.get_parameter('loss_confirmation_seconds').value
        recognition_audio_filename = self.get_parameter('recognition_audio_file').value
        loss_audio_filename = self.get_parameter('loss_audio_file').value
        self.enabled = self.get_parameter('enabled').value
        
        # State machine constants (fixed timing rules - DO NOT CHANGE)
        # REACQUIRE_WINDOW: Continuous recognized==False required in RED before transition to BLUE
        #   - Prevents false transitions during brief recognition dropouts
        #   - Must be 5.0s as specified
        self.REACQUIRE_WINDOW = 5.0
        # RED_HOLD_TIME: Minimum seconds RED state must be held before transition to BLUE is allowed
        #   - Ensures RED state is maintained for at least 15 seconds regardless of recognition dropouts
        #   - Must be 15.0s as specified
        self.RED_HOLD_TIME = 15.0
        # STALE_INPUT_THRESHOLD: Maximum seconds without person_id messages before input is considered stale
        #   - Prevents RED state from getting stuck forever if upstream recognition stops publishing
        #   - When input is stale, force recognized=False to allow normal loss logic to progress
        #   - CRITICAL: This does NOT bypass timing rules - normal state machine logic still applies
        self.STALE_INPUT_THRESHOLD = 3.0
        
        # State tracking
        self.current_status = "blue"  # "red" (recognized) | "blue" (lost)
        self.current_person = "no_person"  # "severin" | "no_person"
        self.status_changed_time = time.time()
        
        # Recognition state
        self.recognized: bool = False  # Continuous boolean (true iff person_id == target_person)
        self.red_enter_time: Optional[float] = None  # Timestamp when entered RED state (for 15s minimum hold)
        self.last_recognized_true_time: Optional[float] = None  # Last time recognized == True (for 5s reacquire window)
        self.last_person_id_message_time: Optional[float] = None  # Timestamp of last person_id message (for stale input watchdog)
        
        # Initialize last_person_id_message_time to prevent immediate stale detection on startup
        self.last_person_id_message_time = time.time()
        
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
        
        # Create subscription to person_id topic (only input needed - no face_count shortcuts)
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
        
        # Create publisher for person status (for LED, STT-LLM-TTS, database logging)
        self.status_pub = self.create_publisher(
            String,
            '/r2d2/audio/person_status',
            qos_profile=rclpy.qos.QoSProfile(depth=10)
        )
        
        # Create timer for state machine evaluation (check every 500ms for continuous evaluation)
        # This ensures state transitions happen promptly even if person_id topic slows
        self.create_timer(0.5, self._evaluate_state_machine)
        
        # Create timer for regular status publishing (1 Hz = every 1 second)
        # This ensures web page and other subscribers get regular updates even when person_id topic stops publishing
        self.create_timer(1.0, self.publish_current_status)
        
        self.get_logger().info(
            f"Audio Notification Node initialized (Audio Files):\n"
            f"  Target person: {self.target_person}\n"
            f"  Recognition audio: {self.recognition_audio.name}\n"
            f"  Loss audio: {self.loss_audio.name}\n"
            f"  Audio volume: {self.audio_volume*100:.0f}%\n"
            f"  ALSA device: {self.alsa_device}\n"
            f"  REACQUIRE_WINDOW: {self.REACQUIRE_WINDOW}s (continuous loss required in RED)\n"
            f"  RED_HOLD_TIME: {self.RED_HOLD_TIME}s (minimum RED state duration)\n"
            f"  STALE_INPUT_THRESHOLD: {self.STALE_INPUT_THRESHOLD}s (watchdog for stale input)\n"
            f"  Enabled: {self.enabled}\n"
            f"  Audio player: {self.audio_player_path}"
        )
    
    def person_callback(self, msg: String):
        """
        Handle person_id messages from face recognition.
        
        Evaluates recognized boolean continuously (true iff person_id == target_person).
        Updates last_recognized_true_time when recognized becomes True.
        Triggers state machine evaluation on every callback.
        
        Args:
            msg: String message containing person name (e.g., "severin") or other values
        """
        if not self.enabled:
            return
        
        person_id = msg.data
        current_time = time.time()
        
        # Update last message time for stale input watchdog
        self.last_person_id_message_time = current_time
        
        # Evaluate recognized boolean continuously (true iff person_id == target_person)
        self.recognized = (person_id == self.target_person)
        
        # Update last_recognized_true_time when recognized becomes True
        # Note: last_recognized_true_time represents the start of the continuous loss period.
        # Updating it to current_time when recognized becomes True ensures the 5s window
        # represents continuous absence (resets on reacquire), not cumulative loss.
        if self.recognized:
            self.last_recognized_true_time = current_time
        
        # Trigger state machine evaluation on every callback
        self._evaluate_state_machine()
    
    def _evaluate_state_machine(self):
        """
        Evaluate state machine according to exact timing rules.
        
        Core rules:
        1. BLUE -> RED: Immediate transition when recognized==True, play hello, set red_enter_time
        2. RED minimum hold: Must remain RED for at least 15 seconds
        3. 5s reacquire rule in RED: If recognized==False, start loss timer. If recognized==True, clear timer.
        4. RED -> BLUE: Only when 15s hold met AND 5s continuous loss
        5. BLUE -> RED: Immediate re-recognition (no cooldown)
        """
        if not self.enabled:
            return
        
        current_time = time.time()
        
        # Check for stale input (watchdog requirement per spec 6.4.1)
        # CRITICAL: This is ONLY an input-sanity mechanism, NOT a state-transition shortcut
        # When input is stale, we force recognized=False to allow normal loss logic to progress,
        # but the normal timing rules (15s hold + 5s continuous loss) still apply
        if self.last_person_id_message_time is None:
            self.last_person_id_message_time = current_time
        time_since_last_message = current_time - self.last_person_id_message_time
        if time_since_last_message > self.STALE_INPUT_THRESHOLD:
            # Input is stale - force recognized=False to prevent stuck RED state
            # NOTE: This does NOT bypass timing rules - normal state machine logic still applies
            original_recognized = self.recognized
            self.recognized = False
            if original_recognized and self.current_status == "red":
                self.get_logger().warn(
                    f"⚠️ Stale input detected ({time_since_last_message:.1f}s since last message). "
                    f"Forcing recognized=False. Normal timing rules (15s hold + 5s continuous loss) still apply."
                )
        
        # Rule 1: BLUE -> RED transition
        if self.current_status == "blue" and self.recognized:
            # Immediate transition to RED (no delay, no cooldown)
            self.current_status = "red"
            self.current_person = self.target_person
            self.status_changed_time = current_time
            # CRITICAL: red_enter_time MUST only be set on BLUE→RED transition (spec 6.4.1)
            # MUST NOT be reset on every recognized frame - this would break 15s hold requirement
            self.red_enter_time = current_time  # Set RED entry time for 15s minimum hold
            self.last_recognized_true_time = current_time  # Clear loss timers
            
            # Publish status FIRST (so LED lights up immediately)
            self._publish_status("red", self.target_person, confidence=0.95)
            
            # Play hello audio on entry
            self._trigger_recognition_alert()
            self._publish_event(f"🎉 Recognized {self.target_person}!")
            self.get_logger().info(f"✓ {self.target_person} recognized! (BLUE -> RED)")
            return
        
        # Rule 2 & 3: RED state handling (minimum hold + reacquire window)
        if self.current_status == "red":
            # Rule 2: RED minimum hold - cannot transition to BLUE until 15s have passed
            time_in_red = current_time - (self.red_enter_time or current_time)
            
            # Rule 3: 5s reacquire rule in RED
            if self.recognized:
                # If recognized becomes True, immediately clear loss timers and stay RED
                # Note: last_recognized_true_time reset is for loss timer (5s continuous loss window)
                # This is INDEPENDENT of red_enter_time (15s hold timer) - both must be tracked separately
                self.last_recognized_true_time = current_time
                # Update status (duration keeps updating)
                self._publish_status("red", self.target_person, confidence=0.95)
                # No audio, no state change - just reset the loss timer
            else:
                # recognized == False in RED
                # Check if we can transition to BLUE (Rule 4)
                if self.last_recognized_true_time is None:
                    # Initialize if not set (shouldn't happen, but safety check)
                    self.last_recognized_true_time = current_time - self.REACQUIRE_WINDOW
                
                time_since_last_recognized = current_time - self.last_recognized_true_time
                
                # CRITICAL: RED→BLUE transition ONLY allowed when BOTH conditions met (spec 6.4):
                # 1. 15s minimum hold time (red_enter_time)
                # 2. 5s continuous loss (last_recognized_true_time)
                # NO SHORTCUTS: face_count, no_person, camera loss cannot bypass these rules
                # Rule 4: RED -> BLUE transition (only allowed path)
                # BOTH conditions must hold:
                # a) Minimum 15s hold time has passed
                # b) Continuous recognized==False for at least 5s
                if time_in_red >= self.RED_HOLD_TIME and time_since_last_recognized >= self.REACQUIRE_WINDOW:
                    # Transition to BLUE immediately
                    self.current_status = "blue"
                    self.current_person = "no_person"
                    self.status_changed_time = current_time
                    self.red_enter_time = None  # Clear RED entry time
                    self.last_recognized_true_time = None  # Clear loss timers
                    
                    # Publish status FIRST (so LED lights up immediately)
                    self._publish_status("blue", "no_person", confidence=0.0)
                    
                    # Play lost you audio on entry
                    self._trigger_loss_alert()
                    self._publish_event(f"❌ {self.target_person} lost (after {time_in_red:.1f}s in RED, {time_since_last_recognized:.1f}s continuous loss)")
                    self.get_logger().info(
                        f"✗ {self.target_person} lost (RED -> BLUE: {time_in_red:.1f}s in RED, "
                        f"{time_since_last_recognized:.1f}s continuous loss)"
                    )
                else:
                    # Still in RED - update status (duration keeps updating)
                    # No audio, no state change - waiting for either:
                    # - 15s minimum hold to pass (if not yet)
                    # - 5s continuous loss (if 15s hold already met)
                    self._publish_status("red", self.target_person, confidence=0.95)
        
        # Rule 5: BLUE state (idle, waiting for recognition)
        # No special handling needed - Rule 1 handles BLUE -> RED transition
        if self.current_status == "blue":
            # Update status (duration keeps updating)
            self._publish_status("blue", "no_person", confidence=0.0)
    
    def publish_current_status(self):
        """
        Timer callback: Regularly publish current status (1 Hz) so web page and other subscribers
        get updates even when person_id topic stops publishing.
        """
        if not self.enabled:
            return
        
        # Map current state to confidence values (only RED and BLUE states)
        confidence_map = {
            "red": 0.95,
            "blue": 0.0
        }
        
        confidence = confidence_map.get(self.current_status, 0.0)
        self._publish_status(self.current_status, self.current_person, confidence)
    
    def _publish_status(self, status: str, person_identity: str, confidence: float):
        """
        Publish current recognition status for LED, STT-LLM-TTS, and database logging.
        
        Publishes as JSON String message to /r2d2/audio/person_status
        
        Args:
            status: "red" (recognized) | "blue" (lost)
            person_identity: "severin" | "no_person"
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
        
        self.get_logger().debug(
            f"📊 Status: {status.upper()} | Person: {person_identity} | Duration: {duration:.1f}s"
        )
    
    def _trigger_recognition_alert(self):
        """
        Play recognition audio when target person is recognized (BLUE -> RED transition).
        Audio only plays on state entry, never during RED state.
        No cooldown needed - state machine ensures this is only called on entry.
        """
        self._play_audio_file(
            audio_file=self.recognition_audio,
            alert_type="RECOGNITION"
        )
    
    def _trigger_loss_alert(self):
        """
        Play loss audio when person is confirmed lost (RED -> BLUE transition).
        Audio only plays on state entry.
        No cooldown needed - state machine ensures this is only called on entry.
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
            
            self.get_logger().debug(f"Audio command: {' '.join(cmd)}")
            
            process = subprocess.Popen(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                start_new_session=True
            )
            
            # Check if process started successfully (non-blocking check)
            if process.poll() is None:
                self.get_logger().debug(f"Audio player process started (PID: {process.pid})")
            else:
                # Process already finished (error)
                stdout, stderr = process.communicate(timeout=1)
                self.get_logger().error(
                    f"Audio player failed immediately: stdout={stdout.decode()}, stderr={stderr.decode()}"
                )
            
        except Exception as e:
            self.get_logger().error(f"Error playing {alert_type} audio: {e}", exc_info=True)
    
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
