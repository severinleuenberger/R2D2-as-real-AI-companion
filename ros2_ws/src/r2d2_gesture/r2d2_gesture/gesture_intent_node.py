#!/usr/bin/env python3
"""
Gesture Intent Node - Translates gesture events into conversation actions

Purpose:
  Subscribes to gesture events from perception node and triggers speech system
  actions (start/stop conversation) with strict gating based on person recognition
  status and speech system state.

Gating Logic:
  - Only processes gestures when target person is recognized (RED state)
  - Start gesture: only works when speech system is inactive
  - Stop gesture: only works when speech system is active
  - Cooldown periods prevent rapid re-triggering

Watchdog Features:
  - Monitors person presence (RED/BLUE/GREEN LED status)
  - Automatically stops speech service after configurable timeout (default 5 min)
  - Prevents unnecessary OpenAI API calls when no one is present
  - Optional auto-restart when person returns

Author: R2D2 Perception Pipeline
Date: December 17, 2025
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32
from std_srvs.srv import Trigger
import json
import time
import subprocess
from pathlib import Path


class GestureIntentNode(Node):
    """
    ROS 2 node that translates gesture events into conversation actions.
    
    Subscribes to:
      - /r2d2/perception/gesture_event (String): Gesture events
      - /r2d2/audio/person_status (String, JSON): Person recognition status  
      - /r2d2/speech/session_status (String, JSON): Speech system state
    
    Service clients:
      - /r2d2/speech/start_session (Trigger): Start conversation
      - /r2d2/speech/stop_session (Trigger): Stop conversation
    """
    
    def __init__(self):
        super().__init__('gesture_intent_node')
        
        # Declare parameters
        self.declare_parameter('cooldown_start_seconds', 2.0)
        self.declare_parameter('cooldown_stop_seconds', 1.0)
        self.declare_parameter('enabled', True)
        self.declare_parameter('auto_shutdown_enabled', True)
        self.declare_parameter('auto_shutdown_timeout_seconds', 35.0)  # 35 seconds (cost optimization)
        self.declare_parameter('auto_restart_on_return', False)
        self.declare_parameter('audio_feedback_enabled', True)
        self.declare_parameter('audio_volume', 0.02)  # 0.0-1.0 (audio feedback volume) - 30% volume (from config/audio_params.yaml)
        self.declare_parameter('vad_silence_timeout_seconds', 30.0)  # VAD-based silence timeout (Option 2: VAD-only)
        self.declare_parameter('speaking_start_grace_seconds', 5.0)  # Grace period after starting conversation to ignore fist gestures
        self.declare_parameter('fist_window_seconds', 1.0)  # Rolling window duration for sustained fist detection
        self.declare_parameter('fist_threshold', 2)  # Detections required in window (~20% of 10 max at 10Hz)
        
        # Get parameters
        self.cooldown_start = self.get_parameter('cooldown_start_seconds').value
        self.cooldown_stop = self.get_parameter('cooldown_stop_seconds').value
        self.enabled = self.get_parameter('enabled').value
        self.auto_shutdown_enabled = self.get_parameter('auto_shutdown_enabled').value
        self.auto_shutdown_timeout = self.get_parameter('auto_shutdown_timeout_seconds').value
        self.auto_restart_on_return = self.get_parameter('auto_restart_on_return').value
        self.audio_feedback_enabled = self.get_parameter('audio_feedback_enabled').value
        self.audio_volume = self.get_parameter('audio_volume').value
        self.vad_silence_timeout = self.get_parameter('vad_silence_timeout_seconds').value
        self.speaking_start_grace = self.get_parameter('speaking_start_grace_seconds').value
        self.fist_window = self.get_parameter('fist_window_seconds').value
        self.fist_threshold = self.get_parameter('fist_threshold').value
        
        # Audio feedback paths
        audio_assets_dir = Path.home() / 'dev' / 'r2d2' / 'ros2_ws' / 'src' / 'r2d2_audio' / 'r2d2_audio' / 'assets' / 'audio'
        self.gesture_ack_sound = audio_assets_dir / 'Voicy_R2-D2 - 12.mp3'  # Immediate acknowledgment
        self.start_beep_sound = audio_assets_dir / 'Voicy_R2-D2 - 16.mp3'   # Session ready
        self.warning_beep_sound = audio_assets_dir / 'Voicy_R2-D2 - 7.mp3'   # Fist warning (stage 1)
        self.stop_beep_sound = audio_assets_dir / 'Voicy_R2-D2 - 20.mp3'    # Session stopped (stage 2)
        
        # State tracking
        self.person_status = None  # "red", "blue", or "green"
        self.session_active = False
        self.last_trigger_time = 0.0
        self.last_red_status_time = None  # Watchdog: time when RED status last seen
        self.auto_shutdown_triggered = False  # Watchdog: flag to prevent repeated shutdowns
        
        # Master volume from physical volume knob (multiplier for audio_volume)
        self.master_volume = 1.0  # Default: no attenuation until volume_control_node publishes
        
        # SPEAKING state tracking (conversation protection)
        self.speaking_state = "idle"  # "idle" or "speaking"
        self.speaking_start_time = None
        
        # VAD-based activity tracking (Option 2: VAD-only approach)
        self.vad_state = "silent"  # "speaking" or "silent" from OpenAI VAD
        self.last_vad_activity_time = None  # Last time user was speaking (from VAD)
        
        # Two-stage fist gesture detection state machine
        self.fist_detection_buffer = []  # List of timestamps for rolling window
        self.fist_stage = "idle"  # "idle" | "stage1" | "warning_played" | "stage2"
        self.last_fist_detection_time = None  # For timeout detection (release check)
        self.stop_beep_already_played = False  # Flag to avoid duplicate stop beep
        
        # Create subscriptions
        self.gesture_sub = self.create_subscription(
            String,
            '/r2d2/perception/gesture_event',
            self.gesture_callback,
            10
        )
        
        self.person_status_sub = self.create_subscription(
            String,
            '/r2d2/audio/person_status',
            self.person_status_callback,
            10
        )
        
        self.session_status_sub = self.create_subscription(
            String,
            '/r2d2/speech/session_status',
            self.session_status_callback,
            10
        )
        
        self.vad_sub = self.create_subscription(
            String,
            '/r2d2/speech/voice_activity',
            self.vad_callback,
            10
        )
        
        # Create subscription to master volume from volume_control_node
        self.master_volume_sub = self.create_subscription(
            Float32,
            '/r2d2/audio/master_volume',
            self.master_volume_callback,
            rclpy.qos.QoSProfile(
                depth=10,
                durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL  # Get last value on subscribe
            )
        )
        
        # Create service clients for Fast Mode (Realtime API)
        self.start_session_client = self.create_client(
            Trigger,
            '/r2d2/speech/start_session'
        )
        
        self.stop_session_client = self.create_client(
            Trigger,
            '/r2d2/speech/stop_session'
        )
        
        # Create service clients for Intelligent Mode (REST APIs)
        self.start_intelligent_client = self.create_client(
            Trigger,
            '/r2d2/speech/intelligent/start_session'
        )
        
        self.process_intelligent_turn_client = self.create_client(
            Trigger,
            '/r2d2/speech/intelligent/process_turn'
        )
        
        self.stop_intelligent_client = self.create_client(
            Trigger,
            '/r2d2/speech/intelligent/stop_session'
        )
        
        # Intelligent mode state
        self.intelligent_mode_active = False
        self.intelligent_mode_processing = False
        self.intelligent_speaking_state = "idle"  # "idle" or "speaking"
        self.intelligent_speaking_start_time = None
        self.intelligent_last_activity_time = None  # Time of last successful turn
        
        # Intelligent mode conversation loop timer
        self.intelligent_loop_timer = None
        
        # Create watchdog timer (checks every 10 seconds)
        if self.auto_shutdown_enabled:
            self.watchdog_timer = self.create_timer(10.0, self.watchdog_callback)
        
        self.get_logger().info('Gesture Intent Node initialized')
        self.get_logger().info(f'Cooldown: start={self.cooldown_start}s, stop={self.cooldown_stop}s')
        self.get_logger().info(f'Enabled: {self.enabled}')
        self.get_logger().info(
            f'Auto-shutdown: enabled={self.auto_shutdown_enabled}, '
            f'idle_timeout={self.auto_shutdown_timeout}s, '
            f'auto_restart={self.auto_restart_on_return}'
        )
        self.get_logger().info(f'Audio feedback: {self.audio_feedback_enabled}')
        self.get_logger().info(f'VAD-Only Mode: {self.vad_silence_timeout}s silence timeout (Option 2)')
        self.get_logger().info(f'Speaking grace period: {self.speaking_start_grace}s (ignores fist after start)')
        self.get_logger().info(f'Two-stage fist stop: window={self.fist_window}s, threshold={self.fist_threshold} detections')
    
    def person_status_callback(self, msg):
        """
        Handle person status updates from audio notification node.
        
        Option 2: VAD-Only - Camera status only used for initial gesture gating.
        During active sessions, VAD determines when to stop (not camera).
        
        Args:
            msg: String message containing JSON status data
        """
        try:
            status_data = json.loads(msg.data)
            old_status = self.person_status
            self.person_status = status_data.get('status', None)
            
            # Log status changes
            if old_status != self.person_status:
                self.get_logger().debug(f'👤 Person status: {old_status} → {self.person_status}')
                
            # Reset idle watchdog when person returns (for auto-restart feature)
            if self.person_status == "red":
                if self.last_red_status_time is not None:
                    self.get_logger().debug('Idle watchdog: RED detected, timer reset')
                self.last_red_status_time = None
                self.auto_shutdown_triggered = False
                
                # Auto-restart if enabled
                if self.auto_shutdown_triggered and self.auto_restart_on_return:
                    self.get_logger().info('👤 Person returned. Auto-restarting speech service.')
                    self._start_session()
                
        except json.JSONDecodeError as e:
            self.get_logger().warn(f'Failed to parse person status: {e}')
    
    def session_status_callback(self, msg):
        """
        Handle speech session status updates.
        
        Args:
            msg: String message containing JSON session status
        """
        try:
            status_data = json.loads(msg.data)
            # Speech node publishes {"status": "active"|"inactive"|"connected"|"disconnected"}
            # "active" = lifecycle state (node ready, NOT conversation)
            # "connected" = session state (conversation active with OpenAI)
            status_str = status_data.get('status', '')
            old_active = self.session_active
            self.session_active = (status_str == 'connected')  # Only connected = conversation active
            
            # Log all session status updates for debugging
            self.get_logger().info(f'📡 Session status received: {status_str} (active={self.session_active}, was={old_active})')
            
            # Play audio feedback on status changes
            if old_active != self.session_active:
                if self.session_active:
                    # Session started (inactive → active or disconnected → connected)
                    self.get_logger().info(f'🔊 Session started (status={status_str})')
                    self._play_audio_feedback(self.start_beep_sound)
                    # Reset stop beep flag on new session
                    self.stop_beep_already_played = False
                else:
                    # Session stopped (active → inactive or connected → disconnected)
                    self.get_logger().info(f'🔊 Session stopped (status={status_str})')
                    # Only play stop beep if not already played by fist gesture
                    if not self.stop_beep_already_played:
                        self._play_audio_feedback(self.stop_beep_sound)
                    else:
                        self.get_logger().debug('Stop beep already played by fist gesture confirmation')
                    # CRITICAL: Also reset speaking state when session disconnects externally
                    if self.speaking_state == "speaking":
                        self.speaking_state = "idle"
                        self.speaking_start_time = None
                        self.last_vad_activity_time = None
                        self.vad_state = "silent"
                        self.get_logger().info('🔇 Exited SPEAKING state (reason: session_disconnected)')
                    
                self.get_logger().info(f'Session active changed: {old_active} → {self.session_active} (status={status_str})')
            else:
                self.get_logger().debug(f'Session active: {self.session_active} (status={status_str})')
        except json.JSONDecodeError as e:
            self.get_logger().warn(f'Failed to parse session status: {e}')
    
    def vad_callback(self, msg):
        """
        Handle Voice Activity Detection updates from OpenAI speech system.
        
        Option 2: VAD-Only Approach - Uses OpenAI's built-in VAD instead of camera.
        
        The silence timer only starts when speech_stopped is received.
        While user is speaking, no timeout is counted.
        
        Args:
            msg: String message containing JSON VAD data
        """
        try:
            vad_data = json.loads(msg.data)
            old_state = self.vad_state
            self.vad_state = vad_data.get('state', 'silent')  # "speaking" or "silent"
            
            if self.vad_state == "speaking":
                # User is actively speaking - STOP counting silence
                self.last_vad_activity_time = None  # No timer while speaking
                if old_state != "speaking":
                    self.get_logger().info('🎤 VAD: User speaking (silence timer paused)')
            else:
                # User stopped speaking - START counting silence from NOW
                if old_state == "speaking":
                    self.last_vad_activity_time = self.get_clock().now()
                    self.get_logger().info('🔇 VAD: User silent (60s silence timer started)')
                    
        except json.JSONDecodeError as e:
            self.get_logger().warn(f'Failed to parse VAD data: {e}')
    
    def master_volume_callback(self, msg: Float32):
        """
        Handle master volume updates from volume_control_node (physical knob).
        
        The master volume multiplies with the local audio_volume parameter.
        """
        old_volume = self.master_volume
        self.master_volume = max(0.0, min(1.0, msg.data))  # Clamp to 0.0-1.0
        
        if abs(old_volume - self.master_volume) > 0.01:
            self.get_logger().debug(
                f'🎚️ Master volume updated: {old_volume:.2f} -> {self.master_volume:.2f}'
            )
    
    def gesture_callback(self, msg):
        """
        Handle gesture events and trigger appropriate actions.
        
        Args:
            msg: String message containing gesture name
        """
        if not self.enabled:
            return
        
        gesture_name = msg.data
        current_time = time.time()
        
        # Log all gesture detections for debugging
        self.get_logger().info(f'🎯 Gesture detected: {gesture_name} (person_status={self.person_status}, session_active={self.session_active})')
        
        # Gate 1: Check person status (must be RED = target person recognized)
        if self.person_status != "red":
            self.get_logger().warn(
                f'❌ Gesture ignored: person_status={self.person_status} (need "red")'
            )
            return
        
        # Gate 2: Check cooldown
        time_since_last = current_time - self.last_trigger_time
        
        if gesture_name == "index_finger_up":
            # Start gesture: only if speech inactive and cooldown elapsed
            if self.session_active:
                self.get_logger().warn('❌ Start gesture ignored: session already active')
                return
            
            if time_since_last < self.cooldown_start:
                self.get_logger().warn(
                    f'❌ Start gesture ignored: cooldown ({time_since_last:.1f}s < {self.cooldown_start}s)'
                )
                return
            
            # Trigger start session - Enter SPEAKING state
            self.get_logger().info('🤚 Index finger up detected → Starting conversation')
            
            # Play immediate acknowledgment beep (before service call)
            self._play_audio_feedback(self.gesture_ack_sound)
            
            self._enter_speaking_state()
            self.last_trigger_time = current_time
        
        elif gesture_name == "fist":
            # Two-stage fist stop: requires sustained detection with warning beep
            # Stage 1: 1.5s sustained → warning beep (chance to cancel)
            # Stage 2: continue holding 1.5s → actual stop + confirmation beep
            
            # Must have an active session or intelligent speaking state
            if not self.session_active and self.intelligent_speaking_state != "speaking":
                self.get_logger().debug('Fist detected but no active session')
                return
            
            # Check speaking start grace period for Fast Mode
            if self.speaking_state == "speaking" and self.speaking_start_time:
                time_since_start = (self.get_clock().now() - self.speaking_start_time).nanoseconds / 1e9
                if time_since_start < self.speaking_start_grace:
                    self.get_logger().info(
                        f'✊ Fist ignored: speaking grace period ({time_since_start:.1f}s < {self.speaking_start_grace}s)'
                    )
                    return
            
            # Check speaking start grace period for R2-D2 Mode
            if self.intelligent_speaking_state == "speaking" and self.intelligent_speaking_start_time:
                time_since_start = (self.get_clock().now() - self.intelligent_speaking_start_time).nanoseconds / 1e9
                if time_since_start < self.speaking_start_grace:
                    self.get_logger().info(
                        f'✊ Fist ignored: R2-D2 speaking grace period ({time_since_start:.1f}s < {self.speaking_start_grace}s)'
                    )
                    return
            
            # Rolling window fist detection
            self.last_fist_detection_time = current_time
            self.fist_detection_buffer.append(current_time)
            
            # Prune old entries outside window
            cutoff_time = current_time - self.fist_window
            self.fist_detection_buffer = [t for t in self.fist_detection_buffer if t >= cutoff_time]
            
            # Count detections in window
            detection_count = len(self.fist_detection_buffer)
            
            # State machine logic
            if self.fist_stage == "idle" or self.fist_stage == "stage1":
                # Stage 1: detecting first sustained hold
                if detection_count >= self.fist_threshold:
                    # Threshold met → play warning beep
                    self.get_logger().info(
                        f'⚠️  Fist Stage 1 complete ({detection_count}/{self.fist_threshold}) → Playing warning beep'
                    )
                    self._play_audio_feedback(self.warning_beep_sound)
                    
                    # Transition to warning_played, reset buffer for stage 2
                    self.fist_stage = "warning_played"
                    self.fist_detection_buffer = []
                    self.last_trigger_time = current_time
                else:
                    # Still accumulating
                    if self.fist_stage == "idle":
                        self.fist_stage = "stage1"
                    self.get_logger().debug(
                        f'✊ Fist Stage 1: {detection_count}/{self.fist_threshold} detections in {self.fist_window}s window'
                    )
            
            elif self.fist_stage == "warning_played" or self.fist_stage == "stage2":
                # Stage 2: detecting confirmation hold after warning
                if detection_count >= self.fist_threshold:
                    # Threshold met again → STOP session
                    self.get_logger().info(
                        f'🛑 Fist Stage 2 complete ({detection_count}/{self.fist_threshold}) → Stopping session'
                    )
                    
                    # Play stop beep IMMEDIATELY (on gesture confirmation, not on disconnect)
                    self._play_audio_feedback(self.stop_beep_sound)
                    self.stop_beep_already_played = True  # Prevent duplicate in session_status_callback
                    
                    # Stop appropriate mode
                    if self.speaking_state == "speaking":
                        self.get_logger().info('✊ Fist confirmed → Stopping Fast Mode conversation')
                        self._exit_speaking_state(reason="user_fist_gesture_confirmed")
                    elif self.intelligent_speaking_state == "speaking":
                        self.get_logger().info('✊ Fist confirmed → Stopping R2-D2 Mode conversation')
                        self._exit_intelligent_speaking_state(reason="user_fist_gesture_confirmed")
                    
                    # Reset state machine
                    self.fist_stage = "idle"
                    self.fist_detection_buffer = []
                    self.last_trigger_time = current_time
                else:
                    # Still accumulating stage 2
                    if self.fist_stage == "warning_played":
                        self.fist_stage = "stage2"
                    self.get_logger().debug(
                        f'✊ Fist Stage 2: {detection_count}/{self.fist_threshold} detections (continue holding to stop)'
                    )
        
        elif gesture_name == "open_hand":
            # Open hand gesture: trigger R2-D2 Mode (REST APIs with intelligent model)
            # This ENTERS a speaking state with continuous turn loop (like Fast Mode)
            
            if self.session_active:
                self.get_logger().warn('❌ Open hand ignored: fast mode session active (use fist to stop first)')
                return
            
            if self.intelligent_speaking_state == "speaking":
                self.get_logger().warn('❌ Open hand ignored: already in R2-D2 mode speaking state')
                return
            
            if time_since_last < self.cooldown_start:
                self.get_logger().warn(
                    f'❌ Open hand ignored: cooldown ({time_since_last:.1f}s < {self.cooldown_start}s)'
                )
                return
            
            # Trigger R2-D2 mode conversation
            self.get_logger().info('🖐️  Open hand detected → Starting R2-D2 Mode conversation')
            
            # Play immediate acknowledgment beep
            self._play_audio_feedback(self.gesture_ack_sound)
            
            self._enter_intelligent_speaking_state()
            self.last_trigger_time = current_time
        
        else:
            self.get_logger().debug(f'Unknown gesture: {gesture_name}')
        
        # Check for fist release (cancel two-stage stop if user releases fist)
        if gesture_name != "fist" and self.fist_stage != "idle":
            # User made a different gesture or no gesture - check if fist was released
            if self.last_fist_detection_time is not None:
                time_since_fist = current_time - self.last_fist_detection_time
                if time_since_fist > 0.5:  # 0.5s timeout = consider fist released
                    self.get_logger().info(
                        f'✋ Fist released (stage={self.fist_stage}) → Cancelling stop sequence'
                    )
                    self.fist_stage = "idle"
                    self.fist_detection_buffer = []
    
    def _enter_speaking_state(self):
        """Enter SPEAKING state when conversation starts (Option 2: VAD-only)."""
        self.speaking_state = "speaking"
        self.speaking_start_time = self.get_clock().now()
        self.last_vad_activity_time = None  # No silence timer until speech_stopped received
        self.vad_state = "speaking"  # Assume speaking when session starts
        
        self.get_logger().info(
            f'🗣️  Entered SPEAKING state (VAD-only: {self.vad_silence_timeout}s silence timeout)'
        )
        self._start_session()
    
    def _exit_speaking_state(self, reason: str):
        """Exit SPEAKING state when conversation ends (Option 2: VAD-only)."""
        self.speaking_state = "idle"
        self.speaking_start_time = None
        self.last_vad_activity_time = None
        self.vad_state = "silent"
        
        self.get_logger().info(f'🔇 Exited SPEAKING state (reason: {reason})')
        self._stop_session()
    
    def _call_service(self, client, service_name):
        """
        Call a service asynchronously.
        
        Args:
            client: Service client
            service_name: Name of service for logging
        """
        if not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn(f'Service {service_name} not available')
            return
        
        request = Trigger.Request()
        future = client.call_async(request)
        
        # Add callback to log result
        future.add_done_callback(
            lambda f: self._service_callback(f, service_name)
        )
    
    def _service_callback(self, future, service_name):
        """
        Handle service call response.
        
        Args:
            future: Future object from service call
            service_name: Name of service for logging
        """
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(f'✅ {service_name}: {response.message}')
            else:
                self.get_logger().warn(f'⚠️  {service_name} failed: {response.message}')
        except Exception as e:
            self.get_logger().error(f'❌ {service_name} error: {e}')
    
    def watchdog_callback(self):
        """
        Watchdog timer callback: Check timeouts and auto-stop speech if needed.
        
        Option 2: VAD-Only Approach
        - SPEAKING state (Fast Mode): Uses VAD silence timeout (60s default)
        - R2-D2 SPEAKING state: Uses activity-based timeout (60s since last turn)
        - IDLE state: Idle failsafe for forgotten sessions (35s default)
        """
        if not self.auto_shutdown_enabled:
            return
        
        current_time = self.get_clock().now()
        
        # ===== R2-D2 SPEAKING STATE: ACTIVITY-BASED TIMEOUT =====
        if self.intelligent_speaking_state == "speaking":
            if self.intelligent_last_activity_time:
                time_since_activity = (current_time - self.intelligent_last_activity_time).nanoseconds / 1e9
                
                # Log progress every 15 seconds
                if int(time_since_activity) % 15 == 0 and int(time_since_activity) > 0:
                    remaining = self.vad_silence_timeout - time_since_activity
                    if remaining > 0:
                        self.get_logger().info(
                            f'R2-D2: Idle for {int(time_since_activity)}s / {int(self.vad_silence_timeout)}s '
                            f'(auto-stop in {int(remaining)}s if no activity)'
                        )
                
                # Check if timeout exceeded
                if time_since_activity > self.vad_silence_timeout:
                    self.get_logger().warn(
                        f'R2-D2: No activity for {time_since_activity:.0f}s '
                        f'(threshold: {self.vad_silence_timeout}s). Stopping conversation.'
                    )
                    self._exit_intelligent_speaking_state(reason="inactivity_timeout")
            return  # Don't check other timeouts while in R2-D2 speaking state
        
        # ===== SPEAKING STATE: VAD-BASED PROTECTION =====
        # Only count silence when VAD says user is actually silent
        if self.speaking_state == "speaking":
            # If user is currently speaking, do nothing - no timeout while talking
            if self.vad_state == "speaking":
                return  # User is talking, don't count any time
            
            # User is silent - check if timeout exceeded
            if self.last_vad_activity_time is not None:
                time_silent = (current_time - self.last_vad_activity_time).nanoseconds / 1e9
                
                # Log progress every 15 seconds
                if int(time_silent) % 15 == 0 and int(time_silent) > 0:
                    remaining = self.vad_silence_timeout - time_silent
                    if remaining > 0:
                        self.get_logger().info(
                            f'VAD: Silent for {int(time_silent)}s / {int(self.vad_silence_timeout)}s '
                            f'(auto-stop in {int(remaining)}s if no speech detected)'
                        )
                
                # Check if VAD silence timeout exceeded
                if time_silent > self.vad_silence_timeout:
                    self.get_logger().warn(
                        f'VAD: No speech detected for {time_silent:.0f}s '
                        f'(threshold: {self.vad_silence_timeout}s). Stopping conversation.'
                    )
                    self._exit_speaking_state(reason="vad_silence_timeout")
            return  # Don't check idle watchdog while in SPEAKING state
        
        # ===== IDLE STATE FAILSAFE =====
        # Only applies when session is inactive (idle cleanup)
        if self.person_status != "red":
            # Person not present (BLUE or GREEN status)
            if self.last_red_status_time is None:
                # Start tracking absence time
                self.last_red_status_time = current_time
                self.get_logger().debug(
                    f'Idle watchdog: Person absent (status={self.person_status}), starting {int(self.auto_shutdown_timeout)}s timer'
                )
            
            # Calculate time since last RED status
            time_since_red = (current_time - self.last_red_status_time).nanoseconds / 1e9
            
            # Check if timeout exceeded
            if time_since_red > self.auto_shutdown_timeout:
                if self.session_active and not self.auto_shutdown_triggered:
                    self.get_logger().warn(
                        f'Idle watchdog: No person for {time_since_red:.0f}s '
                        f'(timeout: {self.auto_shutdown_timeout}s). Auto-stopping forgotten session.'
                    )
                    self._stop_session()
                    self.auto_shutdown_triggered = True
                elif not self.session_active and not self.auto_shutdown_triggered:
                    self.get_logger().debug('Idle watchdog: Timeout reached, session already inactive')
                    self.auto_shutdown_triggered = True
    
    def _play_audio_feedback(self, audio_file: Path):
        """
        Play audio feedback file asynchronously.
        
        The effective volume is: master_volume * audio_volume
        Where master_volume comes from the physical volume knob.
        
        Args:
            audio_file: Path to audio file to play
        """
        if not self.audio_feedback_enabled:
            return
        
        if not audio_file.exists():
            self.get_logger().warn(f'Audio file not found: {audio_file}')
            return
        
        try:
            # Calculate effective volume: master_volume * audio_volume
            effective_volume = self.master_volume * self.audio_volume
            
            # Play audio in background (non-blocking)
            # pan=stereo|c0=c0|c1=c0 duplicates mono to both L+R channels (for single earbud use)
            subprocess.Popen(
                ['ffplay', '-nodisp', '-autoexit', '-loglevel', 'error', 
                 '-af', f'pan=stereo|c0=c0|c1=c0,volume={effective_volume}', str(audio_file)],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL
            )
        except FileNotFoundError:
            # Try aplay as fallback (no volume control)
            try:
                subprocess.Popen(
                    ['aplay', '-q', str(audio_file)],
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.DEVNULL
                )
            except FileNotFoundError:
                self.get_logger().warn('No audio player found (ffplay or aplay)')
    
    def _start_session(self):
        """
        Helper method to start speech session.
        Used by both gesture triggers and auto-restart watchdog.
        Audio feedback is handled by session_status_callback.
        """
        self._call_service(self.start_session_client, 'start_session')
    
    def _stop_session(self):
        """
        Helper method to stop speech session.
        Used by both gesture triggers and auto-shutdown watchdog.
        Audio feedback is handled by session_status_callback.
        """
        self._call_service(self.stop_session_client, 'stop_session')
    
    def _enter_intelligent_speaking_state(self):
        """Enter R2-D2 speaking state - continuous conversation until fist gesture."""
        self.intelligent_speaking_state = "speaking"
        self.intelligent_speaking_start_time = self.get_clock().now()
        self.intelligent_last_activity_time = self.get_clock().now()
        self.intelligent_mode_active = True
        
        self.get_logger().info(
            f'🤖 Entered R2-D2 SPEAKING state (use fist gesture to stop)'
        )
        
        # Start first turn immediately
        self._process_intelligent_turn()
    
    def _exit_intelligent_speaking_state(self, reason: str):
        """Exit R2-D2 speaking state and stop the conversation loop."""
        self.intelligent_speaking_state = "idle"
        self.intelligent_speaking_start_time = None
        self.intelligent_last_activity_time = None
        self.intelligent_mode_active = False
        self.intelligent_mode_processing = False
        
        # Cancel conversation loop timer
        if self.intelligent_loop_timer:
            self.intelligent_loop_timer.cancel()
            self.destroy_timer(self.intelligent_loop_timer)
            self.intelligent_loop_timer = None
        
        self.get_logger().info(f'🔇 Exited R2-D2 SPEAKING state (reason: {reason})')
        
        # Stop the REST session
        self._call_service(self.stop_intelligent_client, 'stop_intelligent_session')
        
        # Play stop beep (unless already played by fist gesture confirmation)
        if not self.stop_beep_already_played:
            self._play_audio_feedback(self.stop_beep_sound)
        else:
            self.get_logger().debug('Stop beep already played by fist gesture confirmation')
    
    def _process_intelligent_turn(self):
        """
        Process one turn in R2-D2 Mode.
        
        This triggers a complete conversation turn:
        Record → Whisper STT → LLM → TTS → Playback
        
        After completion, schedules the next turn if still in speaking state.
        """
        if self.intelligent_speaking_state != "speaking":
            self.get_logger().debug('R2-D2 turn skipped: not in speaking state')
            return
        
        if self.intelligent_mode_processing:
            self.get_logger().debug('R2-D2 turn skipped: already processing')
            return
        
        self.intelligent_mode_processing = True
        
        # Process turn (includes recording, STT, LLM, TTS, playback)
        if not self.process_intelligent_turn_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('R2-D2 mode process_turn service not available')
            self.intelligent_mode_processing = False
            self._exit_intelligent_speaking_state(reason="service_unavailable")
            return
        
        request = Trigger.Request()
        future = self.process_intelligent_turn_client.call_async(request)
        
        # Add callback to handle completion and schedule next turn
        future.add_done_callback(self._intelligent_turn_callback)
    
    def _intelligent_turn_callback(self, future):
        """
        Handle R2-D2 mode turn completion.
        
        On success, schedules the next turn to continue conversation.
        On failure, exits speaking state.
        """
        self.intelligent_mode_processing = False
        
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(f'✅ R2-D2 turn complete: {response.message}')
                self.intelligent_last_activity_time = self.get_clock().now()
                
                # Check if still in speaking state
                if self.intelligent_speaking_state == "speaking":
                    # Schedule next turn after a short pause (1 second)
                    # This gives user time to react or make fist gesture
                    self.get_logger().info('🔄 R2-D2: Ready for next turn (listening...)')
                    
                    # Use a one-shot timer to schedule the next turn
                    if self.intelligent_loop_timer:
                        self.intelligent_loop_timer.cancel()
                        self.destroy_timer(self.intelligent_loop_timer)
                    
                    self.intelligent_loop_timer = self.create_timer(
                        0.5,  # 0.5 second pause before next recording
                        self._intelligent_loop_callback
                    )
                else:
                    self.get_logger().info('R2-D2: Speaking state ended, not scheduling next turn')
            else:
                self.get_logger().warn(f'⚠️  R2-D2 turn failed: {response.message}')
                # Check for empty recording (user didn't speak)
                if "Empty" in response.message or "no speech" in response.message.lower():
                    self.get_logger().info('🔇 R2-D2: No speech detected, waiting for user...')
                    # Still try again if in speaking state
                    if self.intelligent_speaking_state == "speaking":
                        if self.intelligent_loop_timer:
                            self.intelligent_loop_timer.cancel()
                            self.destroy_timer(self.intelligent_loop_timer)
                        self.intelligent_loop_timer = self.create_timer(
                            1.0,  # 1 second pause before retry
                            self._intelligent_loop_callback
                        )
                else:
                    # Real error - exit speaking state
                    self._exit_intelligent_speaking_state(reason=f"turn_failed: {response.message}")
        except Exception as e:
            self.get_logger().error(f'❌ R2-D2 turn error: {e}')
            self._exit_intelligent_speaking_state(reason=f"turn_error: {e}")
    
    def _intelligent_loop_callback(self):
        """Timer callback to process the next turn in the R2-D2 conversation loop."""
        # Cancel the timer (one-shot behavior)
        if self.intelligent_loop_timer:
            self.intelligent_loop_timer.cancel()
            self.destroy_timer(self.intelligent_loop_timer)
            self.intelligent_loop_timer = None
        
        # Only continue if still in speaking state
        if self.intelligent_speaking_state == "speaking":
            self._process_intelligent_turn()
        else:
            self.get_logger().debug('R2-D2 loop: Speaking state ended, stopping loop')


def main(args=None):
    """Main entry point for gesture intent node."""
    rclpy.init(args=args)
    
    node = GestureIntentNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

