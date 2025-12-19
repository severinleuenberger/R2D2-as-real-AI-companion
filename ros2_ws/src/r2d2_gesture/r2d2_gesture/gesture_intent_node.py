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
from std_msgs.msg import String
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
        self.declare_parameter('auto_shutdown_timeout_seconds', 300.0)  # 5 minutes
        self.declare_parameter('auto_restart_on_return', False)
        self.declare_parameter('audio_feedback_enabled', True)
        self.declare_parameter('audio_volume', 0.02)  # 0.0-1.0 (audio feedback volume) - 30% volume (from config/audio_params.yaml)
        self.declare_parameter('speaking_protection_seconds', 35.0)  # Consecutive non-RED during SPEAKING
        
        # Get parameters
        self.cooldown_start = self.get_parameter('cooldown_start_seconds').value
        self.cooldown_stop = self.get_parameter('cooldown_stop_seconds').value
        self.enabled = self.get_parameter('enabled').value
        self.auto_shutdown_enabled = self.get_parameter('auto_shutdown_enabled').value
        self.auto_shutdown_timeout = self.get_parameter('auto_shutdown_timeout_seconds').value
        self.auto_restart_on_return = self.get_parameter('auto_restart_on_return').value
        self.audio_feedback_enabled = self.get_parameter('audio_feedback_enabled').value
        self.audio_volume = self.get_parameter('audio_volume').value
        self.speaking_protection_timeout = self.get_parameter('speaking_protection_seconds').value
        
        # Audio feedback paths
        audio_assets_dir = Path.home() / 'dev' / 'r2d2' / 'ros2_ws' / 'src' / 'r2d2_audio' / 'r2d2_audio' / 'assets' / 'audio'
        self.start_beep_sound = audio_assets_dir / 'Voicy_R2-D2 - 16.mp3'
        self.stop_beep_sound = audio_assets_dir / 'Voicy_R2-D2 - 20.mp3'
        
        # State tracking
        self.person_status = None  # "red", "blue", or "green"
        self.session_active = False
        self.last_trigger_time = 0.0
        self.last_red_status_time = None  # Watchdog: time when RED status last seen
        self.auto_shutdown_triggered = False  # Watchdog: flag to prevent repeated shutdowns
        
        # SPEAKING state tracking (conversation protection)
        self.speaking_state = "idle"  # "idle" or "speaking"
        self.speaking_start_time = None
        self.consecutive_nonred_start_time = None
        self.last_red_in_speaking_time = None
        
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
        
        # Create service clients
        self.start_session_client = self.create_client(
            Trigger,
            '/r2d2/speech/start_session'
        )
        
        self.stop_session_client = self.create_client(
            Trigger,
            '/r2d2/speech/stop_session'
        )
        
        # Create watchdog timer (checks every 10 seconds)
        if self.auto_shutdown_enabled:
            self.watchdog_timer = self.create_timer(10.0, self.watchdog_callback)
        
        self.get_logger().info('Gesture Intent Node initialized')
        self.get_logger().info(f'Cooldown: start={self.cooldown_start}s, stop={self.cooldown_stop}s')
        self.get_logger().info(f'Enabled: {self.enabled}')
        self.get_logger().info(
            f'Auto-shutdown: enabled={self.auto_shutdown_enabled}, '
            f'timeout={self.auto_shutdown_timeout}s, '
            f'auto_restart={self.auto_restart_on_return}'
        )
        self.get_logger().info(f'Audio feedback: {self.audio_feedback_enabled}')
        self.get_logger().info(f'SPEAKING protection: {self.speaking_protection_timeout}s consecutive non-RED')
    
    def person_status_callback(self, msg):
        """
        Handle person status updates from audio notification node.
        
        Args:
            msg: String message containing JSON status data
        """
        try:
            status_data = json.loads(msg.data)
            old_status = self.person_status
            self.person_status = status_data.get('status', None)
            
            # Log status changes
            if old_status != self.person_status:
                self.get_logger().info(f'👤 Person status: {old_status} → {self.person_status}')
            
            # CRITICAL: Immediate stop on BLUE during active session
            if self.speaking_state == "speaking" and self.person_status == "blue":
                self.get_logger().warn(
                    '🔴 Person confirmed absent (BLUE status). Stopping session immediately.'
                )
                self._exit_speaking_state(reason="confirmed_absence_blue")
                return
            
            # Update timers based on status
            if self.person_status == "red":
                # Target person recognized - reset all timers
                if self.speaking_state == "speaking":
                    self.last_red_in_speaking_time = self.get_clock().now()
                    self.consecutive_nonred_start_time = None
                    self.get_logger().debug('SPEAKING: RED detected, grace timer reset')
                
                # Reset idle watchdog
                if self.last_red_status_time is not None:
                    self.get_logger().debug('Idle watchdog: RED detected, timer reset')
                self.last_red_status_time = None
                self.auto_shutdown_triggered = False
                
                # Auto-restart if enabled
                if self.auto_shutdown_triggered and self.auto_restart_on_return:
                    self.get_logger().info('👤 Person returned. Auto-restarting speech service.')
                    self._start_session()
            
            elif self.person_status == "green":
                # Unknown person - start grace period if in speaking state
                if self.speaking_state == "speaking" and self.consecutive_nonred_start_time is None:
                    self.consecutive_nonred_start_time = self.get_clock().now()
                    self.get_logger().info(
                        f'SPEAKING: GREEN (unknown) detected, starting {self.speaking_protection_timeout}s grace period'
                    )
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
                else:
                    # Session stopped (active → inactive or connected → disconnected)
                    self.get_logger().info(f'🔊 Session stopped (status={status_str})')
                    self._play_audio_feedback(self.stop_beep_sound)
                    
                self.get_logger().info(f'Session active changed: {old_active} → {self.session_active} (status={status_str})')
            else:
                self.get_logger().debug(f'Session active: {self.session_active} (status={status_str})')
        except json.JSONDecodeError as e:
            self.get_logger().warn(f'Failed to parse session status: {e}')
    
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
            self._enter_speaking_state()
            self.last_trigger_time = current_time
        
        elif gesture_name == "fist":
            # Stop gesture: only if speech active and cooldown elapsed
            if not self.session_active:
                self.get_logger().warn('❌ Stop gesture ignored: no active session')
                return
            
            if time_since_last < self.cooldown_stop:
                self.get_logger().warn(
                    f'❌ Stop gesture ignored: cooldown ({time_since_last:.1f}s < {self.cooldown_stop}s)'
                )
                return
            
            # Trigger stop session - Exit SPEAKING state
            self.get_logger().info('✊ Fist detected → Stopping conversation')
            self._exit_speaking_state(reason="user_fist_gesture")
            self.last_trigger_time = current_time
        else:
            self.get_logger().debug(f'Unknown gesture: {gesture_name}')
    
    def _enter_speaking_state(self):
        """Enter SPEAKING state when conversation starts."""
        self.speaking_state = "speaking"
        self.speaking_start_time = self.get_clock().now()
        self.last_red_in_speaking_time = self.get_clock().now()
        self.consecutive_nonred_start_time = None
        
        self.get_logger().info(
            f'🗣️  Entered SPEAKING state (protection: {self.speaking_protection_timeout}s consecutive non-RED)'
        )
        self._start_session()
    
    def _exit_speaking_state(self, reason: str):
        """Exit SPEAKING state when conversation ends."""
        self.speaking_state = "idle"
        self.speaking_start_time = None
        self.consecutive_nonred_start_time = None
        self.last_red_in_speaking_time = None
        
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
        
        Two separate timeout mechanisms:
        1. SPEAKING state: 35s grace period for GREEN (unknown person)
        2. IDLE state: 5-minute failsafe for forgotten sessions
        """
        if not self.auto_shutdown_enabled:
            return
        
        current_time = self.get_clock().now()
        
        # ===== SPEAKING STATE PROTECTION =====
        # Check if in SPEAKING state and grace period exceeded
        if self.speaking_state == "speaking":
            # Check 35s grace period for GREEN status
            if self.consecutive_nonred_start_time is not None:
                time_nonred = (current_time - self.consecutive_nonred_start_time).nanoseconds / 1e9
                
                # Log progress every 10 seconds
                if int(time_nonred) % 10 == 0 and int(time_nonred) > 0:
                    remaining = self.speaking_protection_timeout - time_nonred
                    if remaining > 0:
                        self.get_logger().info(
                            f'SPEAKING: GREEN for {int(time_nonred)}s / {int(self.speaking_protection_timeout)}s '
                            f'(grace expires in {int(remaining)}s if person doesn\'t return)'
                        )
                
                # Check if grace period exceeded
                if time_nonred > self.speaking_protection_timeout:
                    self.get_logger().warn(
                        f'SPEAKING: Unknown person (GREEN) for {time_nonred:.0f}s consecutive '
                        f'(threshold: {self.speaking_protection_timeout}s). Stopping conversation.'
                    )
                    self._exit_speaking_state(reason="green_grace_expired")
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
            
            # Check if timeout exceeded (5 minutes)
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
        
        Args:
            audio_file: Path to audio file to play
        """
        if not self.audio_feedback_enabled:
            return
        
        if not audio_file.exists():
            self.get_logger().warn(f'Audio file not found: {audio_file}')
            return
        
        try:
            # Play audio in background (non-blocking)
            subprocess.Popen(
                ['ffplay', '-nodisp', '-autoexit', '-loglevel', 'error', 
                 '-af', f'volume={self.audio_volume}', str(audio_file)],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL
            )
        except FileNotFoundError:
            # Try aplay as fallback
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

