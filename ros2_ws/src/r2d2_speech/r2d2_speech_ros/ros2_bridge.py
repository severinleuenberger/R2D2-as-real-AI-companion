#!/usr/bin/env python3
"""ROS2 Bridge - Connects existing speech system to ROS2"""

import logging
import subprocess
from pathlib import Path
from typing import Dict, Any, Optional
from std_msgs.msg import String
import json

logger = logging.getLogger(__name__)

# State file for tutor mode
TUTOR_MODE_FILE = Path.home() / 'dev' / 'r2d2' / 'data' / 'tutor_mode_active.txt'


class ROS2TranscriptHandler:
    """Wrapper around TranscriptHandler that publishes to ROS2 topics"""
    
    def __init__(self, original_handler, ros2_node):
        self.handler = original_handler
        self.node = ros2_node
        
        self.user_pub = ros2_node.create_publisher(
            String, '/r2d2/speech/user_transcript', 10)
        self.assistant_pub = ros2_node.create_publisher(
            String, '/r2d2/speech/assistant_transcript', 10)
        
        logger.info("ROS2TranscriptHandler initialized")
    
    async def handle_user_transcript(self, event: Dict[str, Any]) -> None:
        await self.handler.handle_user_transcript(event)
        transcript = event.get("transcript", "")
        if transcript:
            msg = String()
            msg.data = transcript
            self.user_pub.publish(msg)
            logger.debug(f"Published user transcript: {transcript}")
    
    async def handle_assistant_transcript(self, event: Dict[str, Any]) -> None:
        await self.handler.handle_assistant_transcript(event)
        transcript = event.get("transcript", self.handler.assistant_transcript_buffer)
        if transcript:
            # Detect and handle learning mode triggers
            clean_transcript = self._process_learning_mode_triggers(transcript)
            
            msg = String()
            msg.data = clean_transcript
            self.assistant_pub.publish(msg)
            logger.debug(f"Published assistant transcript: {clean_transcript}")
    
    def _process_learning_mode_triggers(self, text: str) -> str:
        """Detect [LEARNING_ON/OFF] triggers and toggle narrator service."""
        return process_learning_mode_triggers(text)
    
    async def handle_assistant_delta(self, event: Dict[str, Any]) -> None:
        await self.handler.handle_assistant_delta(event)
    
    def get_buffer_content(self) -> str:
        return self.handler.get_buffer_content()
    
    def clear_buffer(self) -> None:
        self.handler.clear_buffer()


class ROS2StatusPublisher:
    """Publishes session status updates to ROS2"""
    
    def __init__(self, ros2_node):
        self.node = ros2_node
        self.status_pub = ros2_node.create_publisher(
            String, '/r2d2/speech/session_status', 10)
        logger.info("ROS2StatusPublisher initialized")
    
    def publish_status(self, status: str, session_id: Optional[str] = None, 
                      details: Optional[Dict[str, Any]] = None) -> None:
        status_data = {
            "status": status,
            "timestamp": self.node.get_clock().now().to_msg().sec
        }
        if session_id:
            status_data["session_id"] = session_id
        if details:
            status_data["details"] = details
        
        msg = String()
        msg.data = json.dumps(status_data)
        self.status_pub.publish(msg)
        logger.info(f"Published status: {status}")


class ROS2VADPublisher:
    """Publishes Voice Activity Detection events to ROS2"""
    
    def __init__(self, ros2_node):
        self.node = ros2_node
        self.vad_pub = ros2_node.create_publisher(
            String, '/r2d2/speech/voice_activity', 10)
        self.current_state = "silent"  # Track current state to avoid duplicate publishes
        logger.info("ROS2VADPublisher initialized")
    
    def publish_speech_started(self) -> None:
        """Publish when user starts speaking"""
        if self.current_state != "speaking":
            self.current_state = "speaking"
            vad_data = {
                "state": "speaking",
                "timestamp": self.node.get_clock().now().to_msg().sec
            }
            msg = String()
            msg.data = json.dumps(vad_data)
            self.vad_pub.publish(msg)
            logger.info("VAD: User started speaking")
    
    def publish_speech_stopped(self) -> None:
        """Publish when user stops speaking"""
        if self.current_state != "silent":
            self.current_state = "silent"
            vad_data = {
                "state": "silent",
                "timestamp": self.node.get_clock().now().to_msg().sec
            }
            msg = String()
            msg.data = json.dumps(vad_data)
            self.vad_pub.publish(msg)
            logger.info("VAD: User stopped speaking")


def process_learning_mode_triggers(text: str) -> str:
    """
    Detect [LEARNING_ON/OFF] and [TUTOR_ON/OFF] triggers.
    Returns cleaned text with trigger tags removed.
    Standalone utility function for use by any speech handler.
    
    Learning Mode: Starts/stops narrator service (coding narration)
    Tutor Mode: Activates teaching personality (interactive Q&A)
    """
    # Learning mode triggers (narrator service)
    if "[LEARNING_ON]" in text:
        toggle_learning_mode(True)
        text = text.replace("[LEARNING_ON]", "").strip()
    elif "[LEARNING_OFF]" in text:
        toggle_learning_mode(False)
        text = text.replace("[LEARNING_OFF]", "").strip()
    
    # Tutor mode triggers (personality change)
    if "[TUTOR_ON]" in text:
        toggle_tutor_mode(True)
        text = text.replace("[TUTOR_ON]", "").strip()
    elif "[TUTOR_OFF]" in text:
        toggle_tutor_mode(False)
        text = text.replace("[TUTOR_OFF]", "").strip()
    
    return text


def toggle_learning_mode(enable: bool) -> None:
    """Start or stop the narrator service for coding narration."""
    action = "start" if enable else "stop"
    try:
        subprocess.run(
            ["sudo", "systemctl", action, "r2d2-narrator.service"],
            check=True, capture_output=True
        )
        logger.info(f"Learning mode {'ON' if enable else 'OFF'}")
    except subprocess.CalledProcessError as e:
        logger.error(f"Failed to toggle learning mode: {e.stderr.decode() if e.stderr else e}")
    except Exception as e:
        logger.error(f"Failed to toggle learning mode: {e}")


def toggle_tutor_mode(enable: bool) -> None:
    """
    Activate or deactivate tutor mode (teaching personality).
    Writes state to file for other components to read.
    """
    try:
        # Ensure data directory exists
        TUTOR_MODE_FILE.parent.mkdir(parents=True, exist_ok=True)
        
        # Write state
        TUTOR_MODE_FILE.write_text("true" if enable else "false")
        logger.info(f"Tutor mode {'ON' if enable else 'OFF'}")
    except Exception as e:
        logger.error(f"Failed to toggle tutor mode: {e}")


def is_tutor_mode_active() -> bool:
    """Check if tutor mode is currently active."""
    try:
        if TUTOR_MODE_FILE.exists():
            return TUTOR_MODE_FILE.read_text().strip().lower() == "true"
    except Exception:
        pass
    return False


def ros2_params_to_config(node) -> dict:
    """Convert ROS2 parameters to existing config format"""
    config = {
        'openai_api_key': node.get_parameter('openai_api_key').value,
        'realtime_model': node.get_parameter('realtime_model').value,
        'realtime_voice': node.get_parameter('realtime_voice').value,
        'mic_device': node.get_parameter('mic_device').value,
        'mic_native_sample_rate': node.get_parameter('mic_native_sample_rate').value,
        'mic_sample_rate': node.get_parameter('mic_sample_rate').value,
        'mic_channels': 1,
        'sink_device': node.get_parameter('sink_device').value,
        'db_path': node.get_parameter('db_path').value,
    }
    return config

