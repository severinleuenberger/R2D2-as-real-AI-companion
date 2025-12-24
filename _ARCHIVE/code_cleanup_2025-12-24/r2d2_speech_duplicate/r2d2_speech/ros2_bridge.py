#!/usr/bin/env python3
"""ROS2 Bridge - Connects existing speech system to ROS2"""

import logging
from typing import Dict, Any, Optional
from std_msgs.msg import String
import json

logger = logging.getLogger(__name__)


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
            msg = String()
            msg.data = transcript
            self.assistant_pub.publish(msg)
            logger.debug(f"Published assistant transcript: {transcript}")
    
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


