#!/usr/bin/env python3
"""
R2D2 REST Speech Node - ROS2 Lifecycle Node for "Intelligent Mode"

Uses OpenAI REST APIs for turn-based conversation:
- Whisper API for speech-to-text
- Chat Completions API (o1-preview or other models) for response generation
- TTS API for text-to-speech

Triggered by open_hand gesture for more thoughtful, intelligent responses.
"""

import sys
import os
import asyncio
import logging
import threading
from typing import Optional, Dict, Any
from pathlib import Path
from datetime import datetime

import rclpy
from rclpy.lifecycle import Node as LifecycleNode
from rclpy.lifecycle import State, TransitionCallbackReturn
from std_msgs.msg import String
from std_srvs.srv import Trigger

# Add existing r2d2_speech package to path
sys.path.insert(0, str(Path.home() / 'dev' / 'r2d2'))

# Import REST speech client
from r2d2_speech.rest_api import RestSpeechClient
from r2d2_speech.config import get_config
from r2d2_speech.storage import init_db, create_session, insert_message

# Import ROS2 bridge
from .ros2_bridge import ROS2StatusPublisher

logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)


class RestSpeechNode(LifecycleNode):
    """ROS2 lifecycle node for REST-based intelligent speech mode"""
    
    def __init__(self):
        super().__init__('rest_speech_node')
        self.get_logger().info("R2D2 REST Speech Node (Intelligent Mode) initializing...")
        
        # Declare parameters
        self.declare_parameter('openai_api_key', '')
        
        # LLM parameters
        self.declare_parameter('intelligent_model', 'o1-preview')
        self.declare_parameter('intelligent_instructions', 
            'You are R2-D2, the wise and thoughtful astromech droid. '
            'Take time to think before responding. Provide insightful, '
            'well-reasoned answers. Be helpful but maintain your distinctive '
            'personality. Speak with wisdom and depth.')
        self.declare_parameter('intelligent_temperature', 0.7)
        
        # STT parameters
        self.declare_parameter('stt_model', 'whisper-1')
        
        # TTS parameters
        self.declare_parameter('tts_model', 'tts-1')
        self.declare_parameter('tts_voice', 'nova')
        self.declare_parameter('tts_speed', 1.0)
        
        # Audio parameters
        self.declare_parameter('mic_device', '')
        self.declare_parameter('mic_device_index', 0)
        self.declare_parameter('silence_threshold', 500.0)
        self.declare_parameter('silence_duration', 1.5)
        self.declare_parameter('output_device', 'default')
        
        # Database
        self.declare_parameter('db_path', 
            str(Path.home() / 'dev' / 'r2d2' / 'r2d2_speech' / 'data' / 'conversations.db'))
        
        # State
        self.config: Optional[Dict[str, Any]] = None
        self.client: Optional[RestSpeechClient] = None
        self.session_id: Optional[str] = None
        self.is_processing: bool = False
        
        # Asyncio integration
        self.asyncio_loop: Optional[asyncio.AbstractEventLoop] = None
        self.asyncio_thread: Optional[threading.Thread] = None
        
        # ROS2 interfaces
        self.status_publisher: Optional[ROS2StatusPublisher] = None
        self.user_transcript_pub = None
        self.assistant_transcript_pub = None
        
        # Services
        self.start_session_srv = None
        self.stop_session_srv = None
        self.process_turn_srv = None
        
        self.get_logger().info("REST Speech Node initialized")
    
    def on_configure(self, state: State) -> TransitionCallbackReturn:
        """Configure: load parameters, initialize database"""
        self.get_logger().info("Configuring REST Speech Node...")
        
        try:
            # Load API key from parameter or environment
            api_key = self.get_parameter('openai_api_key').value
            
            if not api_key:
                try:
                    env_config = get_config()
                    api_key = env_config['openai_api_key']
                except Exception as e:
                    self.get_logger().error(f"No API key: {e}")
                    return TransitionCallbackReturn.FAILURE
            
            if not api_key.startswith('sk-'):
                self.get_logger().error("Invalid API key format")
                return TransitionCallbackReturn.FAILURE
            
            # Build configuration
            self.config = {
                'openai_api_key': api_key,
                'intelligent_model': self.get_parameter('intelligent_model').value,
                'intelligent_instructions': self.get_parameter('intelligent_instructions').value,
                'intelligent_temperature': self.get_parameter('intelligent_temperature').value,
                'stt_model': self.get_parameter('stt_model').value,
                'tts_model': self.get_parameter('tts_model').value,
                'tts_voice': self.get_parameter('tts_voice').value,
                'tts_speed': self.get_parameter('tts_speed').value,
                'mic_device_index': self.get_parameter('mic_device_index').value,
                'silence_threshold': self.get_parameter('silence_threshold').value,
                'silence_duration': self.get_parameter('silence_duration').value,
                'output_device': self.get_parameter('output_device').value,
                'db_path': self.get_parameter('db_path').value,
            }
            
            # Initialize database
            init_db(self.config['db_path'])
            self.get_logger().info(f"Database: {self.config['db_path']}")
            
            # Create ROS2 interfaces
            self.status_publisher = ROS2StatusPublisher(self)
            
            self.user_transcript_pub = self.create_publisher(
                String, '/r2d2/speech/intelligent/user_transcript', 10)
            self.assistant_transcript_pub = self.create_publisher(
                String, '/r2d2/speech/intelligent/assistant_transcript', 10)
            
            # Create services
            self.start_session_srv = self.create_service(
                Trigger, '/r2d2/speech/intelligent/start_session', 
                self._start_session_callback)
            self.stop_session_srv = self.create_service(
                Trigger, '/r2d2/speech/intelligent/stop_session',
                self._stop_session_callback)
            self.process_turn_srv = self.create_service(
                Trigger, '/r2d2/speech/intelligent/process_turn',
                self._process_turn_callback)
            
            self.get_logger().info("Configuration complete")
            self.get_logger().info(f"  LLM: {self.config['intelligent_model']}")
            self.get_logger().info(f"  STT: {self.config['stt_model']}")
            self.get_logger().info(f"  TTS: {self.config['tts_model']} ({self.config['tts_voice']})")
            
            return TransitionCallbackReturn.SUCCESS
            
        except Exception as e:
            self.get_logger().error(f"Configuration failed: {e}")
            import traceback
            traceback.print_exc()
            return TransitionCallbackReturn.FAILURE
    
    def on_activate(self, state: State) -> TransitionCallbackReturn:
        """Activate: start asyncio loop and create client"""
        self.get_logger().info("Activating REST Speech Node...")
        
        try:
            self._start_asyncio_loop()
            
            # Create REST speech client
            self.client = RestSpeechClient(
                api_key=self.config['openai_api_key'],
                instructions=self.config['intelligent_instructions'],
                model=self.config['intelligent_model'],
                stt_model=self.config['stt_model'],
                tts_model=self.config['tts_model'],
                tts_voice=self.config['tts_voice'],
                tts_speed=self.config['tts_speed'],
                temperature=self.config['intelligent_temperature'],
                device_index=self.config['mic_device_index'],
                silence_threshold=self.config['silence_threshold'],
                silence_duration=self.config['silence_duration'],
                output_device=self.config['output_device'],
            )
            
            self.status_publisher.publish_status("active", details={"mode": "intelligent"})
            self.get_logger().info("REST Speech Node activated")
            
            return TransitionCallbackReturn.SUCCESS
            
        except Exception as e:
            self.get_logger().error(f"Activation failed: {e}")
            import traceback
            traceback.print_exc()
            return TransitionCallbackReturn.FAILURE
    
    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        """Deactivate: stop client and asyncio loop"""
        self.get_logger().info("Deactivating REST Speech Node...")
        
        try:
            if self.client and self.client.is_active:
                self.client.end_session()
            
            if self.client:
                self._run_in_asyncio_loop(self.client.close())
                self.client = None
            
            self._stop_asyncio_loop()
            
            if self.status_publisher:
                self.status_publisher.publish_status("inactive")
            
            self.get_logger().info("Deactivation complete")
            return TransitionCallbackReturn.SUCCESS
            
        except Exception as e:
            self.get_logger().error(f"Deactivation failed: {e}")
            return TransitionCallbackReturn.FAILURE
    
    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        """Cleanup: release resources"""
        self.get_logger().info("Cleaning up REST Speech Node...")
        
        try:
            if self.user_transcript_pub:
                self.destroy_publisher(self.user_transcript_pub)
            if self.assistant_transcript_pub:
                self.destroy_publisher(self.assistant_transcript_pub)
            if self.start_session_srv:
                self.destroy_service(self.start_session_srv)
            if self.stop_session_srv:
                self.destroy_service(self.stop_session_srv)
            if self.process_turn_srv:
                self.destroy_service(self.process_turn_srv)
            
            self.config = None
            self.session_id = None
            self.status_publisher = None
            
            self.get_logger().info("Cleanup complete")
            return TransitionCallbackReturn.SUCCESS
            
        except Exception as e:
            self.get_logger().error(f"Cleanup failed: {e}")
            return TransitionCallbackReturn.FAILURE
    
    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        """Emergency shutdown"""
        self.get_logger().warn("Shutdown requested")
        
        try:
            if self.client:
                self.client.end_session()
                self._run_in_asyncio_loop(self.client.close())
            self._stop_asyncio_loop()
            return TransitionCallbackReturn.SUCCESS
        except Exception as e:
            self.get_logger().error(f"Shutdown failed: {e}")
            return TransitionCallbackReturn.FAILURE
    
    def _start_asyncio_loop(self) -> None:
        """Start asyncio loop in background thread"""
        if self.asyncio_loop and self.asyncio_loop.is_running():
            return
        
        def run_loop(loop):
            asyncio.set_event_loop(loop)
            loop.run_forever()
        
        self.asyncio_loop = asyncio.new_event_loop()
        self.asyncio_thread = threading.Thread(
            target=run_loop, args=(self.asyncio_loop,), daemon=True)
        self.asyncio_thread.start()
        self.get_logger().info("Asyncio loop started")
    
    def _stop_asyncio_loop(self) -> None:
        """Stop asyncio loop"""
        if not self.asyncio_loop:
            return
        self.asyncio_loop.call_soon_threadsafe(self.asyncio_loop.stop)
        if self.asyncio_thread:
            self.asyncio_thread.join(timeout=5.0)
        self.asyncio_loop = None
        self.asyncio_thread = None
    
    def _run_in_asyncio_loop(self, coro) -> Any:
        """Run coroutine in asyncio loop"""
        if not self.asyncio_loop:
            raise RuntimeError("Asyncio loop not running")
        future = asyncio.run_coroutine_threadsafe(coro, self.asyncio_loop)
        return future.result(timeout=300.0)  # Long timeout for o1
    
    def _start_session_callback(self, request, response):
        """Service: start intelligent conversation session"""
        self.get_logger().info("Service: start_session (intelligent mode)")
        
        try:
            if not self.client:
                response.success = False
                response.message = "Client not initialized"
                return response
            
            if self.client.is_active:
                response.success = True
                response.message = f"Session already active: {self.session_id}"
                return response
            
            # Create session
            self.session_id = self.client.start_session()
            
            # Store in database
            create_session(
                self.config['db_path'],
                self.session_id,
                metadata={
                    'mode': 'intelligent',
                    'model': self.config['intelligent_model'],
                    'voice': self.config['tts_voice'],
                    'source': 'ros2'
                }
            )
            
            self.status_publisher.publish_status(
                "session_started", 
                session_id=self.session_id,
                details={"mode": "intelligent"}
            )
            
            response.success = True
            response.message = f"Session started: {self.session_id}"
            
        except Exception as e:
            response.success = False
            response.message = str(e)
            self.get_logger().error(f"Start session failed: {e}")
        
        return response
    
    def _stop_session_callback(self, request, response):
        """Service: stop intelligent conversation session"""
        self.get_logger().info("Service: stop_session (intelligent mode)")
        
        try:
            if not self.client or not self.client.is_active:
                response.success = True
                response.message = "No active session"
                return response
            
            self.client.end_session()
            
            self.status_publisher.publish_status(
                "session_stopped",
                session_id=self.session_id,
                details={"mode": "intelligent"}
            )
            
            response.success = True
            response.message = f"Session stopped: {self.session_id}"
            
        except Exception as e:
            response.success = False
            response.message = str(e)
            self.get_logger().error(f"Stop session failed: {e}")
        
        return response
    
    def _process_turn_callback(self, request, response):
        """Service: process one conversation turn (record → STT → LLM → TTS)"""
        self.get_logger().info("Service: process_turn (intelligent mode)")
        
        try:
            if not self.client:
                response.success = False
                response.message = "Client not initialized"
                return response
            
            if not self.client.is_active:
                # Auto-start session if needed
                self._start_session_callback(request, response)
                if not response.success:
                    return response
            
            if self.is_processing:
                response.success = False
                response.message = "Already processing a turn"
                return response
            
            self.is_processing = True
            
            def on_transcription_complete(transcript):
                # Publish user transcript
                msg = String()
                msg.data = transcript
                self.user_transcript_pub.publish(msg)
                self.get_logger().info(f"User: {transcript}")
                
                # Store in database
                insert_message(
                    self.config['db_path'],
                    self.session_id,
                    role="user",
                    item_id=None,
                    response_id=None,
                    text=transcript
                )
            
            def on_response_ready(assistant_response):
                # Publish assistant transcript
                msg = String()
                msg.data = assistant_response
                self.assistant_transcript_pub.publish(msg)
                self.get_logger().info(f"Assistant: {assistant_response[:100]}...")
                
                # Store in database
                insert_message(
                    self.config['db_path'],
                    self.session_id,
                    role="assistant",
                    item_id=None,
                    response_id=None,
                    text=assistant_response
                )
            
            # Process turn
            result = self._run_in_asyncio_loop(
                self.client.process_turn(
                    on_transcription_complete=on_transcription_complete,
                    on_response_ready=on_response_ready,
                )
            )
            
            self.is_processing = False
            
            if result['success']:
                response.success = True
                response.message = f"Turn complete: {result['timings']['total']:.1f}s total"
            else:
                response.success = False
                response.message = result.get('error', 'Unknown error')
            
        except Exception as e:
            self.is_processing = False
            response.success = False
            response.message = str(e)
            self.get_logger().error(f"Process turn failed: {e}")
            import traceback
            traceback.print_exc()
        
        return response


def main(args=None):
    """Main entry point"""
    rclpy.init(args=args)
    node = RestSpeechNode()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

