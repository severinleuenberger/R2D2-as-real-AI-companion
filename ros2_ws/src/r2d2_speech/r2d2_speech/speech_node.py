#!/usr/bin/env python3
"""R2D2 Speech Node - ROS2 Lifecycle Node wrapper for speech system"""

import sys
import os
import asyncio
import logging
import threading
from typing import Optional, Dict, Any
from pathlib import Path

import rclpy
from rclpy.lifecycle import Node as LifecycleNode
from rclpy.lifecycle import State, TransitionCallbackReturn
from std_msgs.msg import String
from std_srvs.srv import Trigger

# Add existing r2d2_speech package to path
sys.path.insert(0, str(Path.home() / 'dev' / 'r2d2'))

# Import existing speech system components
from r2d2_speech.config import get_config
from r2d2_speech.storage import init_db, create_session
from r2d2_speech.realtime import RealtimeClient, EventRouter, TranscriptHandler
from r2d2_speech.utils import AudioStreamManager

# Import ROS2 bridge
from .ros2_bridge import ROS2TranscriptHandler, ROS2StatusPublisher, ros2_params_to_config

logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)


class SpeechNode(LifecycleNode):
    """ROS2 lifecycle node for speech system"""
    
    def __init__(self):
        super().__init__('speech_node')
        self.get_logger().info("R2D2 Speech Node initializing...")
        
        # Declare parameters
        self.declare_parameter('openai_api_key', '')
        self.declare_parameter('realtime_model', 'gpt-4o-realtime-preview-2024-12-17')
        self.declare_parameter('realtime_voice', 'alloy')
        self.declare_parameter('mic_device', '')
        self.declare_parameter('mic_native_sample_rate', 48000)
        self.declare_parameter('mic_sample_rate', 24000)
        self.declare_parameter('sink_device', 'default')
        self.declare_parameter('db_path', str(Path.home() / 'dev' / 'r2d2' / 'r2d2_speech' / 'data' / 'conversations.db'))
        self.declare_parameter('auto_start', True)
        self.declare_parameter('instructions', 'You are a helpful assistant.')
        
        # State
        self.config: Optional[Dict[str, Any]] = None
        self.session_id: Optional[str] = None
        self.client: Optional[RealtimeClient] = None
        self.audio_manager: Optional[AudioStreamManager] = None
        self.event_router: Optional[EventRouter] = None
        self.transcript_handler: Optional[TranscriptHandler] = None
        self.ros2_transcript_handler: Optional[ROS2TranscriptHandler] = None
        self.status_publisher: Optional[ROS2StatusPublisher] = None
        
        # Asyncio integration
        self.asyncio_loop: Optional[asyncio.AbstractEventLoop] = None
        self.asyncio_thread: Optional[threading.Thread] = None
        self.speech_task: Optional[asyncio.Future] = None
        self.event_task: Optional[asyncio.Future] = None
        
        # Subscribers & services
        self.command_sub = None
        self.prompt_sub = None
        self.start_session_srv = None
        self.stop_session_srv = None
        
        self.get_logger().info("Speech Node initialized")
    
    def on_configure(self, state: State) -> TransitionCallbackReturn:
        """Configure: load parameters, initialize database"""
        self.get_logger().info("Configuring...")
        
        try:
            self.config = ros2_params_to_config(self)
            
            # Try loading API key from environment if not in params
            if not self.config['openai_api_key']:
                try:
                    env_config = get_config()
                    self.config['openai_api_key'] = env_config['openai_api_key']
                except Exception as e:
                    self.get_logger().error(f"No API key: {e}")
                    return TransitionCallbackReturn.FAILURE
            
            if not self.config['openai_api_key'].startswith('sk-'):
                self.get_logger().error("Invalid API key format")
                return TransitionCallbackReturn.FAILURE
            
            # Initialize database
            init_db(self.config['db_path'])
            self.get_logger().info(f"Database: {self.config['db_path']}")
            
            # Create ROS2 interfaces
            self.status_publisher = ROS2StatusPublisher(self)
            
            self.command_sub = self.create_subscription(
                String, '/r2d2/speech/commands', self._command_callback, 10)
            self.prompt_sub = self.create_subscription(
                String, '/r2d2/speech/assistant_prompt', self._prompt_callback, 10)
            
            self.start_session_srv = self.create_service(
                Trigger, '/r2d2/speech/start_session', self._start_session_callback)
            self.stop_session_srv = self.create_service(
                Trigger, '/r2d2/speech/stop_session', self._stop_session_callback)
            
            self.get_logger().info("Configuration complete")
            return TransitionCallbackReturn.SUCCESS
            
        except Exception as e:
            self.get_logger().error(f"Configuration failed: {e}")
            return TransitionCallbackReturn.FAILURE
    
    def on_activate(self, state: State) -> TransitionCallbackReturn:
        """Activate: start speech system"""
        self.get_logger().info("Activating...")
        
        try:
            self._start_asyncio_loop()
            
            auto_start = self.get_parameter('auto_start').value
            if auto_start:
                success = self._run_in_asyncio_loop(self._start_speech_system())
                if not success:
                    self.get_logger().error("Failed to start speech system")
                    return TransitionCallbackReturn.FAILURE
                self.get_logger().info("Speech system started")
            else:
                self.get_logger().info("Auto-start disabled")
            
            self.status_publisher.publish_status("active")
            self.get_logger().info("Activation complete")
            return TransitionCallbackReturn.SUCCESS
            
        except Exception as e:
            self.get_logger().error(f"Activation failed: {e}")
            return TransitionCallbackReturn.FAILURE
    
    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        """Deactivate: stop streaming"""
        self.get_logger().info("Deactivating...")
        
        try:
            if self.client and self.client.connected:
                self._run_in_asyncio_loop(self._stop_speech_system())
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
        self.get_logger().info("Cleaning up...")
        
        try:
            if self.command_sub:
                self.destroy_subscription(self.command_sub)
            if self.prompt_sub:
                self.destroy_subscription(self.prompt_sub)
            if self.start_session_srv:
                self.destroy_service(self.start_session_srv)
            if self.stop_session_srv:
                self.destroy_service(self.stop_session_srv)
            
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
            if self.client and self.client.connected:
                self._run_in_asyncio_loop(self._stop_speech_system())
            self._stop_asyncio_loop()
            return TransitionCallbackReturn.SUCCESS
        except Exception as e:
            self.get_logger().error(f"Shutdown failed: {e}")
            return TransitionCallbackReturn.FAILURE
    
    async def _start_speech_system(self) -> bool:
        """Start the speech system"""
        try:
            from datetime import datetime
            
            self.session_id = f"ros2-{datetime.now().strftime('%Y%m%d-%H%M%S')}"
            create_session(
                self.config['db_path'], self.session_id,
                metadata={'model': self.config['realtime_model'], 'voice': self.config['realtime_voice'], 'source': 'ros2'})
            self.get_logger().info(f"Session: {self.session_id}")
            
            self.client = RealtimeClient(
                api_key=self.config['openai_api_key'],
                model=self.config['realtime_model'],
                voice=self.config['realtime_voice'])
            
            await self.client.connect()
            self.get_logger().info("Connected to API")
            
            instructions = self.get_parameter('instructions').value
            await self.client.create_session(instructions=instructions, temperature=0.8)
            self.get_logger().info("Session created")
            
            self.audio_manager = AudioStreamManager(self.config, self.client)
            self.get_logger().info(f"Audio: {self.audio_manager.device_info['name']}")
            
            self.audio_manager.start_playback()
            self.get_logger().info("Playback started")
            
            self.transcript_handler = TranscriptHandler(self.config['db_path'], self.session_id)
            self.ros2_transcript_handler = ROS2TranscriptHandler(self.transcript_handler, self)
            
            self.event_router = EventRouter(
                self.client, self.ros2_transcript_handler,
                audio_playback=self.audio_manager.playback)
            
            self.event_task = asyncio.create_task(self.event_router.start_listening())
            await self.audio_manager.start()
            self.speech_task = asyncio.create_task(self._stream_audio_loop())
            
            self.get_logger().info("✓ Speech system running")
            self.status_publisher.publish_status("connected", self.session_id)
            return True
            
        except Exception as e:
            self.get_logger().error(f"Start failed: {e}")
            return False
    
    async def _stop_speech_system(self) -> None:
        """Stop the speech system"""
        try:
            self.get_logger().info("Stopping...")
            
            if self.speech_task:
                self.speech_task.cancel()
                try:
                    await self.speech_task
                except asyncio.CancelledError:
                    pass
            
            if self.event_task:
                self.event_router.stop()
                self.event_task.cancel()
                try:
                    await self.event_task
                except asyncio.CancelledError:
                    pass
            
            if self.audio_manager:
                if self.audio_manager.is_running:
                    await self.audio_manager.stop()
                if self.audio_manager.playback:
                    self.audio_manager.stop_playback()
            
            if self.client and self.client.connected:
                await self.client.disconnect()
            
            self.status_publisher.publish_status("disconnected")
            self.get_logger().info("Stopped")
            
        except Exception as e:
            self.get_logger().error(f"Stop error: {e}")
    
    async def _stream_audio_loop(self) -> None:
        """Continuous audio streaming"""
        try:
            while self.audio_manager and self.audio_manager.is_running:
                success = await self.audio_manager.process_and_send()
                if not success:
                    await asyncio.sleep(0.01)
                    continue
                await asyncio.sleep(0.01)
        except asyncio.CancelledError:
            pass
        except Exception as e:
            self.get_logger().error(f"Streaming error: {e}")
    
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
        return future.result(timeout=30.0)
    
    def _command_callback(self, msg: String) -> None:
        """Handle commands"""
        command = msg.data.lower()
        self.get_logger().info(f"Command: {command}")
        if command in ['mute', 'unmute', 'pause', 'resume']:
            self.get_logger().warn(f"'{command}' not yet implemented")
        else:
            self.get_logger().warn(f"Unknown command: {command}")
    
    def _prompt_callback(self, msg: String) -> None:
        """Handle prompt updates"""
        instructions = msg.data
        self.get_logger().info(f"New instructions: {instructions[:50]}...")
        if self.client and self.client.connected:
            try:
                coro = self.client.create_session(instructions=instructions, temperature=0.8)
                self._run_in_asyncio_loop(coro)
                self.get_logger().info("Instructions updated")
            except Exception as e:
                self.get_logger().error(f"Update failed: {e}")
        else:
            self.get_logger().warn("Not connected")
    
    def _start_session_callback(self, request, response):
        """Service: start session"""
        self.get_logger().info("Service: start_session")
        try:
            if self.client and self.client.connected:
                response.success = True
                response.message = "Already running"
            else:
                success = self._run_in_asyncio_loop(self._start_speech_system())
                response.success = success
                response.message = "Started" if success else "Failed"
        except Exception as e:
            response.success = False
            response.message = str(e)
        return response
    
    def _stop_session_callback(self, request, response):
        """Service: stop session"""
        self.get_logger().info("Service: stop_session")
        try:
            if self.client and self.client.connected:
                self._run_in_asyncio_loop(self._stop_speech_system())
                response.success = True
                response.message = "Stopped"
            else:
                response.success = True
                response.message = "No active session"
        except Exception as e:
            response.success = False
            response.message = str(e)
        return response


def main(args=None):
    """Main entry point"""
    rclpy.init(args=args)
    node = SpeechNode()
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


