"""
OpenAI Realtime API Client
===========================

WebSocket client for OpenAI Realtime API (speech-to-speech).

Handles:
- WebSocket connection with Bearer auth
- Session configuration (transcription, VAD, voice)
- Sending/receiving events
- Audio buffer management
"""

import asyncio
import json
import logging
from typing import Dict, Optional, Any, Callable
import websockets
from websockets.client import WebSocketClientProtocol

logger = logging.getLogger(__name__)


class RealtimeClient:
    """
    WebSocket client for OpenAI Realtime API.
    
    Usage:
        client = RealtimeClient(api_key="sk-...", model="gpt-4o-realtime-preview")
        await client.connect()
        await client.create_session()
        await client.send_audio_frame(base64_audio)
        event = await client.receive_event()
        await client.disconnect()
    """
    
    def __init__(
        self,
        api_key: str,
        model: str = "gpt-4o-realtime-preview-2024-12-17",
        voice: str = "alloy"
    ):
        """
        Initialize Realtime client.
        
        Args:
            api_key: OpenAI API key (Bearer token)
            model: Realtime model name
            voice: Voice selection (alloy, echo, fable, onyx, nova, shimmer)
        """
        self.api_key = api_key
        self.model = model
        self.voice = voice
        
        # WebSocket connection
        self.ws: Optional[WebSocketClientProtocol] = None
        self.connected = False
        
        # WebSocket URL with model in query parameter
        self.url = f"wss://api.openai.com/v1/realtime?model={model}"
        
        logger.info(f"RealtimeClient initialized (model={model}, voice={voice})")
    
    async def connect(self) -> None:
        """
        Open WebSocket connection to OpenAI Realtime API.
        
        Raises:
            ConnectionError: If connection fails
            ValueError: If API key is invalid
        """
        try:
            # Connect with Bearer authentication header
            # websockets 15.x uses additional_headers with list of tuples
            logger.info(f"Connecting to {self.url}")
            self.ws = await websockets.connect(
                self.url,
                additional_headers=[
                    ("Authorization", f"Bearer {self.api_key}"),
                    ("OpenAI-Beta", "realtime=v1"),
                ],
                ping_interval=20,
                ping_timeout=10,
            )
            
            self.connected = True
            logger.info("✓ WebSocket connected")
            
        except websockets.exceptions.InvalidStatusCode as e:
            if e.status_code == 401:
                raise ValueError(f"Invalid API key (401 Unauthorized)")
            elif e.status_code == 403:
                raise ValueError(f"API access forbidden (403). Check Realtime API access.")
            else:
                raise ConnectionError(f"WebSocket connection failed: {e}")
        except Exception as e:
            raise ConnectionError(f"Failed to connect to Realtime API: {e}")
    
    async def create_session(
        self,
        instructions: str = "You are the R2D2 robot from the Star Wars Movie. Speak with a slightly synthetic, system-like delivery. Use short, precise sentences. Fast-paced, efficient cadence. Recognize emotions internally, but keep vocal emotional inflection minimal. Clear, clipped articulation. Avoid unnecessary pauses. Sound efficient and machine-like.",
        temperature: float = 0.8,
    ) -> None:
        """
        Create/update Realtime session with configuration.
        
        Args:
            instructions: System instructions for the assistant
            temperature: Temperature for response generation (0.0-1.0)
        """
        if not self.connected or not self.ws:
            raise ConnectionError("Not connected. Call connect() first.")
        
        # Session configuration
        session_config = {
            "type": "session.update",
            "session": {
                "modalities": ["audio", "text"],
                
                # Enable input audio transcription with Whisper model
                "input_audio_transcription": {
                    "model": "whisper-1"
                },
                
                # Server VAD with auto-response (adjusted for better detection)
                "turn_detection": {
                    "type": "server_vad",
                    "threshold": 0.3,  # More sensitive (lower = more sensitive)
                    "prefix_padding_ms": 300,
                    "silence_duration_ms": 700  # Longer silence to ensure natural pauses
                },
                
                # Voice and temperature
                "voice": self.voice,
                "temperature": temperature,
                "instructions": instructions,
            }
        }
        
        # Send session configuration
        await self.send_event(session_config)
        logger.info("✓ Session configuration sent")
        logger.info(f"  - input_audio_transcription: enabled")
        logger.info(f"  - output_audio_transcript: enabled")
        logger.info(f"  - voice: {self.voice}")
        logger.info(f"  - turn_detection: server_vad")
    
    async def disconnect(self) -> None:
        """
        Close WebSocket connection.
        """
        if self.ws:
            await self.ws.close()
            self.connected = False
            logger.info("✓ WebSocket disconnected")
    
    async def send_event(self, event: Dict[str, Any]) -> None:
        """
        Send a client event to the Realtime API.
        
        Args:
            event: Event dictionary (must include "type" field)
        """
        if not self.connected or not self.ws:
            raise ConnectionError("Not connected")
        
        event_json = json.dumps(event)
        await self.ws.send(event_json)
        logger.debug(f"→ Sent event: {event.get('type', 'unknown')}")
    
    async def receive_event(self) -> Optional[Dict[str, Any]]:
        """
        Receive and parse a server event from Realtime API.
        
        Returns:
            Event dictionary or None if connection closed
        """
        if not self.connected or not self.ws:
            raise ConnectionError("Not connected")
        
        try:
            message = await self.ws.recv()
            event = json.loads(message)
            logger.debug(f"← Received event: {event.get('type', 'unknown')}")
            return event
        except websockets.exceptions.ConnectionClosed:
            logger.warning("WebSocket connection closed")
            self.connected = False
            return None
        except json.JSONDecodeError as e:
            logger.error(f"Failed to parse event JSON: {e}")
            return None
    
    async def send_audio_frame(self, base64_audio: str) -> None:
        """
        Send audio frame via input_audio_buffer.append.
        
        Args:
            base64_audio: Base64-encoded PCM16 audio (24kHz, mono)
        """
        event = {
            "type": "input_audio_buffer.append",
            "audio": base64_audio
        }
        await self.send_event(event)
    
    async def commit_audio(self) -> None:
        """
        Commit audio buffer to trigger transcription.
        
        Note: This triggers transcription but does NOT auto-generate response.
        With server VAD enabled, responses are auto-generated.
        """
        event = {"type": "input_audio_buffer.commit"}
        await self.send_event(event)
        logger.debug("Audio buffer committed")
    
    async def create_response(self) -> None:
        """
        Explicitly request a response generation.
        
        Only needed if NOT using server VAD auto-response.
        """
        event = {"type": "response.create"}
        await self.send_event(event)
        logger.debug("Response creation requested")
    
    async def send_test_audio(self, duration_seconds: float = 1.0) -> None:
        """
        Send dummy PCM16 silence frames for testing.
        
        Args:
            duration_seconds: Duration of silence to send
        """
        import base64
        import struct
        
        # Generate PCM16 silence (16-bit signed integers, zero values)
        sample_rate = 24000
        channels = 1
        num_samples = int(sample_rate * duration_seconds)
        
        # Create silence: all zeros
        silence_samples = [0] * num_samples
        
        # Pack as 16-bit PCM (little-endian signed integers)
        pcm_data = struct.pack(f'<{num_samples}h', *silence_samples)
        
        # Base64 encode
        base64_audio = base64.b64encode(pcm_data).decode('utf-8')
        
        logger.info(f"Sending {duration_seconds}s of silence ({len(pcm_data)} bytes)")
        
        # Send in chunks (OpenAI recommends ~100ms chunks)
        chunk_samples = int(sample_rate * 0.1)  # 100ms chunks
        chunk_size = chunk_samples * 2  # 2 bytes per sample
        
        for i in range(0, len(pcm_data), chunk_size):
            chunk = pcm_data[i:i+chunk_size]
            chunk_base64 = base64.b64encode(chunk).decode('utf-8')
            await self.send_audio_frame(chunk_base64)
            await asyncio.sleep(0.1)  # Simulate real-time
        
        # Commit audio buffer
        await self.commit_audio()
        logger.info("✓ Test audio sent and committed")


if __name__ == "__main__":
    # Test client connection
    import os
    from dotenv import load_dotenv
    
    # Load API key
    load_dotenv(os.path.expanduser("~/.r2d2/.env"))
    api_key = os.getenv("OPENAI_API_KEY")
    
    if not api_key:
        print("✗ OPENAI_API_KEY not found in ~/.r2d2/.env")
        exit(1)
    
    async def test_connection():
        client = RealtimeClient(api_key=api_key)
        
        try:
            print("Connecting to Realtime API...")
            await client.connect()
            print("✓ Connected")
            
            print("Creating session...")
            await client.create_session()
            print("✓ Session created")
            
            print("Disconnecting...")
            await client.disconnect()
            print("✓ Disconnected")
            
        except Exception as e:
            print(f"✗ Error: {e}")
            if client.connected:
                await client.disconnect()
    
    asyncio.run(test_connection())

