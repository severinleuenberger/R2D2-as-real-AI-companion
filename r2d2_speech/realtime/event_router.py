"""
Event Router
=============

Routes OpenAI Realtime API server events to appropriate handlers.

Handles:
- conversation.item.input_audio_transcription.completed (user transcripts)
- response.output_audio_transcript.delta/done (assistant transcripts)
- response.output_audio.delta/done (audio playback, future)
- error events
"""

import asyncio
import logging
from typing import Dict, Any, Optional, Callable

logger = logging.getLogger(__name__)


class EventRouter:
    """
    Routes Realtime API server events to handlers.
    
    Usage:
        router = EventRouter(realtime_client, transcript_handler)
        await router.start_listening()
    """
    
    def __init__(
        self,
        realtime_client,
        transcript_handler,
        audio_playback=None,
    ):
        """
        Initialize event router.
        
        Args:
            realtime_client: RealtimeClient instance
            transcript_handler: TranscriptHandler instance
            audio_playback: Optional AudioPlayback instance for assistant audio
        """
        self.client = realtime_client
        self.transcript_handler = transcript_handler
        self.audio_playback = audio_playback
        
        self.running = False
        
        # Accumulator for delta events
        self.assistant_transcript_buffer = ""
        
        logger.info("EventRouter initialized")
    
    async def start_listening(self) -> None:
        """
        Start event loop to receive and route events.
        
        This is a blocking call that runs until stopped or connection closes.
        """
        self.running = True
        logger.info("EventRouter started, listening for events...")
        
        try:
            while self.running and self.client.connected:
                # Receive next event
                event = await self.client.receive_event()
                
                if event is None:
                    # Connection closed
                    logger.info("Connection closed, stopping event router")
                    break
                
                # Route event
                await self.route_event(event)
                
        except Exception as e:
            logger.error(f"Event router error: {e}", exc_info=True)
        finally:
            self.running = False
            logger.info("EventRouter stopped")
    
    async def route_event(self, event: Dict[str, Any]) -> None:
        """
        Route a server event to the appropriate handler.
        
        Args:
            event: Event dictionary from Realtime API
        """
        event_type = event.get("type", "unknown")
        
        try:
            # User transcript events
            if event_type == "conversation.item.input_audio_transcription.completed":
                await self._handle_user_transcript(event)
            
            # Assistant transcript events (handle both old and new event names)
            elif event_type in ["response.output_audio_transcript.delta", "response.audio_transcript.delta"]:
                await self._handle_assistant_transcript_delta(event)
            
            elif event_type in ["response.output_audio_transcript.done", "response.audio_transcript.done"]:
                await self._handle_assistant_transcript_done(event)
            
            # Audio events (playback) (handle both old and new event names)
            elif event_type in ["response.output_audio.delta", "response.audio.delta"]:
                await self._handle_audio_delta(event)
            
            elif event_type in ["response.output_audio.done", "response.audio.done"]:
                await self._handle_audio_done(event)
            
            # Session events
            elif event_type == "session.created":
                logger.info("Session created")
            
            elif event_type == "session.updated":
                logger.info("Session updated")
            
            # Response events
            elif event_type == "response.created":
                logger.info("Response created")
            
            elif event_type == "response.done":
                logger.info("Response complete")
                # Log full response structure for debugging
                response_obj = event.get("response", {})
                output_items = response_obj.get("output", [])
                logger.info(f"  Response has {len(output_items)} output items")
                for i, item in enumerate(output_items):
                    item_type = item.get("type", "unknown")
                    logger.info(f"    Item [{i}]: type={item_type}")
                    if item_type == "message":
                        content = item.get("content", [])
                        logger.info(f"      Content parts: {len(content)}")
                        for j, part in enumerate(content):
                            part_type = part.get("type", "unknown")
                            logger.info(f"        Part [{j}]: {part_type}")
                            if part_type == "audio":
                                has_transcript = bool(part.get("transcript"))
                                logger.info(f"          Has transcript: {has_transcript}")
            
            # VAD events
            elif event_type == "input_audio_buffer.speech_started":
                logger.info("🎤 Speech detected - listening...")
            
            elif event_type == "input_audio_buffer.speech_stopped":
                logger.info("🔇 Speech ended - processing...")
            
            elif event_type == "input_audio_buffer.committed":
                logger.info("✓ Audio buffer committed")
            
            # Error events
            elif event_type == "error":
                self._handle_error(event)
            
            # Other events (log for debugging)
            else:
                logger.info(f"Unhandled event: {event_type}")
                
        except Exception as e:
            logger.error(f"Error routing event {event_type}: {e}", exc_info=True)
    
    async def _handle_user_transcript(self, event: Dict[str, Any]) -> None:
        """
        Handle user transcript event.
        
        Event: conversation.item.input_audio_transcription.completed
        """
        try:
            # Extract transcript from top-level field
            transcript = event.get("transcript", "")
            
            if not transcript:
                logger.warning("User transcript event has no transcript field")
                return
            
            logger.info(f"User transcript: {transcript}")
            
            # Route to transcript handler
            await self.transcript_handler.handle_user_transcript(event)
            
        except Exception as e:
            logger.error(f"Error handling user transcript: {e}", exc_info=True)
    
    async def _handle_assistant_transcript_delta(self, event: Dict[str, Any]) -> None:
        """
        Handle assistant transcript delta event.
        
        Event: response.output_audio_transcript.delta
        """
        try:
            delta = event.get("delta", "")
            if delta:
                self.assistant_transcript_buffer += delta
                logger.debug(f"Assistant delta: {delta}")
                
        except Exception as e:
            logger.error(f"Error handling assistant transcript delta: {e}", exc_info=True)
    
    async def _handle_assistant_transcript_done(self, event: Dict[str, Any]) -> None:
        """
        Handle assistant transcript done event.
        
        Event: response.output_audio_transcript.done
        """
        try:
            # Get final transcript (either from event or buffer)
            transcript = event.get("transcript", self.assistant_transcript_buffer)
            
            if not transcript:
                logger.warning("Assistant transcript done but no text")
                return
            
            logger.info(f"Assistant transcript: {transcript}")
            
            # Route to transcript handler
            await self.transcript_handler.handle_assistant_transcript(event)
            
            # Clear buffer
            self.assistant_transcript_buffer = ""
            
        except Exception as e:
            logger.error(f"Error handling assistant transcript done: {e}", exc_info=True)
    
    async def _handle_audio_delta(self, event: Dict[str, Any]) -> None:
        """
        Handle audio delta event (assistant audio playback).
        
        Event: response.output_audio.delta or response.audio.delta
        """
        try:
            # Get base64 audio chunk (try both possible field names)
            audio_delta = event.get("delta", event.get("audio", ""))
            
            if not audio_delta:
                logger.warning("Audio delta event received but no audio data found")
                return
            
            # Play audio chunk if playback is available
            if self.audio_playback and self.audio_playback.is_running:
                self.audio_playback.play_chunk(audio_delta)
                logger.debug(f"Playing audio chunk ({len(audio_delta)} chars base64)")
            else:
                logger.warning("Audio delta received but playback not active")
                
        except Exception as e:
            logger.error(f"Error handling audio delta: {e}", exc_info=True)
    
    async def _handle_audio_done(self, event: Dict[str, Any]) -> None:
        """
        Handle audio done event.
        
        Event: response.output_audio.done
        """
        try:
            # Flush playback buffer
            if self.audio_playback and self.audio_playback.is_running:
                self.audio_playback.flush()
            
            logger.debug("Audio playback complete")
            
        except Exception as e:
            logger.error(f"Error handling audio done: {e}", exc_info=True)
    
    def _handle_error(self, event: Dict[str, Any]) -> None:
        """
        Handle error event.
        
        Event: error
        """
        # Try different error structures
        error_code = event.get("code", event.get("error", {}).get("code", "unknown"))
        error_message = event.get("message", event.get("error", {}).get("message", "No error message"))
        
        # Log the full event for debugging
        logger.error(f"Realtime API error [{error_code}]: {error_message}")
        logger.error(f"Full error event: {event}")
    
    def stop(self) -> None:
        """
        Stop the event router.
        """
        self.running = False
        logger.info("EventRouter stop requested")


if __name__ == "__main__":
    # Test event routing with mock data
    import json
    
    # Mock handlers
    class MockTranscriptHandler:
        async def handle_user_transcript(self, event):
            print(f"✓ User transcript: {event.get('transcript')}")
        
        async def handle_assistant_transcript(self, event):
            print(f"✓ Assistant transcript: {event.get('transcript', 'N/A')}")
    
    class MockClient:
        connected = True
        
        def __init__(self):
            self.events = [
                {
                    "type": "conversation.item.input_audio_transcription.completed",
                    "transcript": "Hello, how are you?",
                    "item": {"id": "item_001"}
                },
                {
                    "type": "response.output_audio_transcript.delta",
                    "delta": "I'm doing "
                },
                {
                    "type": "response.output_audio_transcript.delta",
                    "delta": "well, thank you!"
                },
                {
                    "type": "response.output_audio_transcript.done",
                    "transcript": "I'm doing well, thank you!"
                },
            ]
            self.index = 0
        
        async def receive_event(self):
            if self.index >= len(self.events):
                self.connected = False
                return None
            event = self.events[self.index]
            self.index += 1
            return event
    
    async def test_router():
        client = MockClient()
        handler = MockTranscriptHandler()
        router = EventRouter(client, handler)
        
        print("Testing EventRouter...")
        await router.start_listening()
        print("✓ EventRouter test complete")
    
    asyncio.run(test_router())

