"""
Transcript Handler
==================

Extracts and persists user and assistant transcripts from Realtime API events.

Immediately writes to database when events arrive (not batched).
"""

import logging
from typing import Dict, Any, Optional

# Import storage module
try:
    from r2d2_speech.storage import sqlite_store
except ImportError:
    from ..storage import sqlite_store

logger = logging.getLogger(__name__)


class TranscriptHandler:
    """
    Handles transcript extraction and persistence.
    
    Usage:
        handler = TranscriptHandler(db_path, session_id)
        await handler.handle_user_transcript(event)
        await handler.handle_assistant_transcript(event)
    """
    
    def __init__(self, db_path: str, session_id: str):
        """
        Initialize transcript handler.
        
        Args:
            db_path: Path to SQLite database
            session_id: Current Realtime session ID
        """
        self.db_path = db_path
        self.session_id = session_id
        
        # Buffer for accumulating deltas
        self.assistant_transcript_buffer = ""
        self.current_response_id: Optional[str] = None
        
        logger.info(f"TranscriptHandler initialized (session={session_id})")
    
    async def handle_user_transcript(self, event: Dict[str, Any]) -> None:
        """
        Handle user transcript event and persist immediately.
        
        Event: conversation.item.input_audio_transcription.completed
        
        Args:
            event: Event dictionary from Realtime API
        """
        try:
            # Extract transcript from top-level field (not nested)
            transcript = event.get("transcript", "")
            
            if not transcript:
                logger.warning("User transcript event missing 'transcript' field")
                return
            
            # Extract item_id
            item = event.get("item", {})
            item_id = item.get("id") if isinstance(item, dict) else None
            
            # Log
            logger.info(f"User said: {transcript}")
            
            # Immediately insert to database
            message_id = sqlite_store.insert_message(
                db_path=self.db_path,
                session_id=self.session_id,
                role="user",
                item_id=item_id,
                response_id=None,
                text=transcript,
                raw_event=event,
            )
            
            logger.info(f"✓ User message saved to DB (id={message_id})")
            
        except Exception as e:
            logger.error(f"Error handling user transcript: {e}", exc_info=True)
    
    async def handle_assistant_transcript(self, event: Dict[str, Any]) -> None:
        """
        Handle assistant transcript event and persist immediately.
        
        Event: response.output_audio_transcript.done
        
        Args:
            event: Event dictionary from Realtime API
        """
        try:
            # Extract transcript from event or use accumulated buffer
            transcript = event.get("transcript", self.assistant_transcript_buffer)
            
            if not transcript:
                logger.warning("Assistant transcript done but no text available")
                return
            
            # Extract response_id
            response_id = event.get("response_id") or self.current_response_id
            
            # Log
            logger.info(f"Assistant said: {transcript}")
            
            # Immediately insert to database
            message_id = sqlite_store.insert_message(
                db_path=self.db_path,
                session_id=self.session_id,
                role="assistant",
                item_id=None,
                response_id=response_id,
                text=transcript,
                raw_event=event,
            )
            
            logger.info(f"✓ Assistant message saved to DB (id={message_id})")
            
            # Clear buffer
            self.assistant_transcript_buffer = ""
            self.current_response_id = None
            
        except Exception as e:
            logger.error(f"Error handling assistant transcript: {e}", exc_info=True)
    
    async def handle_assistant_delta(self, event: Dict[str, Any]) -> None:
        """
        Accumulate assistant transcript deltas.
        
        Event: response.output_audio_transcript.delta
        
        Args:
            event: Event dictionary from Realtime API
        """
        try:
            delta = event.get("delta", "")
            
            if delta:
                self.assistant_transcript_buffer += delta
                logger.debug(f"Accumulated delta: +{len(delta)} chars")
            
            # Track response_id if available
            response_id = event.get("response_id")
            if response_id:
                self.current_response_id = response_id
                
        except Exception as e:
            logger.error(f"Error handling assistant delta: {e}", exc_info=True)
    
    def get_buffer_content(self) -> str:
        """
        Get current accumulated transcript buffer (for debugging).
        
        Returns:
            Current buffer content
        """
        return self.assistant_transcript_buffer
    
    def clear_buffer(self) -> None:
        """
        Clear the transcript buffer manually (if needed).
        """
        self.assistant_transcript_buffer = ""
        self.current_response_id = None
        logger.debug("Transcript buffer cleared")


if __name__ == "__main__":
    # Test transcript handler
    import asyncio
    import tempfile
    import os
    
    async def test_handler():
        # Create temporary database
        test_db = os.path.join(tempfile.gettempdir(), "test_transcripts.db")
        
        print(f"Testing TranscriptHandler with: {test_db}")
        
        # Initialize database
        sqlite_store.init_db(test_db)
        print("✓ Database initialized")
        
        # Create session
        session_id = "test-session-123"
        sqlite_store.create_session(
            test_db,
            session_id,
            metadata={"test": True}
        )
        print(f"✓ Session created: {session_id}")
        
        # Create handler
        handler = TranscriptHandler(test_db, session_id)
        
        # Test user transcript
        user_event = {
            "type": "conversation.item.input_audio_transcription.completed",
            "transcript": "Hello, how are you?",
            "item": {"id": "item_001"}
        }
        await handler.handle_user_transcript(user_event)
        
        # Test assistant transcript deltas
        delta_event_1 = {
            "type": "response.output_audio_transcript.delta",
            "delta": "I'm doing ",
            "response_id": "resp_001"
        }
        await handler.handle_assistant_delta(delta_event_1)
        
        delta_event_2 = {
            "type": "response.output_audio_transcript.delta",
            "delta": "well, thank you!",
            "response_id": "resp_001"
        }
        await handler.handle_assistant_delta(delta_event_2)
        
        # Test assistant transcript done
        done_event = {
            "type": "response.output_audio_transcript.done",
            "transcript": "I'm doing well, thank you!",
            "response_id": "resp_001"
        }
        await handler.handle_assistant_transcript(done_event)
        
        # Query messages
        messages = sqlite_store.get_session_messages(test_db, session_id)
        print(f"\n✓ Retrieved {len(messages)} messages:")
        for msg in messages:
            print(f"  - {msg['role']}: {msg['text']}")
        
        # Verify
        assert len(messages) == 2, f"Expected 2 messages, got {len(messages)}"
        assert messages[0]["role"] == "user"
        assert messages[0]["text"] == "Hello, how are you?"
        assert messages[1]["role"] == "assistant"
        assert messages[1]["text"] == "I'm doing well, thank you!"
        
        print("\n✓ TranscriptHandler test PASSED")
        
        # Cleanup
        os.remove(test_db)
        print("✓ Test database cleaned up")
    
    asyncio.run(test_handler())

