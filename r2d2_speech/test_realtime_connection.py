"""
Test Realtime Connection
=========================

Manual test script to verify OpenAI Realtime API connection and transcript persistence.

This test:
1. Loads configuration from ~/.r2d2/.env
2. Initializes SQLite database
3. Connects to Realtime API
4. Creates session with transcription enabled
5. Sends dummy PCM16 silence audio
6. Waits for events and verifies transcripts are saved
7. Queries database to verify persistence

Usage:
    cd ~/dev/r2d2
    source r2d2_speech_env/bin/activate
    python -m r2d2_speech.test_realtime_connection
"""

import asyncio
import logging
import sys
from datetime import datetime
from pathlib import Path

# Add package to path
sys.path.insert(0, str(Path(__file__).parent))

from config import get_config
from storage import init_db, create_session, get_session_messages
from realtime import RealtimeClient, EventRouter, TranscriptHandler

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)


async def test_realtime_connection():
    """
    Test Realtime API connection and transcript persistence.
    """
    print("=" * 60)
    print("Test: Realtime Connection & Text Persistence")
    print("=" * 60)
    print()
    
    client = None
    
    try:
        # Step 1: Load configuration
        print("Step 1: Loading configuration...")
        config = get_config()
        print(f"✓ Config loaded")
        print(f"  - Model: {config['realtime_model']}")
        print(f"  - Voice: {config['realtime_voice']}")
        print(f"  - API Key: {config['openai_api_key'][:10]}...")
        print()
        
        # Step 2: Initialize database
        print("Step 2: Initializing database...")
        db_path = config['db_path']
        init_db(db_path)
        print(f"✓ Database initialized: {db_path}")
        print()
        
        # Step 3: Create session record
        print("Step 3: Creating session record...")
        session_id = f"test-session-{datetime.now().strftime('%Y%m%d-%H%M%S')}"
        create_session(
            db_path,
            session_id,
            metadata={
                "model": config['realtime_model'],
                "voice": config['realtime_voice'],
                "test": True,
            }
        )
        print(f"✓ Session created: {session_id}")
        print()
        
        # Step 4: Connect to Realtime API
        print("Step 4: Connecting to Realtime API...")
        client = RealtimeClient(
            api_key=config['openai_api_key'],
            model=config['realtime_model'],
            voice=config['realtime_voice']
        )
        await client.connect()
        print(f"✓ WebSocket connected")
        print()
        
        # Step 5: Create session
        print("Step 5: Creating Realtime session...")
        await client.create_session(
            instructions="You are a helpful test assistant. Respond briefly to test inputs.",
            temperature=0.8
        )
        print(f"✓ Session created with:")
        print(f"  - input_audio_transcription: enabled")
        print(f"  - output_audio_transcript: enabled")
        print(f"  - server VAD: enabled")
        print()
        
        # Step 6: Initialize handlers
        print("Step 6: Initializing event handlers...")
        transcript_handler = TranscriptHandler(db_path, session_id)
        event_router = EventRouter(client, transcript_handler)
        print(f"✓ Handlers initialized")
        print()
        
        # Step 7: Start event listener in background
        print("Step 7: Starting event listener...")
        event_task = asyncio.create_task(event_router.start_listening())
        print(f"✓ Event listener started")
        print()
        
        # Step 8: Send dummy audio
        print("Step 8: Sending dummy PCM16 silence audio...")
        print("  (This simulates microphone input without actual recording)")
        await client.send_test_audio(duration_seconds=1.0)
        print(f"✓ Test audio sent")
        print()
        
        # Step 9: Wait for events
        print("Step 9: Waiting for events (30 seconds max)...")
        print("  - Waiting for user transcript...")
        print("  - Waiting for assistant response...")
        
        # Wait up to 30 seconds for events to be processed
        for i in range(30):
            await asyncio.sleep(1)
            
            # Check database for messages
            messages = get_session_messages(db_path, session_id)
            
            if len(messages) >= 2:
                print(f"\n✓ Received both transcripts!")
                break
            
            if i % 5 == 0 and i > 0:
                print(f"  ... waiting ({i}s elapsed, {len(messages)} messages so far)")
        else:
            print(f"\n⚠ Timeout after 30 seconds")
        
        print()
        
        # Step 10: Query and verify database
        print("Step 10: Querying database...")
        messages = get_session_messages(db_path, session_id)
        print(f"✓ Retrieved {len(messages)} messages")
        print()
        
        if len(messages) == 0:
            print("✗ No messages found in database")
            print("  This might mean:")
            print("  - Audio was too short/silent to transcribe")
            print("  - API didn't detect speech")
            print("  - Event handling issue")
            print()
            print("Recommendation: Try with real microphone audio in future tests")
            return False
        
        # Display messages
        print("Messages:")
        print("-" * 60)
        for msg in messages:
            role_label = "USER" if msg['role'] == 'user' else "ASSISTANT"
            print(f"[{role_label}] {msg['text']}")
            print(f"  - Created: {msg['created_at']}")
            if msg['item_id']:
                print(f"  - Item ID: {msg['item_id']}")
            if msg['response_id']:
                print(f"  - Response ID: {msg['response_id']}")
            print()
        
        # Verify acceptance criteria
        print("=" * 60)
        print("Verification Results:")
        print("=" * 60)
        
        checks = []
        
        # Check 1: At least one user message
        has_user = any(m['role'] == 'user' for m in messages)
        checks.append(("User message in DB", has_user))
        
        # Check 2: At least one assistant message
        has_assistant = any(m['role'] == 'assistant' for m in messages)
        checks.append(("Assistant message in DB", has_assistant))
        
        # Check 3: Messages have non-empty text
        all_have_text = all(m['text'].strip() for m in messages)
        checks.append(("All messages have text", all_have_text))
        
        # Check 4: Messages written immediately (all have created_at)
        all_have_timestamp = all(m['created_at'] for m in messages)
        checks.append(("All messages timestamped", all_have_timestamp))
        
        # Display results
        for check_name, passed in checks:
            status = "✓" if passed else "✗"
            print(f"{status} {check_name}")
        
        print()
        
        all_passed = all(passed for _, passed in checks)
        
        if all_passed:
            print("=" * 60)
            print("✓ TEST PASSED")
            print("=" * 60)
            print()
            print("Acceptance Criteria Met:")
            print("✓ Realtime WebSocket connected")
            print("✓ Session created with transcription enabled")
            print("✓ User transcript extracted and persisted")
            print("✓ Assistant transcript extracted and persisted")
            print("✓ Messages written immediately to database")
            print()
            return True
        else:
            print("=" * 60)
            print("⚠ TEST PARTIALLY PASSED")
            print("=" * 60)
            print()
            print("Some messages were received, but not all criteria met.")
            print("This may be due to using silent audio for testing.")
            print()
            return False
        
    except Exception as e:
        print()
        print("=" * 60)
        print("✗ TEST FAILED")
        print("=" * 60)
        print(f"Error: {e}")
        logger.exception("Test failed with exception")
        return False
        
    finally:
        # Cleanup
        if client and client.connected:
            print("Cleaning up...")
            await client.disconnect()
            print("✓ Disconnected")


def main():
    """
    Entry point for test script.
    """
    try:
        success = asyncio.run(test_realtime_connection())
        sys.exit(0 if success else 1)
    except KeyboardInterrupt:
        print("\n\nTest interrupted by user")
        sys.exit(130)
    except Exception as e:
        print(f"\n✗ Fatal error: {e}")
        logger.exception("Fatal error")
        sys.exit(1)


if __name__ == "__main__":
    main()

