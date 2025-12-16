"""
Test Realtime Connection with HyperX QuadCast S Microphone
===========================================================

Enhanced test using AudioStreamManager for continuous streaming.

Usage:
    cd ~/dev/r2d2
    source r2d2_speech_env/bin/activate
    python -m r2d2_speech.test_realtime_with_mic
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
from utils import AudioStreamManager

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)


async def test_realtime_with_mic():
    """
    Test Realtime API with real microphone audio from HyperX QuadCast S.
    
    Uses AudioStreamManager for continuous streaming.
    """
    print("=" * 60)
    print("Test: Realtime with HyperX QuadCast S Microphone")
    print("=" * 60)
    print()
    
    client = None
    audio_manager = None
    
    try:
        # Step 1: Load configuration
        print("Step 1: Loading configuration...")
        config = get_config()
        print(f"✓ Config loaded")
        print(f"  - Model: {config['realtime_model']}")
        print(f"  - Voice: {config['realtime_voice']}")
        print(f"  - Native rate: {config['mic_native_sample_rate']} Hz")
        print(f"  - Target rate: {config['mic_sample_rate']} Hz")
        print()
        
        # Step 2: Initialize database
        print("Step 2: Initializing database...")
        db_path = config['db_path']
        init_db(db_path)
        print(f"✓ Database initialized")
        print()
        
        # Step 3: Create session record
        print("Step 3: Creating session record...")
        session_id = f"test-stream-{datetime.now().strftime('%Y%m%d-%H%M%S')}"
        create_session(
            db_path,
            session_id,
            metadata={
                "model": config['realtime_model'],
                "voice": config['realtime_voice'],
                "test": "streaming",
                "native_rate": config['mic_native_sample_rate'],
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
            instructions="You are a helpful assistant. Respond naturally to what you hear.",
            temperature=0.8
        )
        print(f"✓ Session created")
        print()
        
        # Step 6: Initialize AudioStreamManager
        print("Step 6: Initializing AudioStreamManager...")
        audio_manager = AudioStreamManager(config, client)
        print(f"✓ Audio manager initialized")
        print(f"  - Device: {audio_manager.device_info['name']}")
        print(f"  - Native rate: {audio_manager.native_rate} Hz")
        print(f"  - Target rate: {audio_manager.target_rate} Hz")
        print()
        
        # Step 7: Start audio playback
        print("Step 7: Starting audio playback...")
        audio_manager.start_playback()
        print(f"✓ Playback started")
        print()
        
        # Step 8: Initialize handlers
        print("Step 8: Initializing event handlers...")
        transcript_handler = TranscriptHandler(db_path, session_id)
        event_router = EventRouter(
            client,
            transcript_handler,
            audio_playback=audio_manager.playback
        )
        print(f"✓ Handlers initialized")
        print()
        
        # Step 9: Start event listener
        print("Step 9: Starting event listener...")
        event_task = asyncio.create_task(event_router.start_listening())
        await asyncio.sleep(0.5)  # Let listener start
        print(f"✓ Event listener started")
        print()
        
        # Step 10: Start audio streaming
        print("Step 10: Starting audio streaming...")
        await audio_manager.start()
        print(f"✓ Audio streaming started")
        print()
        
        # Step 11: Stream audio continuously
        print("=" * 60)
        print("Ready! Speak into the microphone")
        print("=" * 60)
        print()
        print("The system will:")
        print("  1. Capture audio continuously")
        print("  2. Send to OpenAI Realtime API")
        print("  3. Transcribe your speech")
        print("  4. Generate and play assistant response")
        print("  5. Save both to database")
        print()
        print("Speak now! (Will run for 30 seconds, or press Ctrl+C to stop)")
        print()
        
        # Stream for 30 seconds
        start_time = asyncio.get_event_loop().time()
        chunk_count = 0
        
        while asyncio.get_event_loop().time() - start_time < 30:
            # Process and send one chunk
            success = await audio_manager.process_and_send()
            
            if success:
                chunk_count += 1
                
                # Show progress every 50 chunks (~5 seconds)
                if chunk_count % 50 == 0:
                    elapsed = asyncio.get_event_loop().time() - start_time
                    print(f"  ... streaming ({int(elapsed)}s, {chunk_count} chunks sent)")
            
            # Small delay to maintain real-time pacing
            await asyncio.sleep(0.01)
        
        print()
        print(f"✓ Streaming complete ({chunk_count} chunks sent)")
        print()
        
        # Step 12: Stop streaming
        print("Step 11: Stopping audio streaming...")
        await audio_manager.stop()
        audio_manager.stop_playback()
        print(f"✓ Audio stopped")
        print()
        
        # Step 13: Wait a bit for final events
        print("Step 12: Waiting for final events...")
        await asyncio.sleep(3)
        print()
        
        # Step 14: Display results
        print("Step 13: Results")
        print("=" * 60)
        
        messages = get_session_messages(db_path, session_id)
        
        if len(messages) == 0:
            print("⚠ No transcripts received")
            print("  Possible reasons:")
            print("  - No speech detected (try speaking louder)")
            print("  - Sample rate mismatch (run test_sample_rates_v2.py)")
            print("  - API issue")
            return False
        
        print(f"✓ Retrieved {len(messages)} message(s)")
        print()
        
        for msg in messages:
            role_label = "USER" if msg['role'] == 'user' else "ASSISTANT"
            print(f"[{role_label}]")
            print(f"  Text: {msg['text']}")
            print(f"  Time: {msg['created_at']}")
            print()
        
        # Verify
        has_user = any(m['role'] == 'user' for m in messages)
        has_assistant = any(m['role'] == 'assistant' for m in messages)
        
        print("=" * 60)
        if has_user and has_assistant:
            print("✅ TEST PASSED")
            print("=" * 60)
            print("✓ Audio streaming works")
            print("✓ User transcript received")
            print("✓ Assistant response generated")
            print("✓ Assistant audio played")
            print("✓ Both saved to database")
            return True
        elif has_user:
            print("⚠ PARTIAL SUCCESS")
            print("=" * 60)
            print("✓ User transcript received")
            print("✗ No assistant response (check API/events)")
            return False
        else:
            print("✗ NO TRANSCRIPTS")
            print("=" * 60)
            print("✗ Check sample rate and audio levels")
            return False
        
    except KeyboardInterrupt:
        print("\n\n⚠ Test interrupted by user")
        return False
        
    except Exception as e:
        print()
        print("=" * 60)
        print("✗ TEST FAILED")
        print("=" * 60)
        print(f"Error: {e}")
        logger.exception("Test failed")
        return False
        
    finally:
        # Cleanup
        print("\nCleaning up...")
        
        if audio_manager:
            if audio_manager.is_running:
                await audio_manager.stop()
            if audio_manager.playback:
                audio_manager.stop_playback()
        
        if client and client.connected:
            await client.disconnect()
        
        print("✓ Cleanup complete")


def main():
    """Entry point."""
    try:
        success = asyncio.run(test_realtime_with_mic())
        sys.exit(0 if success else 1)
    except KeyboardInterrupt:
        print("\n\nTest interrupted")
        sys.exit(130)
    except Exception as e:
        print(f"\nFatal error: {e}")
        logger.exception("Fatal error")
        sys.exit(1)


if __name__ == "__main__":
    main()

