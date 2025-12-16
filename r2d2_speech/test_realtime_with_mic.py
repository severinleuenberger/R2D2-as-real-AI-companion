"""
Test Realtime Connection with HyperX QuadCast S Microphone
===========================================================

Enhanced test that captures real audio from HyperX QuadCast S.

Usage:
    cd ~/dev/r2d2
    source r2d2_speech_env/bin/activate
    python -m r2d2_speech.test_realtime_with_mic
"""

import asyncio
import logging
import sys
import base64
import struct
from datetime import datetime
from pathlib import Path

# Audio capture
import pyaudio
import numpy as np

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


def find_hyperx_device():
    """
    Find HyperX QuadCast S microphone device.
    
    Returns:
        Tuple of (device_index, device_info) or (None, None) if not found
    """
    p = pyaudio.PyAudio()
    
    print("\n🔍 Searching for HyperX QuadCast S...")
    
    for i in range(p.get_device_count()):
        info = p.get_device_info_by_index(i)
        name = info.get('name', '').lower()
        
        # Look for HyperX or QuadCast in device name
        if ('hyperx' in name or 'quadcast' in name) and info.get('maxInputChannels', 0) > 0:
            print(f"✓ Found: {info['name']}")
            print(f"  - Device Index: {i}")
            print(f"  - Channels: {info['maxInputChannels']}")
            print(f"  - Sample Rate: {int(info['defaultSampleRate'])} Hz")
            p.terminate()
            return i, info
    
    p.terminate()
    return None, None


def capture_audio_from_mic(device_index, duration_seconds=3.0, sample_rate=24000):
    """
    Capture audio from microphone and convert to PCM16 @ 24kHz mono.
    
    Args:
        device_index: PyAudio device index
        duration_seconds: How long to record
        sample_rate: Target sample rate (24000 for Realtime API)
        
    Returns:
        Base64-encoded PCM16 audio data
    """
    p = pyaudio.PyAudio()
    
    # Get device info
    device_info = p.get_device_info_by_index(device_index)
    device_sample_rate = int(device_info['defaultSampleRate'])
    device_channels = device_info['maxInputChannels']
    
    print(f"\n🎤 Recording {duration_seconds} seconds from microphone...")
    print(f"  - Device sample rate: {device_sample_rate} Hz")
    print(f"  - Device channels: {device_channels}")
    print(f"  - Target: 24000 Hz mono PCM16")
    
    # Open stream
    stream = p.open(
        format=pyaudio.paInt16,
        channels=min(device_channels, 2),  # Use up to stereo
        rate=device_sample_rate,
        input=True,
        input_device_index=device_index,
        frames_per_buffer=1024
    )
    
    # Record
    print(f"🔴 RECORDING... (speak now for {duration_seconds} seconds)")
    frames = []
    num_frames = int(device_sample_rate * duration_seconds / 1024)
    
    for i in range(num_frames):
        data = stream.read(1024, exception_on_overflow=False)
        frames.append(data)
    
    print("✓ Recording complete")
    
    # Stop stream
    stream.stop_stream()
    stream.close()
    p.terminate()
    
    # Convert to numpy array
    audio_data = b''.join(frames)
    audio_array = np.frombuffer(audio_data, dtype=np.int16)
    
    # Convert to mono if stereo
    if device_channels > 1:
        audio_array = audio_array.reshape(-1, device_channels)
        audio_array = audio_array.mean(axis=1).astype(np.int16)
        print("  - Converted to mono")
    
    # Resample if needed
    if device_sample_rate != sample_rate:
        # Simple resampling (linear interpolation)
        num_samples_target = int(len(audio_array) * sample_rate / device_sample_rate)
        indices = np.linspace(0, len(audio_array) - 1, num_samples_target)
        audio_array = np.interp(indices, np.arange(len(audio_array)), audio_array).astype(np.int16)
        print(f"  - Resampled to {sample_rate} Hz")
    
    # Convert to PCM16 bytes
    pcm_bytes = audio_array.tobytes()
    
    # Check audio levels
    rms = np.sqrt(np.mean(audio_array.astype(float)**2))
    peak = np.max(np.abs(audio_array))
    rms_db = 20 * np.log10(rms / 32768.0) if rms > 0 else -100
    peak_db = 20 * np.log10(peak / 32768.0) if peak > 0 else -100
    
    print(f"  - Audio level: RMS={rms_db:.1f} dB, Peak={peak_db:.1f} dB")
    
    if rms < 100:
        print("  ⚠ Warning: Audio level very low - speak louder!")
    
    # Base64 encode
    base64_audio = base64.b64encode(pcm_bytes).decode('utf-8')
    print(f"  - Encoded {len(pcm_bytes)} bytes → {len(base64_audio)} base64 chars")
    
    return base64_audio


async def test_realtime_with_mic():
    """
    Test Realtime API with real microphone audio from HyperX QuadCast S.
    """
    print("=" * 60)
    print("Test: Realtime with HyperX QuadCast S Microphone")
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
        print()
        
        # Step 2: Find HyperX microphone
        print("Step 2: Finding HyperX QuadCast S...")
        device_index, device_info = find_hyperx_device()
        
        if device_index is None:
            print("✗ HyperX QuadCast S not found!")
            print("\nAvailable input devices:")
            p = pyaudio.PyAudio()
            for i in range(p.get_device_count()):
                info = p.get_device_info_by_index(i)
                if info.get('maxInputChannels', 0) > 0:
                    print(f"  [{i}] {info['name']}")
            p.terminate()
            return False
        
        print()
        
        # Step 3: Initialize database
        print("Step 3: Initializing database...")
        db_path = config['db_path']
        init_db(db_path)
        print(f"✓ Database initialized")
        print()
        
        # Step 4: Create session record
        print("Step 4: Creating session record...")
        session_id = f"test-mic-{datetime.now().strftime('%Y%m%d-%H%M%S')}"
        create_session(
            db_path,
            session_id,
            metadata={
                "model": config['realtime_model'],
                "voice": config['realtime_voice'],
                "test": "microphone",
                "device": device_info['name'],
            }
        )
        print(f"✓ Session created: {session_id}")
        print()
        
        # Step 5: Connect to Realtime API
        print("Step 5: Connecting to Realtime API...")
        client = RealtimeClient(
            api_key=config['openai_api_key'],
            model=config['realtime_model'],
            voice=config['realtime_voice']
        )
        await client.connect()
        print(f"✓ WebSocket connected")
        print()
        
        # Step 6: Create session
        print("Step 6: Creating Realtime session...")
        await client.create_session(
            instructions="You are a helpful assistant. Respond naturally to what you hear.",
            temperature=0.8
        )
        print(f"✓ Session created")
        print()
        
        # Step 7: Initialize handlers
        print("Step 7: Initializing event handlers...")
        transcript_handler = TranscriptHandler(db_path, session_id)
        event_router = EventRouter(client, transcript_handler)
        print(f"✓ Handlers initialized")
        print()
        
        # Step 8: Start event listener
        print("Step 8: Starting event listener...")
        event_task = asyncio.create_task(event_router.start_listening())
        await asyncio.sleep(0.5)  # Let listener start
        print(f"✓ Event listener started")
        print()
        
        # Step 9: Wait for user to press Enter
        print("=" * 60)
        print("Ready to record!")
        print("=" * 60)
        input("Press ENTER to start recording (3 seconds)... ")
        print()
        
        # Step 10: Capture audio
        print("Step 9: Capturing audio from HyperX QuadCast S...")
        base64_audio = capture_audio_from_mic(device_index, duration_seconds=3.0)
        print()
        
        # Step 11: Send audio to Realtime API
        print("Step 10: Sending audio to Realtime API...")
        
        # Send in chunks (100ms each)
        audio_bytes = base64.b64decode(base64_audio)
        chunk_size = 24000 * 2 * 0.1  # 100ms chunks (24kHz * 2 bytes * 0.1s)
        chunk_size = int(chunk_size)
        
        for i in range(0, len(audio_bytes), chunk_size):
            chunk = audio_bytes[i:i+chunk_size]
            chunk_base64 = base64.b64encode(chunk).decode('utf-8')
            await client.send_audio_frame(chunk_base64)
            await asyncio.sleep(0.1)  # Real-time pacing
        
        # Commit audio
        await client.commit_audio()
        print("✓ Audio sent and committed")
        print()
        
        # Step 12: Wait for events
        print("Step 11: Waiting for transcripts (30 seconds max)...")
        print("  - Waiting for user transcript...")
        print("  - Waiting for assistant response...")
        print()
        
        for i in range(30):
            await asyncio.sleep(1)
            
            messages = get_session_messages(db_path, session_id)
            
            if len(messages) >= 2:
                print(f"✓ Received both transcripts!")
                break
            
            if i > 0 and i % 5 == 0:
                print(f"  ... waiting ({i}s elapsed, {len(messages)} messages)")
        
        print()
        
        # Step 13: Display results
        print("Step 12: Results")
        print("=" * 60)
        
        messages = get_session_messages(db_path, session_id)
        
        if len(messages) == 0:
            print("⚠ No transcripts received")
            print("  Possible reasons:")
            print("  - Audio too quiet (check mic level)")
            print("  - No speech detected")
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
            print("✓ Microphone capture works")
            print("✓ Audio sent to Realtime API")
            print("✓ User transcript received")
            print("✓ Assistant response received")
            print("✓ Both saved to database")
            return True
        else:
            print("⚠ PARTIAL SUCCESS")
            print("=" * 60)
            print(f"✓ User transcript: {'YES' if has_user else 'NO'}")
            print(f"✓ Assistant transcript: {'YES' if has_assistant else 'NO'}")
            return has_user  # Partial pass if we got user transcript
        
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
        if client and client.connected:
            print("\nCleaning up...")
            await client.disconnect()
            print("✓ Disconnected")


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

