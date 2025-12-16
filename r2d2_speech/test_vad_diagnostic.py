"""
Diagnostic test for VAD (Voice Activity Detection).
Shows real-time audio levels and VAD events.
"""

import asyncio
import logging
import sys
import os
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from realtime.realtime_client import RealtimeClient
from config.config_manager import get_config
from utils.audio_stream import AudioCapture, AudioResampler

# Setup logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)


async def main():
    """Run VAD diagnostic test."""
    
    print("=" * 70)
    print("VAD DIAGNOSTIC TEST")
    print("=" * 70)
    print()
    print("This test will:")
    print("  1. Show real-time audio levels from your microphone")
    print("  2. Send audio to the Realtime API")
    print("  3. Show when the server VAD detects speech")
    print()
    print("INSTRUCTIONS:")
    print("  - Speak clearly into the microphone")
    print("  - Pause for 1 second after speaking")
    print("  - Watch for VAD events (speech started/stopped)")
    print()
    print("=" * 70)
    print()
    
    # Load config
    config = get_config()
    
    # Create audio components
    print("Initializing audio...")
    
    # Get device from config (with auto-detect fallback)
    from utils.audio_stream import get_device_from_config
    device_index, device_info = get_device_from_config(config)
    
    capture = AudioCapture(
        device_index=device_index,
        native_sample_rate=config["mic_native_sample_rate"]
    )
    
    resampler = AudioResampler(
        source_rate=config["mic_native_sample_rate"],
        target_rate=config["mic_sample_rate"]
    )
    
    print(f"✓ Microphone: {device_info['name']}")
    print(f"✓ Sample rate: {config['mic_native_sample_rate']} Hz → {config['mic_sample_rate']} Hz")
    print()
    
    # Create Realtime client
    print("Connecting to Realtime API...")
    client = RealtimeClient(
        api_key=config["openai_api_key"],
        model=config["realtime_model"],
        voice=config["realtime_voice"]
    )
    
    await client.connect()
    await client.create_session()
    print("✓ Connected to Realtime API")
    print()
    
    # Track VAD events
    vad_events = []
    
    async def listen_for_events():
        """Listen for and display VAD events."""
        try:
            while client.connected:
                event = await asyncio.wait_for(
                    client.receive_event(),
                    timeout=0.1
                )
                
                if event is None:
                    break
                
                event_type = event.get("type", "unknown")
                
                if event_type == "input_audio_buffer.speech_started":
                    vad_events.append(("started", asyncio.get_event_loop().time()))
                    print("\n🎤 SPEECH DETECTED!")
                    
                elif event_type == "input_audio_buffer.speech_stopped":
                    vad_events.append(("stopped", asyncio.get_event_loop().time()))
                    print("\n🔇 SPEECH ENDED - processing...")
                    
                elif event_type == "input_audio_buffer.committed":
                    print("✓ Audio committed")
                    
                elif event_type == "conversation.item.input_audio_transcription.completed":
                    transcript = event.get("transcript", "")
                    print(f"\n📝 TRANSCRIPT: \"{transcript}\"")
                    
                elif event_type == "response.created":
                    print("\n🤖 Assistant responding...")
                    
        except asyncio.TimeoutError:
            pass
        except Exception as e:
            logger.error(f"Event listener error: {e}")
    
    # Start event listener
    event_task = asyncio.create_task(listen_for_events())
    
    # Start audio capture
    print("Starting audio capture...")
    print("=" * 70)
    print("🎤 SPEAK NOW! (Test will run for 20 seconds)")
    print("=" * 70)
    print()
    
    capture.start()
    
    start_time = asyncio.get_event_loop().time()
    chunk_count = 0
    
    try:
        while asyncio.get_event_loop().time() - start_time < 20:
            # Capture audio
            audio_data = capture.read_chunk()
            if audio_data is None:
                await asyncio.sleep(0.01)
                continue
            
            # Calculate audio level for visual feedback (audio_data is np.ndarray of int16)
            avg_level = np.abs(audio_data).mean()
            level_bars = int((avg_level / 32767) * 50)  # Scale to 50 chars
            
            # Show audio level every 10 chunks (~1 second)
            if chunk_count % 10 == 0:
                print(f"Level: {'█' * level_bars}{' ' * (50 - level_bars)} ({avg_level:.0f})", end='\r')
            
            # Resample and send
            resampled = resampler.resample_chunk(audio_data)
            base64_audio = resampler.to_base64(resampled)
            await client.send_audio_frame(base64_audio)
            
            chunk_count += 1
            await asyncio.sleep(0.01)  # Small delay to prevent overwhelming
            
    except KeyboardInterrupt:
        print("\n\nInterrupted by user")
    
    # Stop capture
    capture.stop()
    print("\n")
    print("=" * 70)
    print("✓ Audio capture stopped")
    
    # Wait for final events
    print("Waiting for final events (5 seconds)...")
    await asyncio.sleep(5)
    
    # Stop event listener
    event_task.cancel()
    
    # Disconnect
    await client.disconnect()
    
    # Summary
    print()
    print("=" * 70)
    print("SUMMARY")
    print("=" * 70)
    print(f"Chunks sent: {chunk_count}")
    print(f"VAD events: {len(vad_events)}")
    for event_type, timestamp in vad_events:
        print(f"  - {event_type} at {timestamp - start_time:.1f}s")
    
    if len(vad_events) == 0:
        print()
        print("⚠ No speech detected!")
        print()
        print("Troubleshooting:")
        print("  1. Check microphone is not muted")
        print("  2. Speak louder and closer to the microphone")
        print("  3. Reduce background noise")
        print("  4. Check audio level bars showed activity when speaking")
    elif len(vad_events) == 1:
        print()
        print("⚠ Speech started but not stopped!")
        print()
        print("Troubleshooting:")
        print("  1. Pause for at least 1 second after speaking")
        print("  2. Reduce background noise")
        print("  3. The test might have ended before the pause")
    else:
        print()
        print("✓ VAD working correctly!")
    
    print("=" * 70)


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except Exception as e:
        logger.error(f"Test failed: {e}", exc_info=True)

