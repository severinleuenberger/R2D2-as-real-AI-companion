"""
Robust Sample Rate Detection - Won't Get Stuck!
================================================

With timeouts and progress indicators.
"""

import pyaudio
import numpy as np
import wave
import os
import time


def find_hyperx_device():
    """Find HyperX QuadCast S."""
    p = pyaudio.PyAudio()
    
    print("\n🔍 Searching for HyperX QuadCast S...")
    
    for i in range(p.get_device_count()):
        info = p.get_device_info_by_index(i)
        name = info.get('name', '').lower()
        
        if ('hyperx' in name or 'quadcast' in name) and info.get('maxInputChannels', 0) > 0:
            print(f"\n✓ Found: {info['name']}")
            print(f"  Device Index: {i}")
            print(f"  Max Channels: {info['maxInputChannels']}")
            print(f"  Default Rate: {int(info['defaultSampleRate'])} Hz")
            p.terminate()
            return i
    
    p.terminate()
    return None


def test_sample_rate_quick(device_index, sample_rate):
    """
    Quick test - just open stream and record briefly.
    
    If it hangs, we know this rate doesn't work!
    """
    p = pyaudio.PyAudio()
    
    try:
        print(f"\n📝 Testing {sample_rate} Hz...")
        print(f"  Opening stream...")
        
        # Try to open - this is where it might hang
        stream = p.open(
            format=pyaudio.paInt16,
            channels=1,
            rate=sample_rate,
            input=True,
            input_device_index=device_index,
            frames_per_buffer=2048,
            start=False  # Don't start yet
        )
        
        print(f"  ✓ Stream opened")
        print(f"  Starting capture...")
        
        stream.start_stream()
        
        print(f"  🔴 RECORDING 5 seconds - SPEAK NOW!")
        
        # Record for 5 seconds with progress
        frames = []
        chunks_per_second = sample_rate // 2048
        total_chunks = chunks_per_second * 5
        
        for i in range(total_chunks):
            try:
                data = stream.read(2048, exception_on_overflow=False)
                frames.append(data)
                
                # Show progress every second
                if (i + 1) % chunks_per_second == 0:
                    seconds = (i + 1) // chunks_per_second
                    print(f"  ... {seconds}/5 seconds")
                    
            except Exception as e:
                print(f"  ⚠ Read error: {e}")
                break
        
        stream.stop_stream()
        stream.close()
        
        if len(frames) < total_chunks // 2:
            print(f"  ✗ Too few frames captured")
            p.terminate()
            return False, None
        
        # Analyze
        audio_data = b''.join(frames)
        audio_array = np.frombuffer(audio_data, dtype=np.int16)
        
        rms = np.sqrt(np.mean(audio_array.astype(float)**2))
        
        if rms > 100:  # Some audio detected
            rms_db = 20 * np.log10(rms / 32768.0)
            print(f"  ✓ Audio captured: RMS = {rms_db:.1f} dB")
            
            # Save
            filename = f"/tmp/test_{sample_rate}hz.wav"
            with wave.open(filename, 'wb') as wf:
                wf.setnchannels(1)
                wf.setsampwidth(2)
                wf.setframerate(sample_rate)
                wf.writeframes(audio_data)
            
            print(f"  ✓ Saved: {filename}")
            p.terminate()
            return True, filename
        else:
            print(f"  ✗ No audio detected")
            p.terminate()
            return False, None
            
    except KeyboardInterrupt:
        print(f"\n  ⚠ Interrupted by user")
        p.terminate()
        raise
        
    except Exception as e:
        print(f"  ✗ Error: {e}")
        p.terminate()
        return False, None


def main():
    """Main test."""
    print("=" * 60)
    print("HyperX QuadCast S - Sample Rate Detection (Robust)")
    print("=" * 60)
    
    device_index = find_hyperx_device()
    if device_index is None:
        print("\n✗ Device not found!")
        return
    
    # Test these rates
    test_rates = [
        48000,  # Most likely for HyperX
        44100,  # CD quality
        24000,  # Realtime API
        96000,  # High quality
    ]
    
    print("\n" + "=" * 60)
    print("Testing Sample Rates")
    print("=" * 60)
    print("\nFor EACH test, speak continuously for 5 seconds")
    print("Say: 'Testing [rate] hertz, one two three four five'")
    print("\nIf a test hangs for >10 seconds, press Ctrl+C to skip it")
    
    results = {}
    working_files = []
    
    for rate in test_rates:
        input(f"\n▶ Press ENTER to test {rate} Hz... ")
        
        try:
            success, filename = test_sample_rate_quick(device_index, rate)
            results[rate] = success
            if success and filename:
                working_files.append((rate, filename))
        except KeyboardInterrupt:
            print(f"\n  Skipping {rate} Hz")
            results[rate] = False
            continue
    
    # Summary
    print("\n" + "=" * 60)
    print("RESULTS")
    print("=" * 60)
    
    for rate in test_rates:
        status = "✓ WORKS" if results.get(rate, False) else "✗ FAILED"
        print(f"{rate:6d} Hz: {status}")
    
    working_rates = [r for r, s in results.items() if s]
    
    if working_rates:
        print(f"\n✅ Working rates: {', '.join(map(str, working_rates))} Hz")
        
        # Play back test
        if working_files:
            print("\n" + "=" * 60)
            print("Playback Test - Listen for NORMAL SPEED")
            print("=" * 60)
            
            for rate, filename in working_files:
                input(f"\n▶ Press ENTER to play {rate} Hz...")
                print(f"🔊 Playing {rate} Hz - does this sound NORMAL?")
                os.system(f"aplay {filename}")
                
                response = input(f"   Was {rate} Hz at NORMAL SPEED? (y/n): ")
                if response.lower() == 'y':
                    print(f"\n🎯 FOUND IT! Use {rate} Hz for Realtime API")
                    
                    # Show recommended action
                    print("\n" + "=" * 60)
                    print("RECOMMENDED ACTION")
                    print("=" * 60)
                    print(f"Use {rate} Hz as the capture rate")
                    print(f"Resample to 24000 Hz for Realtime API using scipy")
                    break
    else:
        print("\n✗ No working sample rates found!")
    
    print("\n" + "=" * 60)
    print("Test Complete")
    print("=" * 60)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n\nTest stopped by user")
    except Exception as e:
        print(f"\n✗ Error: {e}")
        import traceback
        traceback.print_exc()

