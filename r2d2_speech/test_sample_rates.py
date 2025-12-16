"""
Detect Correct Sample Rate for HyperX QuadCast S
=================================================

This script tests different sample rates to find what actually works.

HyperX QuadCast S specs say 48kHz, but PyAudio reports 44.1kHz.
Let's find the truth!
"""

import pyaudio
import numpy as np
import wave
import os


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
            print(f"  Max Input Channels: {info['maxInputChannels']}")
            print(f"  Default Sample Rate (reported): {int(info['defaultSampleRate'])} Hz")
            p.terminate()
            return i
    
    p.terminate()
    return None


def test_sample_rate(device_index, sample_rate, duration=15.0):
    """
    Test recording at a specific sample rate.
    
    Returns True if successful, False if not supported.
    """
    p = pyaudio.PyAudio()
    
    try:
        print(f"\n📝 Testing {sample_rate} Hz...")
        
        # Try to open stream at this rate
        stream = p.open(
            format=pyaudio.paInt16,
            channels=1,  # Mono
            rate=sample_rate,
            input=True,
            input_device_index=device_index,
            frames_per_buffer=2048
        )
        
        print(f"  ✓ Stream opened successfully")
        
        # Record brief sample
        print(f"  Recording {duration}s...")
        frames = []
        num_chunks = int(sample_rate * duration / 2048)
        
        for i in range(num_chunks):
            data = stream.read(2048, exception_on_overflow=False)
            frames.append(data)
        
        stream.stop_stream()
        stream.close()
        
        # Convert to array and analyze
        audio_data = b''.join(frames)
        audio_array = np.frombuffer(audio_data, dtype=np.int16)
        
        rms = np.sqrt(np.mean(audio_array.astype(float)**2))
        
        if rms > 0:
            rms_db = 20 * np.log10(rms / 32768.0)
            print(f"  ✓ Audio captured: RMS = {rms_db:.1f} dB")
            
            # Save test file
            filename = f"/tmp/test_{sample_rate}hz.wav"
            with wave.open(filename, 'wb') as wf:
                wf.setnchannels(1)
                wf.setsampwidth(2)
                wf.setframerate(sample_rate)
                wf.writeframes(audio_data)
            
            print(f"  💾 Saved: {filename}")
            
            p.terminate()
            return True, filename
        else:
            print(f"  ✗ No audio detected (silence)")
            p.terminate()
            return False, None
            
    except Exception as e:
        print(f"  ✗ Failed: {e}")
        p.terminate()
        return False, None


def main():
    """Test different sample rates."""
    print("=" * 60)
    print("HyperX QuadCast S - Sample Rate Detection")
    print("=" * 60)
    
    device_index = find_hyperx_device()
    if device_index is None:
        print("\n✗ HyperX QuadCast S not found!")
        return
    
    # Common sample rates for USB Audio
    test_rates = [
        44100,  # CD quality (PyAudio reports this)
        48000,  # Professional audio (HyperX spec says this)
        96000,  # High quality
        24000,  # Realtime API target
    ]
    
    print("\n" + "=" * 60)
    print("Testing Sample Rates")
    print("=" * 60)
    print("\nDuring each test, speak continuously for 15 seconds")
    print("Say something like: 'This is a microphone test at [rate] Hz.'")
    print("'I am testing the HyperX QuadCast S to find the correct sample rate.'")
    input("\nPress ENTER to start tests...")
    
    results = {}
    working_files = []
    
    for rate in test_rates:
        success, filename = test_sample_rate(device_index, rate, duration=15.0)
        results[rate] = success
        if success and filename:
            working_files.append((rate, filename))
    
    # Summary
    print("\n" + "=" * 60)
    print("RESULTS")
    print("=" * 60)
    
    for rate, success in results.items():
        status = "✓ WORKS" if success else "✗ FAILED"
        print(f"{rate:6d} Hz: {status}")
    
    # Find the working rate
    working_rates = [r for r, s in results.items() if s]
    
    if working_rates:
        print(f"\n✅ Supported sample rates: {', '.join(map(str, working_rates))} Hz")
        
        # Recommend the best rate
        if 48000 in working_rates:
            best_rate = 48000
            print(f"\n🎯 RECOMMENDED: Use 48000 Hz (HyperX native rate)")
        elif 44100 in working_rates:
            best_rate = 44100
            print(f"\n🎯 RECOMMENDED: Use 44100 Hz")
        else:
            best_rate = working_rates[0]
            print(f"\n🎯 RECOMMENDED: Use {best_rate} Hz")
        
        # Offer to play back test recordings
        if working_files:
            print("\n" + "=" * 60)
            print("Playback Test")
            print("=" * 60)
            print("\nWould you like to play back the recordings?")
            print("This will verify the sample rate is correct.")
            print("If it sounds at NORMAL SPEED, that's the right rate!")
            
            for rate, filename in working_files:
                input(f"\nPress ENTER to play {rate} Hz recording...")
                print(f"▶️  Playing {rate} Hz...")
                os.system(f"aplay {filename}")
                print(f"   Did that sound at NORMAL SPEED?")
        
    else:
        print("\n✗ No sample rates worked!")
    
    print("\n" + "=" * 60)
    print("Test Complete")
    print("=" * 60)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n\nTest interrupted")
    except Exception as e:
        print(f"\n✗ Error: {e}")
        import traceback
        traceback.print_exc()

