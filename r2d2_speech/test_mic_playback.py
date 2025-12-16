"""
Microphone Test with Playback
==============================

Simple test to verify HyperX QuadCast S captures and plays audio correctly.

Usage:
    cd ~/dev/r2d2
    source r2d2_speech_env/bin/activate
    python test_mic_playback.py
"""

import pyaudio
import numpy as np
import wave
import tempfile
import os


def find_hyperx_device():
    """Find HyperX QuadCast S microphone."""
    p = pyaudio.PyAudio()
    
    print("\n🔍 Searching for HyperX QuadCast S...")
    
    for i in range(p.get_device_count()):
        info = p.get_device_info_by_index(i)
        name = info.get('name', '').lower()
        
        if ('hyperx' in name or 'quadcast' in name) and info.get('maxInputChannels', 0) > 0:
            print(f"✓ Found: {info['name']}")
            print(f"  - Device Index: {i}")
            print(f"  - Channels: {info['maxInputChannels']}")
            print(f"  - Sample Rate: {int(info['defaultSampleRate'])} Hz")
            p.terminate()
            return i, info
    
    p.terminate()
    print("✗ HyperX QuadCast S not found!")
    return None, None


def record_audio(device_index, duration_seconds=3.0):
    """Record audio from microphone."""
    p = pyaudio.PyAudio()
    
    device_info = p.get_device_info_by_index(device_index)
    sample_rate = int(device_info['defaultSampleRate'])
    channels = min(device_info['maxInputChannels'], 2)
    
    print(f"\n🎤 Recording {duration_seconds} seconds...")
    print(f"  - Sample rate: {sample_rate} Hz")
    print(f"  - Channels: {channels}")
    
    stream = p.open(
        format=pyaudio.paInt16,
        channels=channels,
        rate=sample_rate,
        input=True,
        input_device_index=device_index,
        frames_per_buffer=1024
    )
    
    print("🔴 RECORDING NOW - SPEAK!")
    frames = []
    num_frames = int(sample_rate * duration_seconds / 1024)
    
    for i in range(num_frames):
        data = stream.read(1024, exception_on_overflow=False)
        frames.append(data)
        
        # Show progress
        if i % (num_frames // 10) == 0:
            progress = int((i / num_frames) * 100)
            print(f"  Recording... {progress}%")
    
    print("✓ Recording complete!")
    
    stream.stop_stream()
    stream.close()
    
    # Analyze audio
    audio_data = b''.join(frames)
    audio_array = np.frombuffer(audio_data, dtype=np.int16)
    
    rms = np.sqrt(np.mean(audio_array.astype(float)**2))
    peak = np.max(np.abs(audio_array))
    
    if rms > 0:
        rms_db = 20 * np.log10(rms / 32768.0)
        peak_db = 20 * np.log10(peak / 32768.0)
        print(f"\n📊 Audio Analysis:")
        print(f"  - RMS Level: {rms_db:.1f} dB")
        print(f"  - Peak Level: {peak_db:.1f} dB")
        
        if rms_db < -40:
            print("  ⚠ Warning: Audio very quiet - speak louder or move closer!")
        elif rms_db > -10:
            print("  ⚠ Warning: Audio might be too loud - risk of clipping!")
        else:
            print("  ✓ Good audio level!")
    else:
        print("  ✗ No audio detected - silence only!")
    
    p.terminate()
    return audio_data, sample_rate, channels


def play_audio(audio_data, sample_rate, channels):
    """Play audio through default output device."""
    p = pyaudio.PyAudio()
    
    # Find default output device
    default_output = p.get_default_output_device_info()
    print(f"\n🔊 Playing back through: {default_output['name']}")
    
    stream = p.open(
        format=pyaudio.paInt16,
        channels=channels,
        rate=sample_rate,
        output=True
    )
    
    print("▶️  PLAYING... (listen now)")
    stream.write(audio_data)
    
    print("✓ Playback complete!")
    
    stream.stop_stream()
    stream.close()
    p.terminate()


def save_wav(audio_data, sample_rate, channels, filename):
    """Save audio to WAV file."""
    with wave.open(filename, 'wb') as wf:
        wf.setnchannels(channels)
        wf.setsampwidth(2)  # 16-bit
        wf.setframerate(sample_rate)
        wf.writeframes(audio_data)
    print(f"💾 Saved to: {filename}")


def main():
    """Main test function."""
    print("=" * 60)
    print("HyperX QuadCast S - Microphone Test with Playback")
    print("=" * 60)
    
    # Find microphone
    device_index, device_info = find_hyperx_device()
    if device_index is None:
        print("\nShowing all input devices:")
        p = pyaudio.PyAudio()
        for i in range(p.get_device_count()):
            info = p.get_device_info_by_index(i)
            if info.get('maxInputChannels', 0) > 0:
                print(f"  [{i}] {info['name']}")
        p.terminate()
        return
    
    # Wait for user
    print("\n" + "=" * 60)
    print("Ready to record!")
    print("=" * 60)
    input("Press ENTER to start recording (3 seconds)... ")
    
    # Record
    audio_data, sample_rate, channels = record_audio(device_index, duration_seconds=3.0)
    
    # Optional: Save to file
    temp_file = os.path.join(tempfile.gettempdir(), "hyperx_test_recording.wav")
    save_wav(audio_data, sample_rate, channels, temp_file)
    
    # Play back
    print("\n" + "=" * 60)
    print("Now playing back your recording...")
    print("=" * 60)
    input("Press ENTER to play back... ")
    
    play_audio(audio_data, sample_rate, channels)
    
    print("\n" + "=" * 60)
    print("✓ Microphone test complete!")
    print("=" * 60)
    print("\nDid you hear your voice clearly?")
    print("  - If YES: Microphone is working correctly!")
    print("  - If NO or too quiet: Check microphone volume/gain settings")
    print("  - If distorted: Lower microphone gain")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n\nTest interrupted")
    except Exception as e:
        print(f"\n✗ Error: {e}")
        import traceback
        traceback.print_exc()

