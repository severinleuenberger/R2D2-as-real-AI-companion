"""
Improved Microphone Test with Better Audio Quality
==================================================

Fixed version with:
- Larger buffers to prevent underruns
- Proper scipy resampling (not linear interpolation)
- Better error handling

Usage:
    cd ~/dev/r2d2/r2d2_speech
    python test_mic_improved.py
"""

import pyaudio
import numpy as np
import wave
import tempfile
import os
from scipy import signal


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


def record_audio_improved(device_index, duration_seconds=3.0):
    """
    Record audio with larger buffers and better handling.
    
    Returns audio at NATIVE sample rate (no resampling during capture).
    """
    p = pyaudio.PyAudio()
    
    device_info = p.get_device_info_by_index(device_index)
    sample_rate = int(device_info['defaultSampleRate'])
    channels = min(device_info['maxInputChannels'], 2)
    
    print(f"\n🎤 Recording {duration_seconds} seconds...")
    print(f"  - Sample rate: {sample_rate} Hz")
    print(f"  - Channels: {channels}")
    print(f"  - Buffer: 2048 frames (larger for stability)")
    
    # Use LARGER buffer to prevent underruns
    CHUNK = 2048
    
    stream = p.open(
        format=pyaudio.paInt16,
        channels=channels,
        rate=sample_rate,
        input=True,
        input_device_index=device_index,
        frames_per_buffer=CHUNK,
        stream_callback=None
    )
    
    print("🔴 RECORDING NOW - SPEAK CLEARLY!")
    frames = []
    num_chunks = int(sample_rate * duration_seconds / CHUNK)
    
    for i in range(num_chunks):
        try:
            data = stream.read(CHUNK, exception_on_overflow=False)
            frames.append(data)
            
            if i % (num_chunks // 10) == 0:
                progress = int((i / num_chunks) * 100)
                print(f"  Recording... {progress}%")
        except IOError as e:
            print(f"  Warning: Buffer issue at {i}/{num_chunks}: {e}")
            # Continue anyway
    
    print("✓ Recording complete!")
    
    stream.stop_stream()
    stream.close()
    
    # Convert to numpy array
    audio_data = b''.join(frames)
    audio_array = np.frombuffer(audio_data, dtype=np.int16)
    
    # Convert to mono if stereo
    if channels > 1:
        audio_array = audio_array.reshape(-1, channels)
        audio_array = audio_array.mean(axis=1).astype(np.int16)
        print("  - Converted to mono")
    
    # Analyze audio
    rms = np.sqrt(np.mean(audio_array.astype(float)**2))
    peak = np.max(np.abs(audio_array))
    
    if rms > 0:
        rms_db = 20 * np.log10(rms / 32768.0)
        peak_db = 20 * np.log10(peak / 32768.0)
        print(f"\n📊 Audio Analysis:")
        print(f"  - RMS Level: {rms_db:.1f} dB")
        print(f"  - Peak Level: {peak_db:.1f} dB")
        
        if rms_db < -40:
            print("  ⚠ Warning: Audio very quiet!")
        elif rms_db > -10:
            print("  ⚠ Warning: Audio might clip!")
        else:
            print("  ✓ Good audio level!")
    
    p.terminate()
    return audio_array, sample_rate


def resample_to_24khz(audio_array, original_rate):
    """
    Properly resample audio to 24kHz using scipy.
    
    This is MUCH better than linear interpolation!
    """
    if original_rate == 24000:
        print("  - Already at 24kHz, no resampling needed")
        return audio_array
    
    print(f"  - Resampling {original_rate}Hz → 24000Hz using scipy...")
    
    # Calculate number of samples needed
    num_samples = int(len(audio_array) * 24000 / original_rate)
    
    # Use scipy's high-quality resampling
    resampled = signal.resample(audio_array, num_samples)
    
    # Convert back to int16
    resampled = np.clip(resampled, -32768, 32767).astype(np.int16)
    
    print(f"  - Resampled: {len(audio_array)} → {len(resampled)} samples")
    
    return resampled


def save_wav(audio_array, sample_rate, filename):
    """Save audio to WAV file."""
    with wave.open(filename, 'wb') as wf:
        wf.setnchannels(1)  # Mono
        wf.setsampwidth(2)  # 16-bit
        wf.setframerate(sample_rate)
        wf.writeframes(audio_array.tobytes())
    print(f"💾 Saved to: {filename}")


def main():
    """Main test function."""
    print("=" * 60)
    print("Improved Microphone Test - Better Audio Quality")
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
    
    # Record at NATIVE sample rate
    audio_native, native_rate = record_audio_improved(device_index, duration_seconds=3.0)
    
    # Save native recording
    native_file = "/tmp/hyperx_native.wav"
    save_wav(audio_native, native_rate, native_file)
    print(f"\n✓ Native recording saved: {native_rate}Hz")
    
    # Resample to 24kHz (for Realtime API)
    audio_24k = resample_to_24khz(audio_native, native_rate)
    
    # Save resampled version
    resampled_file = "/tmp/hyperx_24khz.wav"
    save_wav(audio_24k, 24000, resampled_file)
    print(f"✓ Resampled recording saved: 24000Hz")
    
    # Play back NATIVE version (should sound good)
    print("\n" + "=" * 60)
    print("Playing back NATIVE recording...")
    print("=" * 60)
    input("Press ENTER to play native recording... ")
    os.system(f"aplay {native_file}")
    
    # Play back RESAMPLED version (for API)
    print("\n" + "=" * 60)
    print("Playing back RESAMPLED (24kHz) recording...")
    print("=" * 60)
    input("Press ENTER to play 24kHz version... ")
    os.system(f"aplay {resampled_file}")
    
    print("\n" + "=" * 60)
    print("✓ Test complete!")
    print("=" * 60)
    print("\nCompare the two playbacks:")
    print("  1. Native recording - should sound clear")
    print("  2. Resampled (24kHz) - should also sound clear")
    print("\nIf BOTH sound good, the resampling works!")
    print("If resampled sounds bad, we have a resampling issue.")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n\nTest interrupted")
    except Exception as e:
        print(f"\n✗ Error: {e}")
        import traceback
        traceback.print_exc()

