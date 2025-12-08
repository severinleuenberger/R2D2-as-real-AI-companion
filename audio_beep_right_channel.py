#!/usr/bin/env python3
"""
Audio beep utility that sends to RIGHT channel only
"""

import sys
import struct
import math
import subprocess
from pathlib import Path

def generate_mono_wav(
    frequency: float = 1000.0,
    duration: float = 0.5,
    sample_rate: int = 44100,
    volume: float = 0.5,
) -> bytes:
    """Generate MONO WAV (will be sent to RIGHT channel only)"""
    
    num_samples = int(sample_rate * duration)
    amplitude = int(32767 * volume)
    
    # Build WAV header for MONO (1 channel)
    wav_data = b'RIFF'
    file_size = 36 + num_samples * 2  # 2 bytes per sample (MONO 16-bit)
    wav_data += struct.pack('<I', file_size)
    wav_data += b'WAVEfmt '
    wav_data += struct.pack('<I', 16)  # Subchunk1Size
    wav_data += struct.pack('<H', 1)   # AudioFormat (PCM)
    wav_data += struct.pack('<H', 1)   # NumChannels (MONO - 1 channel)
    wav_data += struct.pack('<I', sample_rate)
    wav_data += struct.pack('<I', sample_rate * 2)  # ByteRate
    wav_data += struct.pack('<H', 2)   # BlockAlign
    wav_data += struct.pack('<H', 16)  # BitsPerSample
    wav_data += b'data'
    wav_data += struct.pack('<I', num_samples * 2)
    
    # Generate sine wave samples (mono)
    for i in range(num_samples):
        sample = int(amplitude * math.sin(2 * math.pi * frequency * i / sample_rate))
        sample = max(-32768, min(32767, sample))
        wav_data += struct.pack('<h', sample)
    
    return wav_data


def play_on_right_channel(
    frequency: float = 1000.0,
    duration: float = 0.5,
    volume: float = 0.5,
) -> bool:
    """Play MONO beep on RIGHT channel using channel map"""
    
    try:
        # Generate MONO WAV
        wav_data = generate_mono_wav(frequency, duration, volume)
        
        # Create temp file
        temp_wav = Path("/tmp/r2d2_beep_mono.wav")
        temp_wav.write_bytes(wav_data)
        
        # Use ALSA channel mapping to send mono to RIGHT channel
        # Using 'plug' plugin to map mono to stereo with right-only output
        cmd = [
            "aplay",
            "-D", "plug:speaker_right",  # Use our defined plugin
            str(temp_wav)
        ]
        
        result = subprocess.run(
            cmd,
            timeout=duration + 2,
            capture_output=True,
            text=True
        )
        
        temp_wav.unlink(missing_ok=True)
        return result.returncode == 0
    
    except Exception as e:
        print(f"Error: {e}", file=sys.stderr)
        return False


if __name__ == "__main__":
    import argparse
    
    parser = argparse.ArgumentParser(description="Play beep on RIGHT channel")
    parser.add_argument("--frequency", "-f", type=float, default=1000.0)
    parser.add_argument("--duration", "-d", type=float, default=0.5)
    parser.add_argument("--volume", "-v", type=float, default=0.5)
    
    args = parser.parse_args()
    
    print(f"Playing beep on RIGHT channel: {args.frequency}Hz, {args.duration}s, {args.volume*100:.0f}%")
    
    if play_on_right_channel(args.frequency, args.duration, args.volume):
        print("✓ Beep played")
        sys.exit(0)
    else:
        print("✗ Failed to play beep")
        sys.exit(1)
