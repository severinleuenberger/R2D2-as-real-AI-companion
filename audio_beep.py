#!/usr/bin/env python3
"""
Simple audio beep utility for R2D2
Generates and plays a beep sound at specified frequency and volume
"""

import sys
import struct
import math
import subprocess
from pathlib import Path
from typing import Optional

def generate_beep(
    frequency: float = 1000.0,
    duration: float = 0.5,
    sample_rate: int = 44100,
    volume: float = 0.5,
) -> bytes:
    """
    Generate a sine wave beep as WAV audio data
    
    Args:
        frequency: Tone frequency in Hz (default 1000 Hz)
        duration: Duration in seconds (default 0.5 sec)
        sample_rate: Sample rate in Hz (default 44100)
        volume: Volume as fraction 0.0-1.0 (default 0.5 = 50%)
    
    Returns:
        WAV file bytes ready to play
    """
    if not 20 <= frequency <= 20000:
        raise ValueError(f"Frequency must be 20-20000 Hz, got {frequency}")
    if not 0.1 <= duration <= 10.0:
        raise ValueError(f"Duration must be 0.1-10.0 sec, got {duration}")
    if not 0.0 <= volume <= 1.0:
        raise ValueError(f"Volume must be 0.0-1.0, got {volume}")
    
    num_samples = int(sample_rate * duration)
    amplitude = int(32767 * volume)  # 16-bit signed max
    
    # Build WAV header
    wav_data = b'RIFF'
    file_size = 36 + num_samples * 4  # 4 bytes per sample (stereo 16-bit)
    wav_data += struct.pack('<I', file_size)
    wav_data += b'WAVEfmt '
    wav_data += struct.pack('<I', 16)  # Subchunk1Size (16 for PCM)
    wav_data += struct.pack('<H', 1)   # AudioFormat (1 = PCM)
    wav_data += struct.pack('<H', 2)   # NumChannels (2 = stereo)
    wav_data += struct.pack('<I', sample_rate)
    wav_data += struct.pack('<I', sample_rate * 4)  # ByteRate
    wav_data += struct.pack('<H', 4)   # BlockAlign
    wav_data += struct.pack('<H', 16)  # BitsPerSample
    wav_data += b'data'
    wav_data += struct.pack('<I', num_samples * 4)
    
    # Generate sine wave samples
    for i in range(num_samples):
        sample = int(amplitude * math.sin(2 * math.pi * frequency * i / sample_rate))
        # Clamp to 16-bit range
        sample = max(-32768, min(32767, sample))
        # Write stereo (same value to both channels)
        wav_data += struct.pack('<hh', sample, sample)
    
    return wav_data


def play_beep(
    frequency: float = 1000.0,
    duration: float = 0.5,
    volume: float = 0.5,
    device: Optional[str] = None,
) -> bool:
    """
    Generate and play a beep sound
    
    Args:
        frequency: Tone frequency in Hz (default 1000)
        duration: Duration in seconds (default 0.5)
        volume: Volume 0.0-1.0 (default 0.5 = 50%)
        device: ALSA device (e.g., 'hw:1,0'). If None, uses system default
    
    Returns:
        True if successful, False if failed
    """
    try:
        # Generate WAV data
        wav_data = generate_beep(
            frequency=frequency,
            duration=duration,
            sample_rate=44100,
            volume=volume
        )
        
        # Create temp file
        temp_wav = Path("/tmp/r2d2_beep.wav")
        temp_wav.write_bytes(wav_data)
        
        # Build aplay command
        cmd = ["aplay"]
        if device:
            cmd.extend(["-D", device])
        cmd.append(str(temp_wav))
        
        # Play with timeout
        result = subprocess.run(
            cmd,
            timeout=duration + 2,
            capture_output=True,
            text=True
        )
        
        # Cleanup
        temp_wav.unlink(missing_ok=True)
        
        return result.returncode == 0
    
    except Exception as e:
        print(f"Error playing beep: {e}", file=sys.stderr)
        return False


def main():
    """CLI interface for audio beep utility"""
    import argparse
    
    parser = argparse.ArgumentParser(
        description="Generate and play a beep sound"
    )
    parser.add_argument(
        "--frequency", "-f",
        type=float,
        default=1000.0,
        help="Frequency in Hz, default 1000"
    )
    parser.add_argument(
        "--duration", "-d",
        type=float,
        default=0.5,
        help="Duration in seconds, default 0.5"
    )
    parser.add_argument(
        "--volume", "-v",
        type=float,
        default=0.5,
        help="Volume 0.0-1.0, default 0.5 = 50 percent"
    )
    parser.add_argument(
        "--device",
        type=str,
        default=None,
        help="ALSA device, e.g. hw:1,0 for I2S speaker"
    )
    parser.add_argument(
        "--list-devices",
        action="store_true",
        help="List available ALSA devices and exit"
    )
    
    args = parser.parse_args()
    
    # List devices if requested
    if args.list_devices:
        print("Available ALSA playback devices:")
        subprocess.run(["aplay", "-l"])
        return 0
    
    # Validate arguments
    if not 20 <= args.frequency <= 20000:
        print(f"Error: frequency must be 20-20000 Hz", file=sys.stderr)
        return 1
    if not 0.1 <= args.duration <= 10.0:
        print(f"Error: duration must be 0.1-10.0 seconds", file=sys.stderr)
        return 1
    if not 0.0 <= args.volume <= 1.0:
        print(f"Error: volume must be 0.0-1.0", file=sys.stderr)
        return 1
    
    print(f"Playing beep: {args.frequency}Hz, {args.duration}s, volume {args.volume*100:.0f}%")
    if args.device:
        print(f"Device: {args.device}")
    
    success = play_beep(
        frequency=args.frequency,
        duration=args.duration,
        volume=args.volume,
        device=args.device
    )
    
    if success:
        print("✓ Beep played successfully")
        return 0
    else:
        print("✗ Failed to play beep", file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main())
