"""R2D2 Audio Module - Audio generation and playback utilities"""

import struct
import math
import subprocess
from pathlib import Path
from typing import Optional


class AudioBeepGenerator:
    """Generate and play beep sounds via ALSA"""
    
    DEFAULT_DEVICE = "hw:1,0"  # Jetson I2S (APE Card 1, Device 0)
    DEFAULT_SAMPLE_RATE = 44100
    DEFAULT_FREQUENCY = 1000.0
    DEFAULT_DURATION = 0.5
    DEFAULT_VOLUME = 0.5
    
    @staticmethod
    def generate_wav(
        frequency: float = 1000.0,
        duration: float = 0.5,
        sample_rate: int = 44100,
        volume: float = 0.5,
    ) -> bytes:
        """
        Generate a WAV file with a sine wave beep
        
        Args:
            frequency: Tone frequency in Hz (default 1000)
            duration: Duration in seconds (default 0.5)
            sample_rate: Sample rate in Hz (default 44100)
            volume: Volume as fraction 0.0-1.0 (default 0.5 = 50%)
        
        Returns:
            WAV file bytes ready to play
        
        Raises:
            ValueError: If parameters are out of valid ranges
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
        wav_data += struct.pack('<I', 16)  # Subchunk1Size
        wav_data += struct.pack('<H', 1)   # AudioFormat (PCM)
        wav_data += struct.pack('<H', 2)   # NumChannels (stereo)
        wav_data += struct.pack('<I', sample_rate)
        wav_data += struct.pack('<I', sample_rate * 4)  # ByteRate
        wav_data += struct.pack('<H', 4)   # BlockAlign
        wav_data += struct.pack('<H', 16)  # BitsPerSample
        wav_data += b'data'
        wav_data += struct.pack('<I', num_samples * 4)
        
        # Generate sine wave samples
        for i in range(num_samples):
            sample = int(amplitude * math.sin(2 * math.pi * frequency * i / sample_rate))
            sample = max(-32768, min(32767, sample))
            wav_data += struct.pack('<hh', sample, sample)
        
        return wav_data
    
    @staticmethod
    def play_wav(
        wav_data: bytes,
        device: Optional[str] = None,
        timeout: float = 5.0,
    ) -> bool:
        """
        Play WAV data via ALSA aplay
        
        Args:
            wav_data: WAV file bytes to play
            device: ALSA device (default: hw:1,0)
            timeout: Playback timeout in seconds
        
        Returns:
            True if successful, False if failed
        """
        device = device or AudioBeepGenerator.DEFAULT_DEVICE
        
        try:
            temp_wav = Path("/tmp/r2d2_audio.wav")
            temp_wav.write_bytes(wav_data)
            
            cmd = ["aplay", "-D", device, str(temp_wav)]
            result = subprocess.run(cmd, timeout=timeout, capture_output=True)
            
            temp_wav.unlink(missing_ok=True)
            return result.returncode == 0
        except Exception as e:
            print(f"Error playing WAV: {e}")
            return False
    
    @staticmethod
    def beep(
        frequency: float = 1000.0,
        duration: float = 0.5,
        volume: float = 0.5,
        device: Optional[str] = None,
    ) -> bool:
        """
        Generate and play a beep sound
        
        Args:
            frequency: Tone frequency in Hz
            duration: Duration in seconds
            volume: Volume 0.0-1.0 (0.5 = 50%)
            device: ALSA device (default: hw:1,0 for I2S speaker)
        
        Returns:
            True if successful, False if failed
        """
        try:
            wav_data = AudioBeepGenerator.generate_wav(
                frequency=frequency,
                duration=duration,
                volume=volume
            )
            return AudioBeepGenerator.play_wav(wav_data, device=device, timeout=duration + 2)
        except Exception as e:
            print(f"Error generating beep: {e}")
            return False


class AudioStatus:
    """Simple audio status tracking"""
    
    def __init__(self):
        self.last_beep_frequency = 0.0
        self.last_beep_duration = 0.0
        self.last_beep_volume = 0.0
        self.beep_count = 0
        self.device = AudioBeepGenerator.DEFAULT_DEVICE
    
    def update_beep(self, frequency: float, duration: float, volume: float):
        """Update after playing a beep"""
        self.last_beep_frequency = frequency
        self.last_beep_duration = duration
        self.last_beep_volume = volume
        self.beep_count += 1
