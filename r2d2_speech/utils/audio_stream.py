"""
Audio Streaming Pipeline
========================

Complete audio pipeline for HyperX QuadCast S microphone:
- AudioCapture: Continuous microphone capture at native sample rate
- AudioResampler: Streaming-safe resampling to 24kHz (resample_poly)
- AudioPlayback: Speaker output for assistant responses
- AudioStreamManager: Orchestrates the complete pipeline

Device Selection:
- Primary: Use MIC_DEVICE from config
- Fallback: Auto-detect HyperX QuadCast S by name
"""

import pyaudio
import numpy as np
import base64
import logging
from typing import Optional, Tuple
from scipy import signal
from math import gcd

logger = logging.getLogger(__name__)


def find_hyperx_device() -> Optional[Tuple[int, dict]]:
    """
    Auto-detect HyperX QuadCast S microphone by name.
    
    This is a FALLBACK - prefer using MIC_DEVICE from config.
    
    Returns:
        Tuple of (device_index, device_info) or (None, None) if not found
    """
    p = pyaudio.PyAudio()
    
    try:
        for i in range(p.get_device_count()):
            info = p.get_device_info_by_index(i)
            name = info.get('name', '').lower()
            
            # Look for HyperX or QuadCast in device name
            if ('hyperx' in name or 'quadcast' in name) and info.get('maxInputChannels', 0) > 0:
                logger.info(f"Auto-detected HyperX: {info['name']} (index {i})")
                return i, info
    finally:
        p.terminate()
    
    return None, None


def get_device_from_config(config: dict) -> Tuple[int, dict]:
    """
    Get audio device from config, with fallback to auto-detection.
    
    Priority:
    1. Use MIC_DEVICE from config if set
    2. Fallback to auto-detect HyperX QuadCast S
    
    Args:
        config: Configuration dict with optional 'mic_device' key
        
    Returns:
        Tuple of (device_index, device_info)
        
    Raises:
        RuntimeError: If no device found
    """
    p = pyaudio.PyAudio()
    
    try:
        mic_device = config.get('mic_device')
        
        if mic_device:
            logger.info(f"Using MIC_DEVICE from config: {mic_device}")
            
            # Try as PyAudio device index first
            if isinstance(mic_device, int) or (isinstance(mic_device, str) and mic_device.isdigit()):
                device_index = int(mic_device)
                try:
                    info = p.get_device_info_by_index(device_index)
                    if info.get('maxInputChannels', 0) > 0:
                        logger.info(f"Found device at index {device_index}: {info['name']}")
                        return device_index, info
                except:
                    logger.warning(f"Device index {device_index} not found")
            
            # Try matching by name (ALSA format like hw:2,0)
            for i in range(p.get_device_count()):
                info = p.get_device_info_by_index(i)
                if mic_device in info.get('name', ''):
                    logger.info(f"Found device by name match: {info['name']} (index {i})")
                    return i, info
            
            logger.warning(f"MIC_DEVICE '{mic_device}' not found, falling back to auto-detect")
        
        # Fallback: auto-detect HyperX QuadCast S
        logger.info("Using fallback auto-detection for HyperX QuadCast S")
        device_index, info = find_hyperx_device()
        
        if device_index is not None:
            return device_index, info
        
        raise RuntimeError(
            "No audio device found. "
            "Set MIC_DEVICE in ~/.r2d2/.env or connect HyperX QuadCast S"
        )
        
    finally:
        p.terminate()


class AudioCapture:
    """
    Continuous microphone audio capture at native sample rate.
    
    Usage:
        capture = AudioCapture(device_index, native_sample_rate=48000)
        capture.start()
        chunk = capture.read_chunk()
        capture.stop()
    """
    
    def __init__(
        self,
        device_index: int,
        native_sample_rate: int = 48000,
        chunk_size: int = 2400,  # 100ms at 24kHz (will be scaled for native rate)
    ):
        """
        Initialize audio capture.
        
        Args:
            device_index: PyAudio device index
            native_sample_rate: Native sample rate of the microphone
            chunk_size: Target chunk size at 24kHz (will be scaled)
        """
        self.device_index = device_index
        self.native_sample_rate = native_sample_rate
        
        # Scale chunk size for native rate (maintain ~100ms chunks)
        self.chunk_size = int(chunk_size * native_sample_rate / 24000)
        
        self.p = None
        self.stream = None
        self.is_running = False
        
        logger.info(f"AudioCapture initialized:")
        logger.info(f"  Device: {device_index}")
        logger.info(f"  Native rate: {native_sample_rate} Hz")
        logger.info(f"  Chunk size: {self.chunk_size} samples (~100ms)")
    
    def start(self) -> None:
        """
        Start audio capture stream.
        
        Raises:
            RuntimeError: If stream cannot be started
        """
        if self.is_running:
            logger.warning("Capture already running")
            return
        
        self.p = pyaudio.PyAudio()
        
        # Get device info
        device_info = self.p.get_device_info_by_index(self.device_index)
        device_channels = int(device_info.get('maxInputChannels', 1))
        
        logger.info(f"Starting capture from: {device_info['name']}")
        logger.info(f"  Device channels: {device_channels}")
        
        try:
            # Open stream
            self.stream = self.p.open(
                format=pyaudio.paInt16,
                channels=min(device_channels, 2),  # Use up to stereo
                rate=self.native_sample_rate,
                input=True,
                input_device_index=self.device_index,
                frames_per_buffer=self.chunk_size,
                stream_callback=None
            )
            
            self.stream.start_stream()
            self.is_running = True
            logger.info("✓ Audio capture started")
            
        except Exception as e:
            self.stop()
            raise RuntimeError(f"Failed to start audio capture: {e}")
    
    def read_chunk(self) -> Optional[np.ndarray]:
        """
        Read next audio chunk from microphone.
        
        Returns:
            Numpy array of audio samples (mono, int16) or None if stopped
        """
        if not self.is_running or not self.stream:
            return None
        
        try:
            # Read from stream
            data = self.stream.read(self.chunk_size, exception_on_overflow=False)
            
            # Convert to numpy array
            audio_array = np.frombuffer(data, dtype=np.int16)
            
            # Get device info to check channels
            device_info = self.p.get_device_info_by_index(self.device_index)
            device_channels = int(device_info.get('maxInputChannels', 1))
            actual_channels = min(device_channels, 2)
            
            # Convert to mono if stereo
            if actual_channels > 1:
                audio_array = audio_array.reshape(-1, actual_channels)
                audio_array = audio_array.mean(axis=1).astype(np.int16)
            
            return audio_array
            
        except Exception as e:
            logger.error(f"Error reading audio chunk: {e}")
            return None
    
    def stop(self) -> None:
        """
        Stop audio capture and cleanup resources.
        """
        self.is_running = False
        
        if self.stream:
            try:
                self.stream.stop_stream()
                self.stream.close()
            except:
                pass
            self.stream = None
        
        if self.p:
            self.p.terminate()
            self.p = None
        
        logger.info("✓ Audio capture stopped")


class AudioResampler:
    """
    Streaming-safe audio resampling using scipy.signal.resample_poly.
    
    Resamples from native rate to 24kHz for Realtime API.
    Uses polyphase filtering (NOT FFT-based) for streaming compatibility.
    
    Usage:
        resampler = AudioResampler(source_rate=48000, target_rate=24000)
        resampled = resampler.resample_chunk(audio_chunk)
        base64_audio = resampler.to_base64(resampled)
    """
    
    def __init__(self, source_rate: int, target_rate: int = 24000):
        """
        Initialize resampler.
        
        Args:
            source_rate: Source sample rate (native mic rate)
            target_rate: Target sample rate (24000 for Realtime API)
        """
        self.source_rate = source_rate
        self.target_rate = target_rate
        
        # Calculate resampling factors
        g = gcd(source_rate, target_rate)
        self.up = target_rate // g
        self.down = source_rate // g
        
        logger.info(f"AudioResampler initialized:")
        logger.info(f"  Source: {source_rate} Hz")
        logger.info(f"  Target: {target_rate} Hz")
        logger.info(f"  Factors: up={self.up}, down={self.down}")
    
    def resample_chunk(self, audio_chunk: np.ndarray) -> np.ndarray:
        """
        Resample a single audio chunk (streaming-safe).
        
        Uses scipy.signal.resample_poly for polyphase filtering.
        This is streaming-safe and does NOT use FFT.
        
        Args:
            audio_chunk: Input audio (mono, int16)
            
        Returns:
            Resampled audio (mono, int16)
        """
        if len(audio_chunk) == 0:
            return np.array([], dtype=np.int16)
        
        # Convert to float for processing
        audio_float = audio_chunk.astype(np.float32)
        
        # Resample using polyphase filter
        resampled = signal.resample_poly(audio_float, self.up, self.down)
        
        # Convert back to int16
        resampled = np.clip(resampled, -32768, 32767).astype(np.int16)
        
        return resampled
    
    def to_pcm16_mono(self, audio: np.ndarray) -> np.ndarray:
        """
        Ensure audio is PCM16 mono format.
        
        Args:
            audio: Input audio array
            
        Returns:
            PCM16 mono audio
        """
        # Already mono and int16
        if audio.dtype == np.int16 and len(audio.shape) == 1:
            return audio
        
        # Convert to int16 if needed
        if audio.dtype != np.int16:
            audio = np.clip(audio, -32768, 32767).astype(np.int16)
        
        # Flatten to mono if needed
        if len(audio.shape) > 1:
            audio = audio.flatten()
        
        return audio
    
    def to_base64(self, audio: np.ndarray) -> str:
        """
        Convert audio to base64-encoded PCM16 for Realtime API.
        
        Args:
            audio: PCM16 mono audio
            
        Returns:
            Base64-encoded string
        """
        pcm_bytes = audio.tobytes()
        return base64.b64encode(pcm_bytes).decode('utf-8')


class AudioPlayback:
    """
    Audio playback for assistant responses.
    
    Plays base64-encoded PCM16 audio chunks (24kHz mono) to speaker.
    
    Supports master volume control via set_master_volume() method.
    The speech_node subscribes to /r2d2/audio/master_volume and calls
    set_master_volume() to update the playback volume from the physical knob.
    
    Usage:
        playback = AudioPlayback(output_device='default')
        playback.set_master_volume(0.5)  # Set from volume knob
        playback.start()
        playback.play_chunk(base64_audio)
        playback.flush()
        playback.stop()
    """
    
    def __init__(self, output_device: Optional[str] = None):
        """
        Initialize audio playback.
        
        Args:
            output_device: Output device name or index (None for default)
        """
        self.output_device = output_device
        self.input_sample_rate = 24000  # Realtime API outputs 24kHz
        self.channels = 1
        
        self.p = None
        self.stream = None
        self.is_running = False
        self.actual_rate = None  # Will be set when stream opens
        self.resampler = None  # Will be created if resampling needed
        
        # Master volume from physical volume knob (0.0-1.0)
        # Updated externally via set_master_volume() from speech_node
        self.master_volume = 1.0
        
        logger.info(f"AudioPlayback initialized:")
        logger.info(f"  Device: {output_device or 'default'}")
        logger.info(f"  Input rate: {self.input_sample_rate} Hz")
        logger.info(f"  Channels: {self.channels}")
        logger.info(f"  Master volume: {self.master_volume} (updated from /r2d2/audio/master_volume)")
    
    def set_master_volume(self, volume: float) -> None:
        """
        Set master volume from physical volume knob.
        
        Called by speech_node when it receives updates from /r2d2/audio/master_volume topic.
        The volume scales PCM16 samples before playback.
        
        Args:
            volume: Master volume level 0.0-1.0 (0.0 = mute, 1.0 = full volume)
        """
        old_volume = self.master_volume
        self.master_volume = max(0.0, min(1.0, volume))
        
        if abs(old_volume - self.master_volume) > 0.01:
            logger.debug(f"Master volume updated: {old_volume:.2f} -> {self.master_volume:.2f}")
    
    def start(self) -> None:
        """
        Start playback stream.
        
        Raises:
            RuntimeError: If stream cannot be started
        """
        if self.is_running:
            logger.warning("Playback already running")
            return
        
        self.p = pyaudio.PyAudio()
        
        try:
            # Determine output device index
            output_device_index = None
            
            if self.output_device and self.output_device != 'default':
                # Try as index
                if isinstance(self.output_device, int) or self.output_device.isdigit():
                    output_device_index = int(self.output_device)
                else:
                    # Try matching by name
                    for i in range(self.p.get_device_count()):
                        info = self.p.get_device_info_by_index(i)
                        if self.output_device in info.get('name', ''):
                            output_device_index = i
                            break
            
            # Check if device supports the sample rate
            device_info = None
            if output_device_index is not None:
                device_info = self.p.get_device_info_by_index(output_device_index)
            else:
                device_info = self.p.get_default_output_device_info()
            
            # Try to use device's default sample rate if 24kHz not supported
            device_rate = int(device_info.get('defaultSampleRate', 44100))
            actual_rate = self.input_sample_rate
            
            # If device doesn't support 24kHz, try 48kHz or device default
            if device_rate != 24000:
                # Try common rates: 48kHz, 44.1kHz, 16kHz
                for test_rate in [48000, 44100, 16000]:
                    try:
                        test_stream = self.p.open(
                            format=pyaudio.paInt16,
                            channels=self.channels,
                            rate=test_rate,
                            output=True,
                            output_device_index=output_device_index,
                            frames_per_buffer=0
                        )
                        test_stream.close()
                        actual_rate = test_rate
                        logger.info(f"Device supports {test_rate} Hz, using that instead of 24kHz")
                        break
                    except:
                        continue
                else:
                    # Fall back to device default
                    actual_rate = device_rate
                    logger.warning(f"Using device default rate {actual_rate} Hz (24kHz not supported)")
            
            # Open stream with determined rate
            self.stream = self.p.open(
                format=pyaudio.paInt16,
                channels=self.channels,
                rate=actual_rate,
                output=True,
                output_device_index=output_device_index,
                frames_per_buffer=int(actual_rate * 0.1)  # 100ms buffer
            )
            
            # Store actual rate and create resampler if needed
            self.actual_rate = actual_rate
            if actual_rate != self.input_sample_rate:
                self.resampler = AudioResampler(self.input_sample_rate, actual_rate)
                logger.info(f"Created resampler: {self.input_sample_rate} Hz → {actual_rate} Hz")
            
            self.stream.start_stream()
            self.is_running = True
            logger.info("✓ Audio playback started")
            
        except Exception as e:
            self.stop()
            raise RuntimeError(f"Failed to start audio playback: {e}")
    
    def play_chunk(self, base64_audio: str) -> None:
        """
        Play a base64-encoded audio chunk.
        
        The audio is scaled by master_volume before playback.
        master_volume comes from the physical volume knob via /r2d2/audio/master_volume topic.
        
        Args:
            base64_audio: Base64-encoded PCM16 audio (24kHz mono)
        """
        if not self.is_running or not self.stream:
            logger.warning("Playback not running")
            return
        
        try:
            # Decode base64
            audio_bytes = base64.b64decode(base64_audio)
            audio_array = np.frombuffer(audio_bytes, dtype=np.int16)
            
            # Apply master volume scaling (from physical volume knob)
            if self.master_volume < 1.0:
                # Scale PCM16 samples by master volume
                # Convert to float32, scale, then back to int16 to avoid overflow
                audio_float = audio_array.astype(np.float32) * self.master_volume
                audio_array = np.clip(audio_float, -32768, 32767).astype(np.int16)
            
            # Resample if needed
            if self.resampler:
                audio_array = self.resampler.resample_chunk(audio_array)
            
            # Convert back to bytes
            audio_bytes = audio_array.tobytes()
            
            # Write to stream
            self.stream.write(audio_bytes)
            
        except Exception as e:
            logger.error(f"Error playing audio chunk: {e}")
    
    def flush(self) -> None:
        """
        Ensure all audio has been played.
        """
        if self.stream:
            try:
                # PyAudio streams are non-blocking, but we can sleep briefly
                import time
                time.sleep(0.1)
            except:
                pass
    
    def stop(self) -> None:
        """
        Stop playback and cleanup resources.
        """
        self.is_running = False
        
        if self.stream:
            try:
                self.stream.stop_stream()
                self.stream.close()
            except:
                pass
            self.stream = None
        
        if self.p:
            self.p.terminate()
            self.p = None
        
        logger.info("✓ Audio playback stopped")


class AudioStreamManager:
    """
    Orchestrates complete audio pipeline: capture → resample → send to API.
    
    Handles:
    - Continuous microphone capture at native rate
    - Streaming-safe resampling to 24kHz
    - Sending audio to Realtime API
    - Explicit buffer commits
    
    Usage:
        manager = AudioStreamManager(config, realtime_client)
        await manager.start()
        # Audio streams continuously
        await manager.stop()
    """
    
    def __init__(self, config: dict, realtime_client):
        """
        Initialize audio stream manager.
        
        Args:
            config: Configuration dict
            realtime_client: RealtimeClient instance
        """
        self.config = config
        self.client = realtime_client
        
        # Get device
        self.device_index, self.device_info = get_device_from_config(config)
        
        # Get sample rates
        self.native_rate = config.get('mic_native_sample_rate', 48000)
        self.target_rate = 24000  # Realtime API requirement
        
        # Initialize components
        self.capture = AudioCapture(self.device_index, self.native_rate)
        self.resampler = AudioResampler(self.native_rate, self.target_rate)
        
        # Playback (optional, will be initialized if needed)
        self.playback = None
        
        # State
        self.is_running = False
        self.commit_interval = 1.0  # Commit every 1 second
        self.last_commit_time = 0
        
        logger.info("AudioStreamManager initialized")
    
    async def start(self) -> None:
        """
        Start audio streaming pipeline.
        
        This starts:
        - Audio capture
        - Continuous processing loop
        """
        if self.is_running:
            logger.warning("Stream already running")
            return
        
        # Start capture
        self.capture.start()
        self.is_running = True
        
        logger.info("✓ Audio streaming started")
    
    async def process_and_send(self) -> bool:
        """
        Process one audio chunk and send to Realtime API.
        
        Returns:
            True if chunk processed, False if stopped or error
        """
        if not self.is_running:
            return False
        
        # Read chunk from microphone
        audio_chunk = self.capture.read_chunk()
        
        if audio_chunk is None or len(audio_chunk) == 0:
            return False
        
        # Resample to 24kHz
        resampled = self.resampler.resample_chunk(audio_chunk)
        
        # Convert to base64
        base64_audio = self.resampler.to_base64(resampled)
        
        # Send to Realtime API
        await self.client.send_audio_frame(base64_audio)
        
        # Note: With server VAD enabled, we don't need to manually commit buffers.
        # The server will automatically detect speech, buffer it, transcribe, and respond.
        # Manual commits would cause "buffer too small" errors.
        
        return True
    
    async def commit_buffer(self) -> None:
        """
        Explicitly commit audio buffer to Realtime API.
        
        This triggers transcription even with server VAD enabled.
        """
        await self.client.commit_audio()
        logger.debug("Audio buffer committed")
    
    async def stop(self) -> None:
        """
        Stop audio streaming pipeline.
        """
        self.is_running = False
        
        # Stop capture
        self.capture.stop()
        
        logger.info("✓ Audio streaming stopped")
    
    def start_playback(self) -> None:
        """
        Start audio playback for assistant responses.
        """
        if self.playback is None:
            output_device = self.config.get('sink_device', 'default')
            self.playback = AudioPlayback(output_device)
        
        if not self.playback.is_running:
            self.playback.start()
    
    def stop_playback(self) -> None:
        """
        Stop audio playback.
        """
        if self.playback:
            self.playback.stop()


if __name__ == "__main__":
    # Test device detection
    import sys
    sys.path.insert(0, str(__file__).rsplit('/', 3)[0])
    
    from config import get_config
    
    logging.basicConfig(level=logging.INFO)
    
    print("=" * 60)
    print("Audio Stream Module Test")
    print("=" * 60)
    
    try:
        # Load config
        config = get_config()
        print("\n1. Device Detection")
        print("-" * 60)
        
        device_index, device_info = get_device_from_config(config)
        print(f"✓ Device: {device_info['name']}")
        print(f"  Index: {device_index}")
        print(f"  Channels: {device_info['maxInputChannels']}")
        print(f"  Sample Rate: {int(device_info['defaultSampleRate'])} Hz")
        
        print("\n2. Resampler Test")
        print("-" * 60)
        
        native_rate = config.get('mic_native_sample_rate', 48000)
        resampler = AudioResampler(native_rate, 24000)
        
        # Test with dummy audio
        test_audio = np.zeros(4800, dtype=np.int16)  # 100ms at 48kHz
        resampled = resampler.resample_chunk(test_audio)
        print(f"✓ Input: {len(test_audio)} samples @ {native_rate} Hz")
        print(f"  Output: {len(resampled)} samples @ 24000 Hz")
        print(f"  Expected: ~2400 samples")
        
        print("\n✅ All tests passed")
        
    except Exception as e:
        print(f"\n✗ Error: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)

