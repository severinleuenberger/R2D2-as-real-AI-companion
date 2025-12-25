"""
REST Speech Client
==================

Turn-based speech pipeline using OpenAI REST APIs for "Intelligent Mode".

Pipeline:
1. AudioRecorder - Records audio with silence detection
2. Whisper API - Transcribes audio to text
3. Chat Completions API - Generates response (o1-preview or other models)
4. TTS API - Synthesizes speech from response
5. AudioPlayback - Plays the synthesized audio

This provides slower but more thoughtful/intelligent responses compared
to the Realtime API's "Fast Mode".
"""

import asyncio
import base64
import io
import logging
import tempfile
import time
from datetime import datetime
from pathlib import Path
from typing import Optional, Dict, Any, List, Callable

import numpy as np
import pyaudio
from scipy import signal
from scipy.io import wavfile
import httpx

logger = logging.getLogger(__name__)


class AudioRecorder:
    """
    Records audio with Voice Activity Detection (VAD) using silence detection.
    
    Features:
    - Configurable silence threshold and duration
    - Automatic recording start on speech detection
    - Automatic recording stop after silence
    - WAV file output for Whisper API
    """
    
    def __init__(
        self,
        device_index: int,
        sample_rate: int = 16000,
        channels: int = 1,
        chunk_size: int = 1024,
        silence_threshold: float = 500.0,
        silence_duration: float = 1.5,
        max_duration: float = 30.0,
    ):
        """
        Initialize audio recorder.
        
        Args:
            device_index: PyAudio device index
            sample_rate: Recording sample rate (16kHz for Whisper)
            channels: Number of audio channels (mono for Whisper)
            chunk_size: Frames per buffer
            silence_threshold: RMS threshold below which audio is silence
            silence_duration: Seconds of silence to stop recording
            max_duration: Maximum recording duration in seconds
        """
        self.device_index = device_index
        self.sample_rate = sample_rate
        self.channels = channels
        self.chunk_size = chunk_size
        self.silence_threshold = silence_threshold
        self.silence_duration = silence_duration
        self.max_duration = max_duration
        
        self.p = None
        self.stream = None
        self.is_recording = False
        
        logger.info(f"AudioRecorder initialized:")
        logger.info(f"  Device: {device_index}")
        logger.info(f"  Sample rate: {sample_rate} Hz")
        logger.info(f"  Silence threshold: {silence_threshold}")
        logger.info(f"  Silence duration: {silence_duration}s")
    
    def _calculate_rms(self, audio_chunk: np.ndarray) -> float:
        """Calculate Root Mean Square (RMS) of audio chunk."""
        return float(np.sqrt(np.mean(audio_chunk.astype(np.float64) ** 2)))
    
    async def record_until_silence(
        self,
        on_recording_start: Optional[Callable] = None,
        on_speech_detected: Optional[Callable] = None,
    ) -> Optional[bytes]:
        """
        Record audio until silence is detected.
        
        This method:
        1. Waits for speech to be detected (above threshold)
        2. Records all audio
        3. Stops after silence_duration of quiet
        
        Args:
            on_recording_start: Callback when recording starts
            on_speech_detected: Callback when speech detected
            
        Returns:
            WAV audio bytes or None if recording failed
        """
        self.p = pyaudio.PyAudio()
        
        try:
            # Get device info
            device_info = self.p.get_device_info_by_index(self.device_index)
            native_rate = int(device_info.get('defaultSampleRate', 48000))
            device_channels = int(device_info.get('maxInputChannels', 1))
            
            logger.info(f"Recording from: {device_info['name']}")
            logger.info(f"  Native rate: {native_rate} Hz")
            logger.info(f"  Device channels: {device_channels}")
            
            # Open stream at native rate (we'll resample if needed)
            actual_rate = native_rate
            use_channels = min(device_channels, 2)
            
            self.stream = self.p.open(
                format=pyaudio.paInt16,
                channels=use_channels,
                rate=actual_rate,
                input=True,
                input_device_index=self.device_index,
                frames_per_buffer=int(self.chunk_size * actual_rate / self.sample_rate),
            )
            
            self.is_recording = True
            
            if on_recording_start:
                on_recording_start()
            
            # Recording state
            audio_frames = []
            speech_started = False
            silence_start_time = None
            recording_start_time = time.time()
            
            logger.info("Listening for speech...")
            
            while self.is_recording:
                # Check max duration
                elapsed = time.time() - recording_start_time
                if elapsed > self.max_duration:
                    logger.info(f"Max recording duration reached ({self.max_duration}s)")
                    break
                
                # Read audio chunk
                try:
                    data = self.stream.read(
                        int(self.chunk_size * actual_rate / self.sample_rate),
                        exception_on_overflow=False
                    )
                except Exception as e:
                    logger.warning(f"Stream read error: {e}")
                    await asyncio.sleep(0.01)
                    continue
                
                # Convert to numpy array
                audio_chunk = np.frombuffer(data, dtype=np.int16)
                
                # Convert stereo to mono if needed
                if use_channels > 1:
                    audio_chunk = audio_chunk.reshape(-1, use_channels)
                    audio_chunk = audio_chunk.mean(axis=1).astype(np.int16)
                
                # Calculate RMS
                rms = self._calculate_rms(audio_chunk)
                
                if rms > self.silence_threshold:
                    # Speech detected
                    if not speech_started:
                        speech_started = True
                        logger.info(f"Speech detected (RMS: {rms:.1f})")
                        if on_speech_detected:
                            on_speech_detected()
                    
                    silence_start_time = None
                    audio_frames.append(audio_chunk)
                    
                elif speech_started:
                    # Silence during recording
                    audio_frames.append(audio_chunk)
                    
                    if silence_start_time is None:
                        silence_start_time = time.time()
                    elif time.time() - silence_start_time > self.silence_duration:
                        logger.info(f"Silence detected, stopping ({self.silence_duration}s)")
                        break
                
                # Allow other async tasks to run
                await asyncio.sleep(0.01)
            
            # Stop recording
            self.is_recording = False
            
            if not audio_frames:
                logger.warning("No audio recorded")
                return None
            
            # Concatenate all frames
            audio_data = np.concatenate(audio_frames)
            
            logger.info(f"Recorded {len(audio_data)} samples ({len(audio_data)/actual_rate:.2f}s)")
            
            # Resample to target sample rate if needed
            if actual_rate != self.sample_rate:
                from math import gcd
                g = gcd(actual_rate, self.sample_rate)
                up = self.sample_rate // g
                down = actual_rate // g
                audio_data = signal.resample_poly(audio_data.astype(np.float32), up, down)
                audio_data = np.clip(audio_data, -32768, 32767).astype(np.int16)
                logger.info(f"Resampled to {self.sample_rate} Hz: {len(audio_data)} samples")
            
            # Convert to WAV bytes
            wav_buffer = io.BytesIO()
            wavfile.write(wav_buffer, self.sample_rate, audio_data)
            wav_bytes = wav_buffer.getvalue()
            
            logger.info(f"WAV file created: {len(wav_bytes)} bytes")
            
            return wav_bytes
            
        except Exception as e:
            logger.error(f"Recording error: {e}")
            import traceback
            traceback.print_exc()
            return None
            
        finally:
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
    
    def stop(self):
        """Stop recording."""
        self.is_recording = False


class RestSpeechClient:
    """
    Turn-based speech client using OpenAI REST APIs.
    
    Pipeline: Record → Whisper STT → Chat Completions → TTS → Playback
    
    Usage:
        client = RestSpeechClient(
            api_key="sk-...",
            instructions="You are a helpful assistant.",
            model="o1-preview",
            voice="nova"
        )
        await client.process_turn()
    """
    
    def __init__(
        self,
        api_key: str,
        instructions: str = "You are a helpful AI assistant.",
        model: str = "o1-preview",
        stt_model: str = "whisper-1",
        tts_model: str = "tts-1",
        tts_voice: str = "nova",
        tts_speed: float = 1.0,
        temperature: float = 0.7,
        device_index: int = 0,
        silence_threshold: float = 500.0,
        silence_duration: float = 1.5,
        output_device: Optional[str] = None,
    ):
        """
        Initialize REST speech client.
        
        Args:
            api_key: OpenAI API key
            instructions: System instructions for the LLM
            model: Chat model (o1-preview, gpt-4-turbo, gpt-4o, etc.)
            stt_model: Whisper model for transcription
            tts_model: TTS model (tts-1, tts-1-hd)
            tts_voice: TTS voice (alloy, echo, fable, onyx, nova, shimmer)
            tts_speed: TTS speed multiplier (0.25 to 4.0)
            temperature: LLM temperature (not supported by o1 models)
            device_index: Audio input device index
            silence_threshold: RMS threshold for silence detection
            silence_duration: Seconds of silence to stop recording
            output_device: Audio output device (None for default)
        """
        self.api_key = api_key
        self.instructions = instructions
        self.model = model
        self.stt_model = stt_model
        self.tts_model = tts_model
        self.tts_voice = tts_voice
        self.tts_speed = tts_speed
        self.temperature = temperature
        self.output_device = output_device
        
        # Conversation history
        self.conversation_history: List[Dict[str, str]] = []
        
        # Session tracking
        self.session_id: Optional[str] = None
        self.is_active = False
        
        # Audio recorder
        self.recorder = AudioRecorder(
            device_index=device_index,
            silence_threshold=silence_threshold,
            silence_duration=silence_duration,
        )
        
        # HTTP client with retry
        self.http_client = httpx.AsyncClient(
            timeout=httpx.Timeout(300.0, connect=30.0),  # Long timeout for o1
            headers={
                "Authorization": f"Bearer {api_key}",
            }
        )
        
        # Audio playback
        self.playback_p = None
        self.playback_stream = None
        
        logger.info(f"RestSpeechClient initialized:")
        logger.info(f"  STT model: {stt_model}")
        logger.info(f"  LLM model: {model}")
        logger.info(f"  TTS model: {tts_model}")
        logger.info(f"  TTS voice: {tts_voice}")
    
    def start_session(self, session_id: Optional[str] = None) -> str:
        """
        Start a new conversation session.
        
        Args:
            session_id: Optional session ID (generated if not provided)
            
        Returns:
            Session ID
        """
        if session_id is None:
            session_id = f"rest-{datetime.now().strftime('%Y%m%d_%H%M%S')}"
        
        self.session_id = session_id
        self.conversation_history = []
        self.is_active = True
        
        # Add system instructions to conversation
        if self.instructions:
            # Note: o1 models don't support system role, use developer role
            if self.model.startswith("o1"):
                # o1 models use 'user' for system-like instructions
                self.conversation_history.append({
                    "role": "user",
                    "content": f"[System Instructions]: {self.instructions}"
                })
            else:
                self.conversation_history.append({
                    "role": "system",
                    "content": self.instructions
                })
        
        logger.info(f"Session started: {session_id}")
        return session_id
    
    def end_session(self):
        """End the current session."""
        self.is_active = False
        self.recorder.stop()
        logger.info(f"Session ended: {self.session_id}")
    
    async def transcribe_audio(self, wav_bytes: bytes) -> Optional[str]:
        """
        Transcribe audio using Whisper API.
        
        Args:
            wav_bytes: WAV audio bytes
            
        Returns:
            Transcription text or None if failed
        """
        logger.info("Transcribing audio with Whisper...")
        start_time = time.time()
        
        try:
            # Create multipart form data
            files = {
                "file": ("audio.wav", wav_bytes, "audio/wav"),
            }
            data = {
                "model": self.stt_model,
                "language": "en",
            }
            
            response = await self.http_client.post(
                "https://api.openai.com/v1/audio/transcriptions",
                files=files,
                data=data,
            )
            
            response.raise_for_status()
            result = response.json()
            
            transcript = result.get("text", "").strip()
            duration = time.time() - start_time
            
            logger.info(f"Transcription complete ({duration:.2f}s): {transcript}")
            return transcript
            
        except Exception as e:
            logger.error(f"Transcription failed: {e}")
            return None
    
    async def generate_response(self, user_message: str) -> Optional[str]:
        """
        Generate response using Chat Completions API.
        
        Args:
            user_message: User's message (transcribed text)
            
        Returns:
            Assistant's response text or None if failed
        """
        logger.info(f"Generating response with {self.model}...")
        start_time = time.time()
        
        # Add user message to history
        self.conversation_history.append({
            "role": "user",
            "content": user_message
        })
        
        try:
            # Build request payload
            payload = {
                "model": self.model,
                "messages": self.conversation_history,
            }
            
            # o1 models don't support temperature parameter
            if not self.model.startswith("o1"):
                payload["temperature"] = self.temperature
            
            response = await self.http_client.post(
                "https://api.openai.com/v1/chat/completions",
                json=payload,
            )
            
            response.raise_for_status()
            result = response.json()
            
            assistant_message = result["choices"][0]["message"]["content"]
            duration = time.time() - start_time
            
            # Add assistant message to history
            self.conversation_history.append({
                "role": "assistant",
                "content": assistant_message
            })
            
            logger.info(f"Response generated ({duration:.2f}s): {assistant_message[:100]}...")
            return assistant_message
            
        except Exception as e:
            logger.error(f"Response generation failed: {e}")
            # Remove the user message if response failed
            self.conversation_history.pop()
            return None
    
    async def synthesize_speech(self, text: str) -> Optional[bytes]:
        """
        Synthesize speech using TTS API.
        
        Args:
            text: Text to synthesize
            
        Returns:
            MP3 audio bytes or None if failed
        """
        logger.info(f"Synthesizing speech with {self.tts_model}...")
        start_time = time.time()
        
        try:
            payload = {
                "model": self.tts_model,
                "input": text,
                "voice": self.tts_voice,
                "speed": self.tts_speed,
            }
            
            response = await self.http_client.post(
                "https://api.openai.com/v1/audio/speech",
                json=payload,
            )
            
            response.raise_for_status()
            audio_bytes = response.content
            
            duration = time.time() - start_time
            logger.info(f"Speech synthesized ({duration:.2f}s): {len(audio_bytes)} bytes")
            
            return audio_bytes
            
        except Exception as e:
            logger.error(f"Speech synthesis failed: {e}")
            return None
    
    def play_audio(self, mp3_bytes: bytes) -> None:
        """
        Play MP3 audio bytes.
        
        Args:
            mp3_bytes: MP3 audio bytes from TTS API
        """
        logger.info("Playing audio response...")
        
        try:
            # Use pydub for MP3 decoding if available
            try:
                from pydub import AudioSegment
                from pydub.playback import _play_with_simpleaudio
                
                audio = AudioSegment.from_mp3(io.BytesIO(mp3_bytes))
                
                # Export to WAV and play
                wav_buffer = io.BytesIO()
                audio.export(wav_buffer, format="wav")
                wav_buffer.seek(0)
                
                # Use simpleaudio for playback
                import simpleaudio as sa
                wave_obj = sa.WaveObject.from_wave_file(wav_buffer)
                play_obj = wave_obj.play()
                play_obj.wait_done()
                
                logger.info("Audio playback complete")
                
            except ImportError:
                # Fallback: save to temp file and use system player
                logger.warning("pydub/simpleaudio not available, using temp file")
                
                with tempfile.NamedTemporaryFile(suffix=".mp3", delete=False) as f:
                    f.write(mp3_bytes)
                    temp_path = f.name
                
                # Try mpv or ffplay
                import subprocess
                try:
                    subprocess.run(
                        ["mpv", "--no-video", "--really-quiet", temp_path],
                        check=True
                    )
                except (FileNotFoundError, subprocess.CalledProcessError):
                    try:
                        subprocess.run(
                            ["ffplay", "-nodisp", "-autoexit", "-loglevel", "quiet", temp_path],
                            check=True
                        )
                    except Exception as e:
                        logger.error(f"No audio player available: {e}")
                
                # Cleanup
                import os
                os.unlink(temp_path)
                
        except Exception as e:
            logger.error(f"Audio playback failed: {e}")
            import traceback
            traceback.print_exc()
    
    async def process_turn(
        self,
        on_recording_start: Optional[Callable] = None,
        on_speech_detected: Optional[Callable] = None,
        on_transcription_complete: Optional[Callable[[str], None]] = None,
        on_response_ready: Optional[Callable[[str], None]] = None,
    ) -> Dict[str, Any]:
        """
        Process one complete turn: Record → STT → LLM → TTS → Playback.
        
        Args:
            on_recording_start: Callback when recording starts
            on_speech_detected: Callback when speech detected
            on_transcription_complete: Callback with transcription text
            on_response_ready: Callback with response text
            
        Returns:
            Dict with turn results:
            {
                "success": bool,
                "user_transcript": str,
                "assistant_response": str,
                "timings": {...}
            }
        """
        if not self.is_active:
            logger.warning("Session not active, call start_session() first")
            return {"success": False, "error": "Session not active"}
        
        timings = {}
        turn_start = time.time()
        
        # Step 1: Record audio
        logger.info("Step 1: Recording audio...")
        record_start = time.time()
        wav_bytes = await self.recorder.record_until_silence(
            on_recording_start=on_recording_start,
            on_speech_detected=on_speech_detected,
        )
        timings["recording"] = time.time() - record_start
        
        if wav_bytes is None:
            return {"success": False, "error": "Recording failed"}
        
        # Step 2: Transcribe audio
        logger.info("Step 2: Transcribing audio...")
        stt_start = time.time()
        user_transcript = await self.transcribe_audio(wav_bytes)
        timings["stt"] = time.time() - stt_start
        
        if user_transcript is None or user_transcript.strip() == "":
            return {"success": False, "error": "Transcription failed or empty"}
        
        if on_transcription_complete:
            on_transcription_complete(user_transcript)
        
        # Step 3: Generate response
        logger.info("Step 3: Generating response...")
        llm_start = time.time()
        assistant_response = await self.generate_response(user_transcript)
        timings["llm"] = time.time() - llm_start
        
        if assistant_response is None:
            return {
                "success": False,
                "error": "Response generation failed",
                "user_transcript": user_transcript,
            }
        
        if on_response_ready:
            on_response_ready(assistant_response)
        
        # Step 4: Synthesize speech
        logger.info("Step 4: Synthesizing speech...")
        tts_start = time.time()
        audio_bytes = await self.synthesize_speech(assistant_response)
        timings["tts"] = time.time() - tts_start
        
        if audio_bytes is None:
            return {
                "success": False,
                "error": "Speech synthesis failed",
                "user_transcript": user_transcript,
                "assistant_response": assistant_response,
            }
        
        # Step 5: Play audio
        logger.info("Step 5: Playing audio...")
        playback_start = time.time()
        self.play_audio(audio_bytes)
        timings["playback"] = time.time() - playback_start
        
        timings["total"] = time.time() - turn_start
        
        logger.info(f"Turn complete: {timings['total']:.2f}s total")
        logger.info(f"  Recording: {timings['recording']:.2f}s")
        logger.info(f"  STT: {timings['stt']:.2f}s")
        logger.info(f"  LLM: {timings['llm']:.2f}s")
        logger.info(f"  TTS: {timings['tts']:.2f}s")
        logger.info(f"  Playback: {timings['playback']:.2f}s")
        
        return {
            "success": True,
            "user_transcript": user_transcript,
            "assistant_response": assistant_response,
            "timings": timings,
        }
    
    async def close(self):
        """Close HTTP client and cleanup resources."""
        await self.http_client.aclose()
        logger.info("RestSpeechClient closed")


if __name__ == "__main__":
    # Test REST speech client
    import os
    from dotenv import load_dotenv
    
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s [%(levelname)s] %(name)s: %(message)s"
    )
    
    # Load API key
    load_dotenv(os.path.expanduser("~/.r2d2/.env"))
    api_key = os.getenv("OPENAI_API_KEY")
    
    if not api_key:
        print("✗ OPENAI_API_KEY not found in ~/.r2d2/.env")
        exit(1)
    
    async def test_client():
        # Find audio device
        p = pyaudio.PyAudio()
        print("\nAvailable input devices:")
        for i in range(p.get_device_count()):
            info = p.get_device_info_by_index(i)
            if info.get('maxInputChannels', 0) > 0:
                print(f"  [{i}] {info['name']}")
        
        # Use device 0 or specify
        device_index = 0
        
        p.terminate()
        
        # Create client
        client = RestSpeechClient(
            api_key=api_key,
            instructions="You are R2-D2, the astromech droid. Be helpful but speak in a distinctive, slightly robotic way. Keep responses concise but thoughtful.",
            model="gpt-4o",  # Use gpt-4o for testing (o1-preview is expensive)
            tts_voice="nova",
            device_index=device_index,
            silence_threshold=500.0,
            silence_duration=1.5,
        )
        
        try:
            # Start session
            session_id = client.start_session()
            print(f"\n✓ Session started: {session_id}")
            print("\nSpeak now! (Recording will stop after 1.5s of silence)")
            
            # Process one turn
            result = await client.process_turn()
            
            print("\n" + "=" * 60)
            if result["success"]:
                print(f"User: {result['user_transcript']}")
                print(f"Assistant: {result['assistant_response']}")
                print(f"\nTimings: {result['timings']}")
            else:
                print(f"Error: {result.get('error')}")
            
            # End session
            client.end_session()
            
        finally:
            await client.close()
    
    asyncio.run(test_client())

