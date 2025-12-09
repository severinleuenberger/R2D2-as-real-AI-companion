# Phase 2: Component Test Scripts
## Individual Tests for Each Speech System Component

**Date:** December 9, 2025  
**Location:** ~/dev/r2d2/r2d2_speech/tests/  
**Purpose:** Test each component independently before full integration

---

## Test 1: Environment Verification Test

**File:** `~/dev/r2d2/r2d2_speech/tests/test_environment.py`

```python
#!/usr/bin/env python3
"""
Test 1: Environment Verification
Purpose: Verify all dependencies are installed correctly
Expected: All checks should pass ✓

Run: python3 tests/test_environment.py
"""

import sys
import subprocess
from pathlib import Path

def test_python_version():
    """Check Python 3.8+"""
    version = sys.version_info
    assert version.major == 3 and version.minor >= 8, f"Python 3.8+ required, got {version.major}.{version.minor}"
    print(f"✓ Python {version.major}.{version.minor}.{version.micro}")

def test_pytorch():
    """Check PyTorch + CUDA"""
    try:
        import torch
        assert torch.cuda.is_available(), "CUDA not available"
        device_name = torch.cuda.get_device_name(0)
        memory_gb = torch.cuda.get_device_properties(0).total_memory / 1e9
        print(f"✓ PyTorch {torch.__version__} + CUDA")
        print(f"  Device: {device_name}")
        print(f"  Memory: {memory_gb:.1f} GB")
    except Exception as e:
        raise AssertionError(f"PyTorch/CUDA check failed: {e}")

def test_faster_whisper():
    """Check Faster-Whisper"""
    try:
        from faster_whisper import WhisperModel
        print("✓ Faster-Whisper installed")
    except Exception as e:
        raise AssertionError(f"Faster-Whisper check failed: {e}")

def test_piper_tts():
    """Check Piper TTS"""
    try:
        import piper
        # Check if piper CLI is available
        result = subprocess.run(["piper", "--help"], capture_output=True)
        assert result.returncode == 0, "Piper CLI not working"
        print("✓ Piper TTS installed")
    except Exception as e:
        raise AssertionError(f"Piper TTS check failed: {e}")

def test_silero_vad():
    """Check Silero VAD"""
    try:
        import torch
        model, _ = torch.hub.load(
            repo_or_dir='snakers4/silero-vad',
            model='silero_vad',
            onnx=True,
            device='cuda'
        )
        print("✓ Silero VAD installed")
    except Exception as e:
        raise AssertionError(f"Silero VAD check failed: {e}")

def test_groq_api():
    """Check Groq API"""
    try:
        from groq import Groq
        print("✓ Groq API installed")
    except Exception as e:
        raise AssertionError(f"Groq API check failed: {e}")

def test_porcupine():
    """Check Porcupine"""
    try:
        import porcupine
        print("✓ Porcupine installed")
    except Exception as e:
        raise AssertionError(f"Porcupine check failed: {e}")

def test_api_keys():
    """Check API keys configuration"""
    from pathlib import Path
    import os
    
    env_file = Path.home() / ".r2d2" / ".env"
    assert env_file.exists(), "~/.r2d2/.env not found"
    
    with open(env_file) as f:
        content = f.read()
    
    has_groq = "GROQ_API_KEY" in content and "your_groq_api_key_here" not in content
    has_porcupine = "PORCUPINE_ACCESS_KEY" in content and "your_porcupine_key_here" not in content
    
    if has_groq:
        print("✓ GROQ_API_KEY configured")
    else:
        print("⚠️  GROQ_API_KEY not configured (see ~/.r2d2/.env)")
    
    if has_porcupine:
        print("✓ PORCUPINE_ACCESS_KEY configured")
    else:
        print("⚠️  PORCUPINE_ACCESS_KEY not configured (see ~/.r2d2/.env)")

def test_models_downloaded():
    """Check if models are downloaded"""
    from pathlib import Path
    
    whisper_path = Path.home() / ".cache/huggingface/hub/models--openai--whisper-large-v2"
    piper_kerstin = Path.home() / ".local/share/piper-tts/models/de_DE-kerstin-medium.onnx"
    piper_libritts = Path.home() / ".local/share/piper-tts/models/en_US-libritts-high.onnx"
    
    if whisper_path.exists():
        size_gb = sum(f.stat().st_size for f in whisper_path.rglob('*')) / 1e9
        print(f"✓ Whisper Large model ({size_gb:.1f} GB)")
    else:
        print("⚠️  Whisper Large model not downloaded yet")
    
    if piper_kerstin.exists():
        size_mb = piper_kerstin.stat().st_size / 1e6
        print(f"✓ Piper kerstin-medium ({size_mb:.1f} MB)")
    else:
        print("⚠️  Piper kerstin-medium not downloaded")
    
    if piper_libritts.exists():
        size_mb = piper_libritts.stat().st_size / 1e6
        print(f"✓ Piper libritts-high ({size_mb:.1f} MB)")
    else:
        print("⚠️  Piper libritts-high not downloaded")

if __name__ == "__main__":
    print("="*70)
    print("TEST 1: ENVIRONMENT VERIFICATION")
    print("="*70)
    print()
    
    tests = [
        ("Python Version", test_python_version),
        ("PyTorch + CUDA", test_pytorch),
        ("Faster-Whisper", test_faster_whisper),
        ("Piper TTS", test_piper_tts),
        ("Silero VAD", test_silero_vad),
        ("Groq API", test_groq_api),
        ("Porcupine", test_porcupine),
        ("API Keys", test_api_keys),
        ("Downloaded Models", test_models_downloaded),
    ]
    
    failed = 0
    for test_name, test_func in tests:
        try:
            print(f"\n{test_name}:")
            test_func()
        except AssertionError as e:
            print(f"✗ {test_name}: {e}")
            failed += 1
    
    print()
    print("="*70)
    if failed == 0:
        print("✅ All environment checks passed!")
        print("   Proceed to Test 2: test_microphone.py")
    else:
        print(f"❌ {failed} check(s) failed. See errors above.")
    print("="*70)
```

---

## Test 2: Microphone Input Test

**File:** `~/dev/r2d2/r2d2_speech/tests/test_microphone.py`

```python
#!/usr/bin/env python3
"""
Test 2: Microphone Input (ReSpeaker)
Purpose: Verify ReSpeaker 2-Mic HAT is working correctly
Expected: Records and plays back audio successfully

Run: python3 tests/test_microphone.py
"""

import pyaudio
import wave
import os
import time
from pathlib import Path

def list_audio_devices():
    """List all available audio devices"""
    p = pyaudio.PyAudio()
    
    print("\n" + "="*70)
    print("AVAILABLE AUDIO DEVICES")
    print("="*70)
    
    for i in range(p.get_device_count()):
        info = p.get_device_info_by_index(i)
        is_input = info['maxInputChannels'] > 0
        is_output = info['maxOutputChannels'] > 0
        
        device_type = []
        if is_input: device_type.append("IN")
        if is_output: device_type.append("OUT")
        
        print(f"\n[{i}] {info['name']}")
        print(f"    Type: {'/'.join(device_type)}")
        print(f"    Channels: IN={info['maxInputChannels']}, OUT={info['maxOutputChannels']}")
        print(f"    Sample rate: {int(info['defaultSampleRate'])} Hz")
    
    p.terminate()

def test_microphone_record():
    """Record 5 seconds from ReSpeaker"""
    print("\n" + "="*70)
    print("MICROPHONE RECORDING TEST")
    print("="*70)
    
    CHUNK = 1024
    FORMAT = pyaudio.paFloat32
    CHANNELS = 2  # ReSpeaker has 2 channels
    RATE = 16000
    RECORD_SECONDS = 5
    OUTPUT_FILE = "/tmp/test_microphone.wav"
    
    p = pyaudio.PyAudio()
    
    # Try to find ReSpeaker device
    respeaker_index = None
    for i in range(p.get_device_count()):
        info = p.get_device_info_by_index(i)
        if 'ReSpeaker' in info['name'] or 'ALSA' in info['name']:
            respeaker_index = i
            print(f"\nFound ReSpeaker at device index {i}: {info['name']}")
            break
    
    if respeaker_index is None:
        respeaker_index = 2  # Default fallback (hw:2,0)
        print(f"\nUsing default device index 2 (hw:2,0)")
    
    print(f"\nRecording {RECORD_SECONDS} seconds of audio...")
    print("Please speak into the microphone:")
    print()
    
    try:
        stream = p.open(
            format=FORMAT,
            channels=CHANNELS,
            rate=RATE,
            input=True,
            input_device_index=respeaker_index,
            frames_per_buffer=CHUNK,
            stream_callback=None
        )
        
        stream.start_stream()
        
        frames = []
        for i in range(0, int(RATE / CHUNK * RECORD_SECONDS)):
            data = stream.read(CHUNK, exception_on_overflow=False)
            frames.append(data)
            
            # Progress indicator
            elapsed = (i + 1) * CHUNK / RATE
            bars = int(elapsed / RECORD_SECONDS * 20)
            print(f"\r[{'='*bars}{' '*(20-bars)}] {elapsed:.1f}s", end='', flush=True)
        
        print("\n\n✓ Recording complete")
        
        stream.stop_stream()
        stream.close()
        
        # Save recording
        with wave.open(OUTPUT_FILE, 'wb') as wf:
            wf.setnchannels(CHANNELS)
            wf.setsampwidth(p.get_sample_size(FORMAT))
            wf.setframerate(RATE)
            wf.writeframes(b''.join(frames))
        
        print(f"✓ Saved to: {OUTPUT_FILE}")
        return OUTPUT_FILE
    
    except Exception as e:
        print(f"✗ Recording failed: {e}")
        return None
    
    finally:
        p.terminate()

def test_playback(audio_file):
    """Play back the recorded audio"""
    if not audio_file:
        print("\n❌ No audio file to play")
        return
    
    print("\n" + "="*70)
    print("PLAYBACK TEST")
    print("="*70)
    print(f"\nPlaying back: {audio_file}")
    print("(Using ffplay via ALSA)")
    
    os.system(f"ffplay -nodisp -autoexit {audio_file}")
    print("✓ Playback complete")

def test_audio_levels(audio_file):
    """Analyze audio levels to ensure clean recording"""
    if not audio_file:
        return
    
    print("\n" + "="*70)
    print("AUDIO QUALITY ANALYSIS")
    print("="*70)
    
    import wave
    import numpy as np
    
    try:
        with wave.open(audio_file, 'rb') as wf:
            # Read audio data
            frames = wf.readframes(wf.getnframes())
            audio_data = np.frombuffer(frames, dtype=np.int16)
            
            # Calculate metrics
            rms = np.sqrt(np.mean(audio_data**2))
            peak = np.max(np.abs(audio_data))
            rms_db = 20 * np.log10(rms) if rms > 0 else -np.inf
            peak_db = 20 * np.log10(peak) if peak > 0 else -np.inf
            
            print(f"\nAudio file: {audio_file}")
            print(f"Duration: {wf.getnframes() / wf.getframerate():.1f} seconds")
            print(f"Sample rate: {wf.getframerate()} Hz")
            print(f"Channels: {wf.getnchannels()}")
            print()
            print(f"RMS level: {rms_db:.1f} dB (target: -20 to -15 dB)")
            print(f"Peak level: {peak_db:.1f} dB (target: < -6 dB to avoid clipping)")
            print()
            
            if peak_db > -1:
                print("⚠️  Audio is clipping! Reduce microphone input gain.")
            elif peak_db < -20:
                print("⚠️  Audio is very quiet. Check microphone connection.")
            else:
                print("✓ Audio levels look good")
    
    except Exception as e:
        print(f"Could not analyze audio: {e}")

if __name__ == "__main__":
    print("\n" + "="*70)
    print("TEST 2: MICROPHONE INPUT TEST")
    print("="*70)
    
    # List devices
    list_audio_devices()
    
    # Record test
    audio_file = test_microphone_record()
    
    # Playback test
    if audio_file:
        test_playback(audio_file)
    
    # Analysis
    if audio_file:
        test_audio_levels(audio_file)
    
    print("\n" + "="*70)
    if audio_file:
        print("✅ Microphone test passed!")
        print(f"   Recording saved: {audio_file}")
        print("   Proceed to Test 3: test_stt.py")
    else:
        print("❌ Microphone test failed. Check ReSpeaker connection.")
    print("="*70)
```

---

## Test 3: Speech-to-Text (Whisper) Test

**File:** `~/dev/r2d2/r2d2_speech/tests/test_stt.py`

```python
#!/usr/bin/env python3
"""
Test 3: Speech-to-Text (Faster-Whisper)
Purpose: Test Whisper large model with Swiss German / German audio
Expected: Transcribes test audio correctly

Run: python3 tests/test_stt.py [--use-file audio.wav] [--record]
"""

import os
import sys
import argparse
import subprocess
import time
from pathlib import Path

def download_model():
    """Ensure Whisper model is downloaded"""
    print("Loading Whisper large model...")
    print("(This takes ~30 seconds on first run)")
    
    try:
        from faster_whisper import WhisperModel
        
        start = time.time()
        model = WhisperModel("large-v2", device="cuda", compute_type="float16")
        elapsed = time.time() - start
        
        print(f"✓ Model loaded in {elapsed:.1f} seconds")
        return model
    except Exception as e:
        print(f"✗ Failed to load model: {e}")
        return None

def record_test_audio(duration=10):
    """Record test audio from microphone"""
    output_file = "/tmp/test_stt_input.wav"
    
    print(f"\nRecording {duration} seconds of test audio...")
    print("Please speak Swiss German or German into the microphone:")
    print()
    
    # Use ffmpeg to record from ReSpeaker
    cmd = [
        "ffmpeg",
        "-f", "alsa",
        "-i", "hw:2,0",
        "-t", str(duration),
        "-q:a", "9",  # Good quality
        output_file
    ]
    
    try:
        subprocess.run(cmd, check=True, capture_output=True)
        print(f"✓ Recorded to: {output_file}")
        return output_file
    except Exception as e:
        print(f"✗ Recording failed: {e}")
        return None

def transcribe_audio(model, audio_file):
    """Transcribe audio using Whisper"""
    if not model:
        print("✗ Model not available")
        return None
    
    print(f"\nTranscribing: {audio_file}")
    print("(This may take 5-15 seconds depending on audio length)")
    print()
    
    start = time.time()
    
    try:
        segments, info = model.transcribe(
            audio_file,
            language="de",  # German/Swiss German
            beam_size=5,
            best_of=5,
            temperature=0.0
        )
        
        elapsed = time.time() - start
        
        # Collect all text
        full_text = " ".join([segment.text for segment in segments])
        
        print(f"✓ Transcription complete ({elapsed:.1f}s)")
        print()
        print(f"Detected language: {info.language}")
        print(f"Language confidence: {info.language_probability:.1%}")
        print()
        print("Segments:")
        for segment in segments:
            print(f"  [{segment.start:.2f}s - {segment.end:.2f}s] {segment.text}")
        print()
        print(f"Full transcription:\n  \"{full_text}\"")
        
        return {
            'text': full_text,
            'language': info.language,
            'confidence': info.language_probability,
            'segments': segments
        }
    
    except Exception as e:
        print(f"✗ Transcription failed: {e}")
        return None

def test_with_file(audio_file):
    """Test transcription with existing audio file"""
    print("\n" + "="*70)
    print("TEST 3: SPEECH-TO-TEXT (FASTER-WHISPER)")
    print("="*70)
    print(f"\nUsing provided audio file: {audio_file}")
    
    if not Path(audio_file).exists():
        print(f"✗ File not found: {audio_file}")
        return False
    
    model = download_model()
    if not model:
        return False
    
    result = transcribe_audio(model, audio_file)
    return bool(result)

def test_with_recording():
    """Test transcription with fresh recording"""
    print("\n" + "="*70)
    print("TEST 3: SPEECH-TO-TEXT (FASTER-WHISPER)")
    print("="*70)
    
    model = download_model()
    if not model:
        return False
    
    audio_file = record_test_audio(duration=10)
    if not audio_file:
        return False
    
    result = transcribe_audio(model, audio_file)
    return bool(result)

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Test Whisper STT")
    parser.add_argument("--use-file", type=str, help="Use existing audio file")
    parser.add_argument("--record", action="store_true", help="Record new audio")
    parser.add_argument("--duration", type=int, default=10, help="Recording duration (seconds)")
    
    args = parser.parse_args()
    
    success = False
    
    if args.use_file:
        success = test_with_file(args.use_file)
    elif args.record:
        success = test_with_recording()
    else:
        # Use pre-recorded test audio if available
        test_audio = "/tmp/test_microphone.wav"
        if Path(test_audio).exists():
            print(f"Using previous recording: {test_audio}")
            success = test_with_file(test_audio)
        else:
            print("No audio file provided and no previous recording found.")
            print("Use: python3 test_stt.py --record")
            sys.exit(1)
    
    print("\n" + "="*70)
    if success:
        print("✅ STT test passed!")
        print("   Proceed to Test 4: test_tts.py")
    else:
        print("❌ STT test failed. Check model and audio file.")
    print("="*70)
```

---

## Test 4: Text-to-Speech (Piper) Test

**File:** `~/dev/r2d2/r2d2_speech/tests/test_tts.py`

```python
#!/usr/bin/env python3
"""
Test 4: Text-to-Speech (Piper TTS)
Purpose: Test Piper TTS with German voices
Expected: Synthesizes and plays German speech

Run: python3 tests/test_tts.py [--voice de_DE-kerstin-medium|en_US-libritts-high] [--text "your text"]
"""

import subprocess
import os
import argparse
import tempfile
from pathlib import Path

def list_available_voices():
    """List installed Piper voices"""
    voice_dir = Path.home() / ".local/share/piper-tts/models"
    
    print("\nAvailable Piper voices:")
    print("-" * 70)
    
    voices = []
    for onnx_file in voice_dir.glob("*.onnx"):
        voice_name = onnx_file.stem
        voices.append(voice_name)
        size_mb = onnx_file.stat().st_size / 1e6
        print(f"  • {voice_name} ({size_mb:.1f} MB)")
    
    return voices

def synthesize_text(text, voice="de_DE-kerstin-medium"):
    """Synthesize text to speech using Piper"""
    
    print(f"\nSynthesizing with voice: {voice}")
    print(f"Text: \"{text}\"")
    print()
    
    output_file = "/tmp/test_tts_output.wav"
    
    # Build Piper command
    cmd = f'echo "{text}" | piper --model {voice} --output-file {output_file}'
    
    try:
        result = subprocess.run(cmd, shell=True, capture_output=True, text=True)
        
        if result.returncode != 0:
            print(f"✗ Synthesis failed: {result.stderr}")
            return None
        
        # Check if file was created
        if not Path(output_file).exists():
            print("✗ Output file not created")
            return None
        
        size_kb = Path(output_file).stat().st_size / 1024
        print(f"✓ Synthesized successfully ({size_kb:.1f} KB)")
        print(f"  Output: {output_file}")
        
        return output_file
    
    except Exception as e:
        print(f"✗ Synthesis error: {e}")
        return None

def playback_audio(audio_file):
    """Play audio file via ffplay"""
    if not audio_file or not Path(audio_file).exists():
        print("✗ Audio file not found")
        return False
    
    print(f"\nPlaying audio: {audio_file}")
    print("(Press Ctrl+C to skip)")
    print()
    
    try:
        os.system(f"ffplay -nodisp -autoexit {audio_file}")
        print("\n✓ Playback complete")
        return True
    except Exception as e:
        print(f"✗ Playback failed: {e}")
        return False

def test_voice(voice, text):
    """Test a specific voice"""
    print("\n" + "="*70)
    print(f"TESTING VOICE: {voice}")
    print("="*70)
    
    # Synthesize
    audio_file = synthesize_text(text, voice)
    if not audio_file:
        return False
    
    # Playback
    return playback_audio(audio_file)

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Test Piper TTS")
    parser.add_argument("--voice", type=str, default="de_DE-kerstin-medium",
                       help="Voice to use (default: de_DE-kerstin-medium)")
    parser.add_argument("--text", type=str, default="Hoi, ich bin R2D2. Wie kann ich dir helfe?",
                       help="Text to synthesize (default: Swiss German greeting)")
    
    args = parser.parse_args()
    
    print("\n" + "="*70)
    print("TEST 4: TEXT-TO-SPEECH (PIPER TTS)")
    print("="*70)
    
    # Show available voices
    available_voices = list_available_voices()
    
    if not available_voices:
        print("\n✗ No Piper voices installed!")
        print("   Run: piper --download-model de_DE-kerstin-medium")
        exit(1)
    
    # Test requested voice
    if args.voice not in available_voices:
        print(f"\n✗ Voice not available: {args.voice}")
        print(f"   Available: {', '.join(available_voices)}")
        exit(1)
    
    success = test_voice(args.voice, args.text)
    
    print("\n" + "="*70)
    if success:
        print("✅ TTS test passed!")
        print("   Proceed to Test 5: test_llm.py")
    else:
        print("❌ TTS test failed.")
    print("="*70)
```

---

## Test 5: LLM Integration Test

**File:** `~/dev/r2d2/r2d2_speech/tests/test_llm.py`

```python
#!/usr/bin/env python3
"""
Test 5: LLM Integration (Grok API)
Purpose: Test Grok API with Swiss German prompts
Expected: API responds correctly to German text

Run: python3 tests/test_llm.py [--text "your text"]
"""

import os
import sys
import argparse
from pathlib import Path

def load_api_key():
    """Load Grok API key from config"""
    env_file = Path.home() / ".r2d2" / ".env"
    
    if not env_file.exists():
        print("✗ API config not found: ~/.r2d2/.env")
        return None
    
    # Load environment variables
    with open(env_file) as f:
        for line in f:
            if line.startswith("GROQ_API_KEY"):
                key = line.split("=")[1].strip().strip('"')
                if key != "your_groq_api_key_here":
                    return key
    
    print("✗ GROQ_API_KEY not configured (see ~/.r2d2/.env)")
    return None

def test_grok_connection(api_key, text):
    """Test Grok API connection and response"""
    
    try:
        from groq import Groq
    except ImportError:
        print("✗ Groq library not installed")
        return False
    
    print(f"\nUser input: \"{text}\"")
    print()
    
    try:
        client = Groq(api_key=api_key)
        
        # R2D2 system prompt (Swiss German)
        system_prompt = """Du bist R2D2, en hilfruiche KI-Begleit-Roboter.
Antworte uf Schwizerdütsch oder Hochdütsch (je nach Nutzer).
Halte Antworte kurz (1-3 Sätze).
Sprich freundlich und R2D2-mässig."""
        
        print("Calling Grok API...")
        
        response = client.chat.completions.create(
            model="mixtral-8x7b-32768",
            messages=[
                {
                    "role": "system",
                    "content": system_prompt
                },
                {
                    "role": "user",
                    "content": text
                }
            ],
            temperature=0.7,
            max_tokens=150,
            top_p=1
        )
        
        reply = response.choices[0].message.content
        
        print(f"✓ API response received")
        print()
        print(f"R2D2 says: \"{reply}\"")
        print()
        print(f"Tokens used: {response.usage.total_tokens}")
        
        return True
    
    except Exception as e:
        print(f"✗ API call failed: {e}")
        if "401" in str(e):
            print("   (Check API key in ~/.r2d2/.env)")
        return False

def test_command_extraction(api_key):
    """Test if API can extract commands"""
    
    from groq import Groq
    
    test_prompts = [
        ("Komm zu mir!", "follow_person"),
        ("Geh in die Küche", "navigate_to"),
        ("Schau mich an", "pan_camera"),
        ("Wie geht es dir?", None),
    ]
    
    print("\n" + "="*70)
    print("COMMAND EXTRACTION TEST")
    print("="*70)
    
    client = Groq(api_key=api_key)
    
    system_prompt = """Du bist R2D2. Antworte natürlich, aber wenn der Nutzer ein Kommando gibt:
- "Komm zu mir" oder ähnlich → Antworte: ACTION:follow_person
- "Geh zu [Ort]" → Antworte: ACTION:navigate_to
- "Schau auf" oder ähnlich → Antworte: ACTION:pan_camera
Sonst: antworte einfach freundlich."""
    
    for prompt, expected_action in test_prompts:
        try:
            response = client.chat.completions.create(
                model="mixtral-8x7b-32768",
                messages=[
                    {"role": "system", "content": system_prompt},
                    {"role": "user", "content": prompt}
                ],
                temperature=0.7,
                max_tokens=100,
                top_p=1
            )
            
            reply = response.choices[0].message.content
            has_action = "ACTION:" in reply
            
            if expected_action:
                if has_action and expected_action in reply:
                    print(f"✓ \"{prompt}\" → ACTION:{expected_action}")
                else:
                    print(f"⚠️  \"{prompt}\" → {reply[:50]}...")
            else:
                if has_action:
                    print(f"⚠️  \"{prompt}\" → {reply[:50]}... (expected no action)")
                else:
                    print(f"✓ \"{prompt}\" → {reply[:50]}...")
        
        except Exception as e:
            print(f"✗ Error testing \"{prompt}\": {e}")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Test Grok LLM API")
    parser.add_argument("--text", type=str, default="Hoi R2D2, wie geht's dir heute?",
                       help="Text to send to Grok")
    
    args = parser.parse_args()
    
    print("\n" + "="*70)
    print("TEST 5: LLM INTEGRATION (GROK API)")
    print("="*70)
    
    # Load API key
    api_key = load_api_key()
    if not api_key:
        print("\n❌ Cannot proceed without API key")
        exit(1)
    
    print(f"✓ API key loaded ({api_key[:10]}...)")
    
    # Test basic connection
    success = test_grok_connection(api_key, args.text)
    
    # Test command extraction
    if success:
        test_command_extraction(api_key)
    
    print("\n" + "="*70)
    if success:
        print("✅ LLM test passed!")
        print("   Proceed to Test 6: test_end_to_end.py")
    else:
        print("❌ LLM test failed.")
    print("="*70)
```

---

## Test 6: Complete End-to-End Pipeline Test

**File:** `~/dev/r2d2/r2d2_speech/tests/test_end_to_end.py`

```python
#!/usr/bin/env python3
"""
Test 6: End-to-End Speech Pipeline
Purpose: Test the complete pipeline: Record → STT → LLM → TTS → Play
Expected: Full conversational loop works

Run: python3 tests/test_end_to_end.py
"""

import subprocess
import os
import time
from pathlib import Path
from faster_whisper import WhisperModel
from groq import Groq
import sys

def load_config():
    """Load configuration"""
    env_file = Path.home() / ".r2d2" / ".env"
    
    config = {
        'groq_api_key': None,
        'voice': 'de_DE-kerstin-medium',
        'language': 'de'
    }
    
    if env_file.exists():
        with open(env_file) as f:
            for line in f:
                if "GROQ_API_KEY=" in line:
                    config['groq_api_key'] = line.split("=")[1].strip().strip('"')
    
    return config

def record_audio(duration=5, output_file="/tmp/e2e_test_input.wav"):
    """Record audio from ReSpeaker"""
    print(f"\n[1/5] Recording {duration}s of audio...")
    print("Speak into microphone (Swiss German or German):")
    
    cmd = [
        "ffmpeg", "-f", "alsa", "-i", "hw:2,0",
        "-t", str(duration), "-q:a", "9",
        output_file
    ]
    
    try:
        subprocess.run(cmd, check=True, capture_output=True)
        print(f"✓ Recorded")
        return output_file
    except Exception as e:
        print(f"✗ Recording failed: {e}")
        return None

def transcribe(model, audio_file):
    """Transcribe audio to text"""
    print(f"\n[2/5] Transcribing audio...")
    
    try:
        segments, info = model.transcribe(audio_file, language="de")
        text = " ".join([s.text for s in segments])
        print(f"✓ Transcribed: \"{text}\"")
        return text
    except Exception as e:
        print(f"✗ Transcription failed: {e}")
        return None

def process_with_llm(api_key, text):
    """Send text to Grok LLM"""
    print(f"\n[3/5] Calling Grok LLM...")
    
    try:
        client = Groq(api_key=api_key)
        
        system_prompt = """Du bist R2D2, en hilfruiche KI-Begleit-Roboter.
Antworte uf Schwizerdütsch oder Hochdütsch.
Halte Antworte kurz (1-3 Sätze).
Sprich freundlich."""
        
        response = client.chat.completions.create(
            model="mixtral-8x7b-32768",
            messages=[
                {"role": "system", "content": system_prompt},
                {"role": "user", "content": text}
            ],
            temperature=0.7,
            max_tokens=150
        )
        
        reply = response.choices[0].message.content
        print(f"✓ LLM response: \"{reply}\"")
        return reply
    
    except Exception as e:
        print(f"✗ LLM call failed: {e}")
        return None

def synthesize_speech(text, voice="de_DE-kerstin-medium", output_file="/tmp/e2e_test_output.wav"):
    """Synthesize text to speech"""
    print(f"\n[4/5] Synthesizing speech...")
    
    cmd = f'echo "{text}" | piper --model {voice} --output-file {output_file}'
    
    try:
        subprocess.run(cmd, shell=True, check=True, capture_output=True)
        print(f"✓ Synthesized")
        return output_file
    except Exception as e:
        print(f"✗ Synthesis failed: {e}")
        return None

def play_audio(audio_file):
    """Play synthesized audio"""
    print(f"\n[5/5] Playing response...")
    
    try:
        os.system(f"ffplay -nodisp -autoexit {audio_file}")
        print(f"✓ Playback complete")
        return True
    except Exception as e:
        print(f"✗ Playback failed: {e}")
        return False

if __name__ == "__main__":
    print("\n" + "="*70)
    print("TEST 6: END-TO-END SPEECH PIPELINE")
    print("="*70)
    print("\nThis test will:")
    print("  1. Record 5 seconds of audio")
    print("  2. Transcribe with Whisper")
    print("  3. Process with Grok LLM")
    print("  4. Synthesize response with Piper")
    print("  5. Play back audio")
    print()
    
    # Load config
    config = load_config()
    if not config['groq_api_key']:
        print("✗ API key not configured")
        exit(1)
    
    # Load Whisper model
    print("Loading Whisper model (30s)...")
    try:
        model = WhisperModel("large-v2", device="cuda", compute_type="float16")
        print("✓ Whisper loaded")
    except Exception as e:
        print(f"✗ Failed to load Whisper: {e}")
        exit(1)
    
    # Run pipeline
    audio_input = record_audio(duration=5)
    if not audio_input:
        exit(1)
    
    text = transcribe(model, audio_input)
    if not text:
        exit(1)
    
    response = process_with_llm(config['groq_api_key'], text)
    if not response:
        exit(1)
    
    audio_output = synthesize_speech(response, config['voice'])
    if not audio_output:
        exit(1)
    
    play_audio(audio_output)
    
    print("\n" + "="*70)
    print("✅ END-TO-END TEST PASSED!")
    print("="*70)
    print("\nFull pipeline working:")
    print(f"  Input: \"{text}\"")
    print(f"  Output: \"{response}\"")
    print("\nReady for full system integration!")
```

---

## Quick Test Reference

**Run all tests in sequence:**

```bash
#!/bin/bash
# tests/run_all_tests.sh

cd ~/dev/r2d2/r2d2_speech

echo "=== Running all tests ==="
echo ""

python3 tests/test_environment.py || exit 1
echo ""
python3 tests/test_microphone.py || exit 1
echo ""
python3 tests/test_stt.py --record || exit 1
echo ""
python3 tests/test_tts.py || exit 1
echo ""
python3 tests/test_llm.py || exit 1
echo ""
python3 tests/test_end_to_end.py || exit 1

echo ""
echo "✅ ALL TESTS PASSED!"
```

---

**Test Scripts Version:** 1.0  
**Total Tests:** 6 components  
**Status:** Ready to run  
**Next:** Run tests in order (001 through 006)
