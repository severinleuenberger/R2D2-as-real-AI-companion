# Phase 2: Speech-to-Text → LLM → Text-to-Speech Architecture
## Complete Recommended Architecture for R2D2 Conversational System

**Date:** December 9, 2025  
**Status:** Recommended Architecture (Ready for Implementation)  
**Platform:** NVIDIA Jetson AGX Orin 64GB + ROS 2 Humble + ReSpeaker 2-Mic HAT

---

## Executive Summary: What I Recommend

Your instinct is exactly right: **minimize ROS 2 overhead for the speech pipeline, only integrate discrete commands back into ROS 2**.

### Architecture Overview (Simple & Efficient)

```
ReSpeaker 2-Mic HAT (hardware)
    ↓ (I2S/USB)
Microphone Manager (Python daemon, NOT ROS)
    ├─ Wake word detection: "Hey R2D2"
    ├─ Audio recording: ring buffer (circular buffer)
    └─ Signal: "wake word detected"
    ↓
Speech-to-Text (STT): Whisper (OpenAI, local, GPU-accelerated)
    ├─ Convert audio → text
    ├─ High accuracy, fast on Jetson GPU
    └─ Output: raw user text
    ↓
LLM Processing (Grok API initially, local Ollama fallback)
    ├─ Parse user intent
    ├─ Generate conversational response
    └─ Detect commands: "come to me", "go to X", "look at this"
    ↓
Text-to-Speech (TTS): pyttsx3 or faster local (Coqui TTS or gTTS)
    ├─ Convert response text → audio
    ├─ Stream playback (not ROS)
    └─ Output: Speaker via PAM8403
    ↓
COMMAND EXTRACTION (only discrete actions go to ROS)
    ├─ IF "come to me" → publish /r2d2/cmd/follow_person
    ├─ IF "go to living room" → publish /r2d2/cmd/navigate_to
    ├─ IF "look at this" → publish /r2d2/cmd/pan_camera
    └─ OTHERWISE: just speak response (no ROS action needed)
```

**Why This Architecture:**
- ✅ **Minimal ROS overhead:** Continuous STT/LLM/TTS runs as a single Python daemon
- ✅ **Low latency:** No ROS message serialization overhead
- ✅ **Scalability:** Easy to add multi-turn conversation state
- ✅ **Separation of concerns:** ROS only for discrete robot commands
- ✅ **Reusable:** STT/LLM/TTS pipeline can be tested standalone
- ✅ **Maintainable:** Clear boundaries between speech & robot systems

---

## Component Recommendations

### 1. MICROPHONE INPUT: ReSpeaker 2-Mic HAT

**Status:** You have this hardware ready ✅

**What It Is:**
- 2-channel microphone array (dual mics for better noise cancellation)
- Connects via I2C + I2S to Jetson
- LED ring for visual feedback (can show status: listening, processing, speaking)
- Wake word acceleration hardware (optional)
- Price: ~$40

**Setup (Phase 2 Task 1):**
```bash
# 1. Physical installation
#    - Stack HAT onto Jetson 40-pin GPIO header
#    - Verify I2C/I2S bus detection

# 2. Install ReSpeaker driver
pip install respeaker

# 3. Test microphone input
python3 -c "import pyaudio; p = pyaudio.PyAudio(); ..." 

# 4. Verify audio recording
python3 ~/dev/r2d2/test_microphone.py
```

**Hardware Connections:**
```
ReSpeaker 2-Mic HAT
├─ GPIO Header: pins 1-40 (standard 40-pin connector)
├─ I2C: pins 3 (SDA), 5 (SCL) — for LED control & button
├─ I2S: pins 12 (PCM_CLK), 35 (PCM_FS), 38 (PCM_DIN)
├─ Power: pins 2 (5V), 4 (5V), 6 (GND)
└─ Status: Jetson automatically detects via I2C
```

**Audio Device Names (ALSA):**
- Capture: `hw:2,0` (ReSpeaker USB audio input) or I2S device
- Playback: Keep using `hw:1,0` (existing PAM8403 speaker)

---

### 2. WAKE WORD DETECTION: Porcupine or Picovoice

**Recommendation:** Picovoice Porcupine (free tier allows 1 wake word)

**Why:**
- ✅ Runs locally on-device (no cloud calls for wake word)
- ✅ Optimized for ARM64 (works on Jetson)
- ✅ Ultra-low latency & CPU efficient
- ✅ Free tier includes "Ok Google", "Hey Google", or custom "Hey R2D2"
- ✅ Accuracy: ~95% at normal conversation level

**Alternative (Simpler):**
- **Snowboy** (legacy but free): Harder to set up, less maintained
- **Pocketsphinx**: CMU's lightweight, but less accurate

**Setup:**
```bash
# Install Porcupine SDK
pip install porcupine

# Create access key at: https://console.picovoice.ai/
# (free tier available)

# Test wake word detection
python3 ~/dev/r2d2/test_wake_word.py "Hey R2D2"
```

**Architecture (within your daemon):**
```python
# Pseudo-code for ReSpeaker audio manager daemon
import pyaudio
import porcupine

porcupine_handle = porcupine.create(
    keywords=['alexa', 'americano', 'hey google', 'hey siri'],
    access_key='<your_key>'
)

# Continuous listening loop
p = pyaudio.PyAudio()
stream = p.open(...)  # ReSpeaker input

while True:
    audio_frame = stream.read(512)  # Non-blocking read
    
    if porcupine_handle.process(audio_frame):
        # WAKE WORD DETECTED! 🎤
        print("Wake word detected at:", porcupine_handle.keywordIndex)
        signal_stt_to_start()  # Tell STT to begin recording
        break

stream.stop_stream()
```

---

### 3. SPEECH-TO-TEXT (STT): Whisper

**Recommendation:** OpenAI Whisper (GPU-accelerated on Jetson)

**Why:**
- ✅ State-of-the-art accuracy (99%+)
- ✅ Runs fully locally (no cloud dependency)
- ✅ GPU-optimized for Jetson (CUDA/cuDNN)
- ✅ Multilingual (can easily add support)
- ✅ Open-source & free
- ✅ Handles accents & background noise well

**Model Sizes (Choose One):**

| Model | Size | Speed (Jetson) | Accuracy | Recommended For |
|-------|------|---|----------|---|
| tiny | 39MB | 0.3s | 85% | Testing, ultra-low latency |
| base | 140MB | 0.5s | 90% | **← BEST CHOICE** |
| small | 466MB | 1.5s | 93% | Better accuracy if available |
| medium | 1.5GB | 4s | 95% | Overkill for conversational |
| large | 2.9GB | 8s | 97% | Way too slow for real-time |

**Recommendation:** Use **base** model (0.5s latency, 90% accuracy, good GPU usage)

**Installation:**
```bash
# Install Whisper
pip install openai-whisper

# Download model (happens auto on first run)
# Base model: ~500MB, downloaded once
whisper audio.wav --model base --device cuda
```

**Architecture Integration:**
```python
import whisper
import numpy as np

# Load model once at startup
model = whisper.load_model("base", device="cuda")

def transcribe_audio(audio_bytes: bytes) -> str:
    """Convert audio bytes to text"""
    audio_np = np.frombuffer(audio_bytes, dtype=np.float32)
    
    result = model.transcribe(audio_np, language="en")
    return result['text']

# Usage:
transcribed_text = transcribe_audio(audio_from_user)
print(f"User said: '{transcribed_text}'")
```

**Performance on Jetson AGX Orin:**
- Latency: 0.5-1.0 second (GPU-accelerated)
- CPU: 5-10% during transcription
- GPU: 30-50% during transcription
- Memory: ~1.5GB peak (base model)

---

### 4. LANGUAGE MODEL (LLM): Grok API (Cloud) vs Ollama (Local)

**Your Choice: Grok API for Now** ✅

This is smart because:
1. **Grok is fast and capable** (xAI's latest model)
2. **No local resource overhead** during conversational turns
3. **Easy to implement** (simple API calls)
4. **Easy to switch later** to local Ollama when needed

#### Option A: Grok API (Recommended Initially)

**Setup:**
```bash
# Get API key from: https://console.groq.com/

# Install Groq SDK
pip install groq

# Set API key
export GROQ_API_KEY="<your_key>"
```

**Usage Pattern:**
```python
from groq import Groq

client = Groq(api_key="YOUR_API_KEY")

def get_llm_response(user_text: str, context: dict = None) -> dict:
    """
    Call Grok LLM to process user input.
    Returns: {
        'response': 'text to speak',
        'command': 'command name or None'
    }
    """
    
    system_prompt = """You are R2D2, a helpful AI companion robot. 
    Keep responses brief (1-3 sentences).
    
    If user asks you to:
    - Move: respond with ACTION:follow_person or ACTION:navigate_to
    - Look at something: respond with ACTION:pan_camera
    - Otherwise: just respond conversationally
    
    Always speak in a friendly, R2D2-like personality."""
    
    messages = [
        {"role": "system", "content": system_prompt},
        {"role": "user", "content": user_text}
    ]
    
    if context and context.get('person_name'):
        messages[0]['content'] += f"\nThe person is named {context['person_name']}."
    
    response = client.chat.completions.create(
        model="mixtral-8x7b-32768",  # Fast, capable
        messages=messages,
        temperature=0.7,
        max_tokens=150,
        top_p=1
    )
    
    response_text = response.choices[0].message.content
    
    # Parse commands
    if "ACTION:" in response_text:
        parts = response_text.split("ACTION:")
        response_text = parts[0].strip()
        command = parts[1].split()[0] if len(parts) > 1 else None
        return {'response': response_text, 'command': command}
    
    return {'response': response_text, 'command': None}
```

**Latency on Jetson:**
- API call: 200-500ms (network dependent)
- Processing: 50-200ms (model inference)
- **Total:** 0.25-0.7 seconds (acceptable for conversation)

**Cost:** Free tier allows ~100 requests/day; paid plans are cheap

---

#### Option B: Ollama (Local, For Future)

**When to Switch:** When you want completely offline operation

**Recommended Model:** Llama 2 7B (quantized)

```bash
# Install Ollama (if switching later)
curl https://ollama.ai/install.sh | sh

# Pull model
ollama pull llama2:7b-chat-q4_0  # 4GB, ~1s response time

# Start server
ollama serve &

# Python client
import requests

def get_ollama_response(user_text: str) -> str:
    response = requests.post('http://localhost:11434/api/generate', 
        json={
            'model': 'llama2:7b-chat-q4_0',
            'prompt': user_text,
            'stream': False
        }
    )
    return response.json()['response']
```

**Tradeoff:** Ollama takes ~2-3 seconds per response on Jetson (slower than Grok)

**My Recommendation for Phase 2:**
- **Use Grok API first** (fast, simple, proven)
- **Keep Ollama as fallback** (just in case internet is down)

---

### 5. TEXT-TO-SPEECH (TTS): Coqui or gTTS

**Recommendation:** Start with **gTTS** (easiest), upgrade to **Coqui** (better quality)

#### Option A: gTTS (Recommended First)

**Pros:**
- ✅ Dead simple to use
- ✅ Uses Google's TTS (high quality)
- ✅ Fast (0.2-0.5s latency)
- ✅ 1 line of code

**Cons:**
- ❌ Requires internet connection
- ❌ Slight cloud dependency (but acceptable fallback)

**Installation:**
```bash
pip install gtts
```

**Usage:**
```python
from gtts import gTTS
import os

def speak(text: str, volume: float = 0.3):
    """Convert text to speech and play immediately"""
    
    tts = gTTS(text=text, lang='en', slow=False)
    temp_file = "/tmp/r2d2_speech.mp3"
    tts.save(temp_file)
    
    # Play using existing ALSA setup
    os.system(f"ffplay -nodisp -autoexit -volume {int(volume*100)} {temp_file}")
    
    # Clean up
    os.remove(temp_file)

# Usage
speak("Hello, I'm R2D2! How can I help you today?", volume=0.3)
```

**Latency:** ~0.5 seconds total (generation + playback)

---

#### Option B: Coqui TTS (Better Quality)

**When:** After gTTS is working

**Pros:**
- ✅ Fully local (no internet)
- ✅ Natural-sounding voices
- ✅ Can run R2D2 personality voice

**Cons:**
- ❌ Slower (1-2 seconds per sentence)
- ❌ Uses more resources initially

**Installation:**
```bash
pip install TTS

# Download model (happens once)
python3 -c "from TTS.api import TTS; TTS(model_name='tts_models/en/ljspeech/tacotron2-DDC')"
```

**Usage:**
```python
from TTS.api import TTS

tts = TTS(model_name="tts_models/en/ljspeech/tacotron2-DDC", gpu=True)

def speak_coqui(text: str, volume: float = 0.3):
    """Local TTS with better quality"""
    
    tts.tts_to_file(text=text, file_path="/tmp/r2d2_speech.wav")
    
    os.system(f"ffplay -nodisp -autoexit -volume {int(volume*100)} /tmp/r2d2_speech.wav")
    os.remove("/tmp/r2d2_speech.wav")
```

---

### 6. COMMAND EXTRACTION & ROS Integration

Only discrete commands go into ROS 2. Here's the smart approach:

```python
import rclpy
from std_msgs.msg import String

def publish_command(command_name: str, args: dict = None):
    """Publish command to ROS 2 for robot actions"""
    
    node = rclpy.create_node('speech_command_publisher')
    pub = node.create_publisher(String, '/r2d2/cmd/speech_intent', 10)
    
    message = {
        'command': command_name,
        'args': args or {}
    }
    
    msg = String()
    msg.data = json.dumps(message)
    pub.publish(msg)
    
    node.destroy_node()

# Mapping from LLM intent to ROS command
COMMAND_MAP = {
    'follow_person': {
        'topic': '/r2d2/cmd/follow_person',
        'parser': lambda text: {}  # No args needed
    },
    'navigate_to': {
        'topic': '/r2d2/cmd/navigate_to',
        'parser': extract_location_from_text  # Custom parser
    },
    'pan_camera': {
        'topic': '/r2d2/cmd/pan_camera',
        'parser': extract_direction_from_text  # Custom parser
    },
    'follow_sound': {
        'topic': '/r2d2/cmd/follow_sound',
        'parser': lambda text: {}
    }
}

def execute_command(llm_response: dict):
    """Parse LLM response and execute ROS command if needed"""
    
    command = llm_response.get('command')
    response_text = llm_response.get('response')
    
    # Speak the response
    speak(response_text)
    
    # If there's a command, publish to ROS
    if command and command in COMMAND_MAP:
        handler = COMMAND_MAP[command]
        args = handler['parser'](response_text)
        publish_command(command, args)
        print(f"[ROS] Published command: {command} with args: {args}")
```

---

## Full Pipeline Flow: From Microphone to Action

```
1. MICROPHONE DAEMON (continuous background process)
   ├─ ReSpeaker 2-Mic HAT
   ├─ Ring buffer (circular, stores last 5 seconds)
   └─ Wake word detector (Porcupine)
   
2. WAKE WORD DETECTED → Signal STT to start recording
   └─ Begin capturing full utterance (until silence timeout)
   
3. STT TRANSCRIPTION (Whisper, ~0.5s)
   └─ User text: "Come over here"
   
4. LLM PROCESSING (Grok API, ~0.3s)
   ├─ Input: "Come over here"
   ├─ Output: {
   │   'response': "I'm coming right over!",
   │   'command': 'follow_person'
   │ }
   └─ Parse command if present
   
5. TTS SYNTHESIS (gTTS, ~0.3s)
   ├─ Text: "I'm coming right over!"
   └─ Audio: MP3 stream
   
6. AUDIO PLAYBACK (PAM8403 speaker, ~2s)
   └─ User hears: "I'm coming right over!"
   
7. ROS COMMAND EXECUTION (if command exists)
   ├─ Publish: /r2d2/cmd/follow_person
   └─ Navigation node receives and executes
   
8. TOTAL LATENCY: ~1.5 seconds (wake word to speech output)
```

---

## Implementation Roadmap (Phase 2)

### Sprint 2.1: Microphone Input (Week 1)
- [ ] Install & physically connect ReSpeaker 2-Mic HAT
- [ ] Test ALSA audio recording
- [ ] Create `MicrophoneManager` Python class
- [ ] Implement circular buffer (5-second ring buffer)
- [ ] Test continuous audio capture

### Sprint 2.2: Wake Word Detection (Week 2)
- [ ] Install Porcupine SDK
- [ ] Create wake word detector
- [ ] Integrate with microphone manager
- [ ] Test "Hey R2D2" detection accuracy
- [ ] Add LED feedback for wake word (ReSpeaker LEDs)

### Sprint 2.3: STT Integration (Week 2-3)
- [ ] Install OpenAI Whisper
- [ ] Download base model (first run auto-downloads)
- [ ] Create `SpeechToText` wrapper class
- [ ] Test transcription accuracy
- [ ] Measure latency on Jetson GPU
- [ ] Optimize: Try different Whisper models

### Sprint 2.4: LLM Integration (Week 3)
- [ ] Get Grok API key
- [ ] Implement LLM response function
- [ ] Create system prompt for R2D2 personality
- [ ] Test command parsing from LLM output
- [ ] Implement command extraction logic

### Sprint 2.5: TTS Integration (Week 4)
- [ ] Install gTTS (first version)
- [ ] Integrate with existing PAM8403 speaker
- [ ] Test audio playback (reuse 050_AUDIO_SETUP.md config)
- [ ] Measure latency & quality
- [ ] Add volume control parameter

### Sprint 2.6: Full Pipeline Integration (Week 4-5)
- [ ] Connect all components end-to-end
- [ ] Create `SpeechPipeline` orchestrator class
- [ ] Test: "Hey R2D2" → STT → LLM → TTS → Speaker
- [ ] Debug latency bottlenecks
- [ ] Add error handling (network down, API limits, etc.)

### Sprint 2.7: ROS 2 Command Bridge (Week 5)
- [ ] Create ROS 2 node for command publishing
- [ ] Map LLM commands to ROS topics
- [ ] Test: LLM command → ROS publish → (future) navigation node
- [ ] Add logging for command execution

### Sprint 2.8: Testing & Optimization (Week 6)
- [ ] Run full system stress tests
- [ ] Measure latency end-to-end
- [ ] Test with different accents & background noise
- [ ] Optimize GPU usage (profile with `tegrastats`)
- [ ] Create test scenarios & validation checklist

---

## System Requirements & Resource Usage

### Hardware Required
- ✅ NVIDIA Jetson AGX Orin 64GB (you have this)
- ✅ ReSpeaker 2-Mic HAT (you have this)
- ✅ PAM8403 speaker (already set up via 050_AUDIO_SETUP.md)
- ✅ Microphone input (ReSpeaker provides this)

### Software Dependencies
```bash
# Core packages
pip install whisper groq gtts porcupine pyaudio

# Optional (for better TTS later)
pip install TTS

# Already installed (from Phase 1)
# - rclpy (ROS 2)
# - opencv-python (perception)
# - numpy, scipy
```

### Estimated Resource Usage (All Components Running)

| Component | CPU | GPU | RAM | Notes |
|-----------|-----|-----|-----|-------|
| ReSpeaker daemon | 2-3% | 0% | 50MB | Continuous listening |
| Porcupine wake word | 1-2% | 0% | 30MB | Running always |
| Whisper STT (base) | 5% | 40% | 800MB | Only during transcription |
| Grok API (network) | 2% | 0% | 30MB | Network I/O only |
| gTTS synthesis | 3% | 0% | 50MB | Only during TTS |
| ffplay speaker | 2% | 0% | 20MB | Only during playback |
| **IDLE (combined)** | **6%** | **0%** | **~150MB** | Just waiting for wake word |
| **PEAK (STT+TTS)** | **12%** | **40%** | **~900MB** | All components active |

**Headroom Available:** You're using <15% of total system resources (plenty left for Phase 3 navigation)

---

## Integration with Phase 1 (Perception)

The speech system doesn't replace Phase 1; it enhances it:

```
PERCEPTION (Phase 1)               SPEECH (Phase 2)
─────────────────────────────────────────────────
/r2d2/perception/person_id    ←→  Context awareness
  (who's speaking?)                  ("Hi Severin, how are you?")

/r2d2/perception/brightness   ←→  Adapt behavior
  (low light, respond quieter)

/r2d2/perception/face_count   ←→  Address specific person
  (one person? all of them?)         ("Welcome back!")

FUTURE PHASE 3 (Navigation)        SPEECH (Phase 2)
─────────────────────────────────────────────────
/r2d2/cmd/follow_person       ←   From: "come to me"
/r2d2/cmd/navigate_to         ←   From: "go to the kitchen"
/r2d2/cmd/pan_camera          ←   From: "look at this"
```

---

## Why This Architecture is Best

### ✅ Advantages

1. **Minimal ROS Overhead**
   - Speech processing as standalone daemon
   - ROS only for discrete commands
   - No serialization/deserialization latency

2. **Separation of Concerns**
   - Daemon handles: wake word, STT, LLM, TTS
   - ROS handles: robot action commands
   - Easy to test components independently

3. **High Responsiveness**
   - 1.5-second latency from "Hey R2D2" to hearing response
   - Acceptable for natural conversation

4. **Scalability**
   - Easy to add multi-turn conversation memory
   - Easy to add emotion detection
   - Easy to add external APIs (weather, news, etc.)

5. **Offline Capability**
   - Whisper (STT): fully local ✅
   - Porcupine (wake word): fully local ✅
   - Coqui (TTS): fully local ✅ (fallback)
   - Grok (LLM): requires internet (but has fallback)

6. **Maintainability**
   - Clear data flow
   - Easy to debug each component separately
   - Well-documented APIs
   - Open-source components

---

## How to Proceed: Step-by-Step

### Phase 2.0: Preparation (This Week)

**1. Document ReSpeaker Setup**
```bash
# Check ReSpeaker detection
i2cdetect -y 1  # Should show ReSpeaker I2C address
arecord -l       # Should show ReSpeaker in list
```

**2. Set Up API Keys**
- [ ] Create Picovoice account: https://console.picovoice.ai/ (free tier)
- [ ] Create Groq account: https://console.groq.com/ (free tier)
- [ ] Save keys to: `~/.r2d2/api_keys.env` (git-ignored)

**3. Create Project Structure**
```bash
mkdir -p ~/dev/r2d2/r2d2_speech/{src,tests,models}
touch ~/dev/r2d2/r2d2_speech/__init__.py
```

**4. Install Dependencies**
```bash
pip install \
  whisper \
  groq \
  gtts \
  porcupine \
  pyaudio \
  requests
```

### Phase 2.1: Start with Microphone (Week 1)

Create `~/dev/r2d2/r2d2_speech/microphone_manager.py`:

```python
#!/usr/bin/env python3
"""ReSpeaker microphone manager with ring buffer"""

import pyaudio
import numpy as np
from collections import deque
import threading

class MicrophoneManager:
    def __init__(self, sample_rate=16000, chunk_size=512, buffer_duration=5):
        self.sample_rate = sample_rate
        self.chunk_size = chunk_size
        self.buffer = deque(maxlen=sample_rate * buffer_duration // chunk_size)
        
        # Detect ReSpeaker device
        self.audio = pyaudio.PyAudio()
        self.device_index = self._find_respeaker_device()
        
        self.stream = self.audio.open(
            format=pyaudio.paFloat32,
            channels=2,  # ReSpeaker has 2 channels
            rate=sample_rate,
            input=True,
            input_device_index=self.device_index,
            frames_per_buffer=chunk_size
        )
        
        # Start recording thread
        self.recording = True
        self.thread = threading.Thread(target=self._record_loop)
        self.thread.daemon = True
        self.thread.start()
    
    def _find_respeaker_device(self):
        """Find ReSpeaker audio device"""
        for i in range(self.audio.get_device_count()):
            info = self.audio.get_device_info_by_index(i)
            if 'ReSpeaker' in info['name']:
                return i
        # Fallback to default device
        return 0
    
    def _record_loop(self):
        """Continuously record audio"""
        while self.recording:
            try:
                data = self.stream.read(self.chunk_size, exception_on_overflow=False)
                audio_np = np.frombuffer(data, dtype=np.float32)
                self.buffer.append(audio_np)
            except Exception as e:
                print(f"Recording error: {e}")
    
    def get_recent_audio(self, duration_seconds=3):
        """Get the last N seconds of audio"""
        chunks_needed = int(duration_seconds * self.sample_rate / self.chunk_size)
        recent = list(self.buffer)[-chunks_needed:]
        return np.concatenate(recent) if recent else np.array([])
    
    def stop(self):
        self.recording = False
        self.stream.stop_stream()
        self.stream.close()
        self.audio.terminate()

# Test
if __name__ == "__main__":
    mic = MicrophoneManager()
    print("Recording for 10 seconds... (ring buffer active)")
    
    import time
    for i in range(10):
        time.sleep(1)
        recent = mic.get_recent_audio(duration_seconds=2)
        print(f"[{i+1}s] Ring buffer has {len(recent)} samples")
    
    mic.stop()
```

### Phase 2.2: Test Wake Word (Week 2)

Create `~/dev/r2d2/test_wake_word.py`:

```python
#!/usr/bin/env python3
"""Test Porcupine wake word detection with ReSpeaker"""

import porcupine
from r2d2_speech.microphone_manager import MicrophoneManager
import os

# Get API key
ACCESS_KEY = os.getenv("PORCUPINE_ACCESS_KEY")

def test_wake_word():
    porcupine_handle = porcupine.create(
        keywords=['jarvis'],  # Or custom: 'hey r2d2'
        access_key=ACCESS_KEY
    )
    
    mic = MicrophoneManager()
    print("🎤 Listening for 'Jarvis'...")
    print("Say: 'Jarvis, hello!'")
    
    detected = False
    for _ in range(300):  # 30 seconds
        audio = mic.get_recent_audio(duration_seconds=0.5)
        
        if len(audio) > 512:
            # Convert to int16 for Porcupine
            audio_int16 = (audio * 32767).astype('int16')
            
            if porcupine_handle.process(audio_int16):
                print("✅ WAKE WORD DETECTED!")
                detected = True
                break
    
    mic.stop()
    porcupine_handle.delete()
    
    if not detected:
        print("❌ No wake word detected (check microphone/API key)")

if __name__ == "__main__":
    test_wake_word()
```

### Phase 2.3: Test STT (Week 2-3)

```bash
# Record 5 seconds of audio
ffmpeg -f alsa -i hw:2,0 -t 5 test_audio.wav

# Transcribe with Whisper
python3 -c "
import whisper
model = whisper.load_model('base', device='cuda')
result = model.transcribe('test_audio.wav')
print(f'Transcribed: {result[\"text\"]}')
"
```

### Phase 2.4: Test Grok LLM (Week 3)

```bash
export GROQ_API_KEY="your_key_here"

python3 << 'EOF'
from groq import Groq

client = Groq()
response = client.chat.completions.create(
    model="mixtral-8x7b-32768",
    messages=[{"role": "user", "content": "Who are you?"}],
    max_tokens=50
)
print(response.choices[0].message.content)
EOF
```

### Phase 2.5: Test TTS (Week 4)

```python
from gtts import gTTS
import os

tts = gTTS("Hello, I'm R2D2")
tts.save("/tmp/test.mp3")

# Play using your existing ALSA setup
os.system("ffplay -nodisp -autoexit /tmp/test.mp3")
```

### Phase 2.6: Full Pipeline (Week 4-5)

Create `~/dev/r2d2/r2d2_speech/speech_pipeline.py` to orchestrate all components.

---

## Tools & Technologies Summary

| Component | Tool | Type | Cloud | Local | Cost |
|-----------|------|------|-------|-------|------|
| **Microphone** | ReSpeaker 2-Mic HAT | Hardware | — | ✅ | $40 (owned) |
| **Wake Word** | Picovoice Porcupine | Software | optional | ✅ | Free tier |
| **STT** | OpenAI Whisper | Software | optional | ✅ | Free |
| **LLM** | Grok API (xAI) | Cloud | ✅ | — | Free tier |
| **LLM (fallback)** | Ollama + Llama2 | Software | — | ✅ | Free |
| **TTS** | gTTS (Google) | Cloud | ✅ | — | Free |
| **TTS (fallback)** | Coqui TTS | Software | — | ✅ | Free |
| **Speaker** | PAM8403 + FFplay | Hardware/Software | — | ✅ | Already set up |

---

## FAQ

### Q: Do I need to integrate everything into ROS 2?

**A:** No. Only discrete commands ("come to me", "go to kitchen") need ROS 2. Keep the speech pipeline as a standalone daemon. This keeps latency low and code simple.

### Q: Should I use OpenAI's Whisper API or local Whisper?

**A:** Use **local Whisper** (fully free, runs on your GPU, no internet dependency). The Jetson GPU can handle the base model at ~0.5s latency.

### Q: Will Whisper + Grok + gTTS work offline?

**A:** Not completely. Grok needs internet (but there's a free tier). For fully offline, you'd switch to local Ollama (slower) + Coqui TTS.

### Q: How long will the full pipeline take (wake word to hearing response)?

**A:** ~1.5 seconds:
- Wake word detection: 0.1s
- Recording: 0.3-1s (until silence)
- Whisper STT: 0.5s
- Grok API: 0.3s
- gTTS: 0.3s
- Playback: 1-5s (depending on response length)

**Total interactive latency:** 1.5s (before hearing response)

### Q: Can I add emotion/tone to the responses?

**A:** Yes, very easily. Just add to the system prompt: "Speak in a cheerful, R2D2-like manner."

### Q: What about multi-turn conversation (remembering context)?

**A:** Easy to add. Keep a conversation history array and include it in the Grok prompt.

### Q: Should I start with gTTS or Coqui?

**A:** Start with **gTTS** (1 line of code, works immediately). Switch to Coqui after you have the full pipeline working (adds 1-2 seconds latency but better quality).

---

## Next: Create Implementation Plan

Once you approve this architecture, I'll create:

1. **Detailed sprint breakdown** (exact files to create & functions to implement)
2. **Test scripts** (one for each component, can run independently)
3. **Integration guide** (how to connect to existing Phase 1 perception)
4. **Troubleshooting guide** (common issues on Jetson + ReSpeaker)
5. **Performance optimization** (profiling, latency reduction strategies)

---

**Document Version:** 1.0  
**Status:** Recommended Architecture, Ready for Implementation  
**Created:** December 9, 2025  
**Author:** Claude  
**Next Step:** Approval + Sprint 2.0 kickoff
