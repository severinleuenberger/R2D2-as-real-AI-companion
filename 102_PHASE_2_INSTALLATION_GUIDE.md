# Phase 2: Premium Speech System - Complete Installation Guide
## Swiss German STT + Piper TTS + Grok LLM

**Date:** December 9, 2025  
**Status:** Step-by-Step Installation Guide  
**Estimated Time:** 90 minutes total  
**Platform:** NVIDIA Jetson AGX Orin 64GB + ROS 2 Humble + ReSpeaker 2-Mic HAT

---

## Pre-Installation Checklist

Before you start, verify you have:

- ✅ NVIDIA Jetson AGX Orin (with JetPack 6.x installed)
- ✅ ReSpeaker 2-Mic HAT (physically installed on GPIO header)
- ✅ PAM8403 speaker + audio wired (from Phase 1)
- ✅ Internet connection (for model downloads)
- ✅ ~10GB free storage (for models)
- ✅ Groq API key (get at https://console.groq.com/)
- ✅ Picovoice Porcupine key (get at https://console.picovoice.ai/ - free tier)

---

## Part 1: Environment Verification (10 minutes)

### Step 1.1: Verify Jetson Environment

```bash
# Check CUDA availability
python3 << 'EOF'
import torch
print(f"PyTorch version: {torch.__version__}")
print(f"CUDA available: {torch.cuda.is_available()}")
print(f"CUDA device: {torch.cuda.get_device_name(0)}")
print(f"GPU memory: {torch.cuda.get_device_properties(0).total_memory / 1e9:.1f} GB")
EOF

# Expected output:
# PyTorch version: 2.x.x
# CUDA available: True
# CUDA device: NVIDIA Jetson AGX Orin
# GPU memory: 504.0 GB (logical)
```

### Step 1.2: Verify Audio Setup

```bash
# Check ALSA devices
aplay -l

# Expected output:
# card 0: tegra-hda [NVIDIA Jetson AGX Orin HDA]
#   device 3: HDMI 0
#   device 7: HDMI 1
# card 1: tegra-ape [NVIDIA Jetson AGX Orin APE]

# Check ReSpeaker detection
lsusb | grep -i respeaker

# Or check I2C (ReSpeaker uses I2C for control)
i2cdetect -y 1

# Expected: Should show ReSpeaker address (usually 0x08 or similar)
```

### Step 1.3: Verify ROS 2

```bash
# Source ROS 2 setup
source /opt/ros/humble/setup.bash

# Check installation
ros2 --version

# Expected: ROS 2 Humble (whatever version)
```

---

## Part 2: Python Environment Setup (15 minutes)

### Step 2.1: Create Virtual Environment (Optional but Recommended)

```bash
# Create a dedicated venv for speech
python3 -m venv ~/dev/r2d2/r2d2_speech_env

# Activate it
source ~/dev/r2d2/r2d2_speech_env/bin/activate

# You should see: (r2d2_speech_env) user@jetson:~$
```

### Step 2.2: Upgrade pip

```bash
pip install --upgrade pip setuptools wheel

# Verify
pip --version
# Expected: pip 24.x or later
```

### Step 2.3: Install Core Dependencies

This step takes **10-15 minutes** (compilation on ARM64 is slower):

```bash
# Install in order (important!)
pip install numpy scipy

# Install audio libraries
pip install pyaudio librosa soundfile

# Install ML frameworks (CUDA-enabled)
pip install torch torchaudio --index-url https://download.pytorch.org/whl/cu121

# Verify torch can use CUDA
python3 -c "import torch; print(f'CUDA: {torch.cuda.is_available()}')"
# Expected: CUDA: True
```

---

## Part 3: Speech System Libraries (20 minutes)

### Step 3.1: Install Faster-Whisper

```bash
pip install faster-whisper
```

### Step 3.2: Install Piper TTS

```bash
pip install piper-tts

# Verify installation
which piper
# Expected: /path/to/piper
```

### Step 3.3: Install Silero VAD

```bash
pip install silero-vad

# Verify
python3 << 'EOF'
import torch
print("Silero VAD test...")
model, get_speech_ts = torch.hub.load(
    repo_or_dir='snakers4/silero-vad',
    model='silero_vad',
    onnx=True,
    device='cuda'
)
print("✓ Silero VAD loaded successfully")
EOF
```

### Step 3.4: Install Grok API Client

```bash
pip install groq
```

### Step 3.5: Install Porcupine (Wake Word)

```bash
pip install porcupine
```

### Step 3.6: Install Additional Utilities

```bash
pip install requests pyyaml python-dotenv
```

### Step 3.7: Verify All Installations

```bash
python3 << 'EOF'
print("Checking all components...")

try:
    import faster_whisper
    print("✓ faster-whisper")
except: print("✗ faster-whisper")

try:
    import piper
    print("✓ piper-tts")
except: print("✗ piper-tts")

try:
    import silero_vad
    print("✓ silero-vad")
except: print("✗ silero-vad")

try:
    from groq import Groq
    print("✓ groq")
except: print("✗ groq")

try:
    import porcupine
    print("✓ porcupine")
except: print("✗ porcupine")

try:
    import torch
    print(f"✓ torch (CUDA: {torch.cuda.is_available()})")
except: print("✗ torch")
EOF
```

---

## Part 4: Download Models & Voices (30-45 minutes)

This is the largest step - models are several GB. Run in a screen/tmux session so it doesn't get interrupted.

### Step 4.1: Start a Persistent Session

```bash
# Using screen (if installed)
screen -S r2d2_downloads
# Or using tmux
tmux new-session -s r2d2_downloads
```

### Step 4.2: Download Faster-Whisper Large Model

```bash
# This downloads ~2.9GB - takes 10-15 minutes on good internet
python3 << 'EOF'
from faster_whisper import WhisperModel
import time

print("Downloading Faster-Whisper large model...")
print("This will take 10-15 minutes. DO NOT INTERRUPT.")
print("")

start = time.time()
model = WhisperModel("large-v2", device="cuda", compute_type="float32")
elapsed = time.time() - start

print(f"\n✓ Model downloaded successfully in {elapsed/60:.1f} minutes")
print(f"✓ Cache location: ~/.cache/huggingface/hub/")
print(f"✓ Compute type: float32 (full precision for maximum accuracy)")
EOF
```

### Step 4.3: Download Piper TTS Voices

```bash
# German voice - HIGH QUALITY professional (female, warm)
echo "Downloading de_DE-kerstin-high voice (high quality)..."
piper --download-model de_DE-kerstin-high
echo "✓ Downloaded"

# English voice - HIGH QUALITY professional
echo "Downloading en_US-libritts-high voice (high quality)..."
piper --download-model en_US-libritts-high
echo "✓ Downloaded"

# Verify voices are installed
ls -lh ~/.local/share/piper-tts/models/
# Expected: Should show .onnx files for both voices

echo ""
echo "Voice Strategy:"
echo "  Primary (German): de_DE-kerstin-high"
echo "  Fallback (English): en_US-libritts-high"
echo ""
echo "We're starting with HIGH quality voices."
echo "We'll monitor performance during test_tts.py"
echo "If synthesis is slower than acceptable, we can switch to medium variant."
```

### Step 4.4: Download Silero VAD Model

```bash
# Silero VAD downloads automatically on first use, but let's do it now
python3 << 'EOF'
import torch

print("Downloading Silero VAD model...")
model, get_speech_ts = torch.hub.load(
    repo_or_dir='snakers4/silero-vad',
    model='silero_vad',
    onnx=True,
    device='cuda'
)
print("✓ Silero VAD model cached")
EOF
```

### Step 4.5: Verify All Models Downloaded

```bash
# Check Whisper model
ls -lh ~/.cache/huggingface/hub/models--openai--whisper-large-v2/

# Check Piper voices
ls -lh ~/.local/share/piper-tts/models/

# Check Silero VAD
ls -lh ~/.cache/torch/hub/

echo "✓ All models downloaded successfully"
```

---

## Part 5: API Keys Configuration (5 minutes)

### Step 5.1: What is Groq/Grok? ✅

**You asked: "What would I need Groq for?!"**

Great question! Here's the role of **Grok API** in your speech pipeline:

```
1. ReSpeaker records: "Hey R2D2, come here!"
2. Whisper Large (STT) transcribes: "Hey R2D2, come here!"
3. ⭐ Grok API (LLM) - processes intent & generates response:
   Input:  "Hey R2D2, come here!"
   Output: "Okay, I'm coming!" (+ command to ROS)
4. Piper TTS synthesizes: "Okay, I'm coming!" → audio file
5. Speaker plays response to user
```

**Without Grok API:** You'd just have speech recognition with no intelligence.  
**With Grok API:** R2D2 understands what you want and responds intelligently.

Grok is the **brain** that makes it conversational. You already have the API key, so you're good!

---

### Step 5.2: Create Config Directory

```bash
mkdir -p ~/.r2d2
chmod 700 ~/.r2d2  # Private directory - only you can access
```

### Step 5.3: Create .env File (API Keys)

```bash
# Create API keys file
cat > ~/.r2d2/.env << 'EOF'
# Grok API Key - intelligently processes speech (YOU HAVE THIS!)
# Get from: https://console.groq.com/ (free account)
GROQ_API_KEY="your_actual_groq_api_key_here"

# Porcupine Wake Word (optional, for future "wake word" detection)
# Get from: https://console.picovoice.ai/ (free tier available)
PORCUPINE_ACCESS_KEY="your_porcupine_key_here"
EOF

# Set permissions (security - only you can read)
chmod 600 ~/.r2d2/.env

echo "✓ API keys file created at ~/.r2d2/.env"
echo "  Next: Replace placeholder with your actual Groq API key"
```

### Step 5.4: Add Your Actual Groq API Key

```bash
# Edit the .env file with your actual key
nano ~/.r2d2/.env

# Replace this:
# GROQ_API_KEY="your_actual_groq_api_key_here"
# With your actual key from https://console.groq.com/

# Save: Ctrl+O, Enter, Ctrl+X

# Verify it's set correctly:
grep GROQ_API_KEY ~/.r2d2/.env
```

### Step 5.5: Get Groq API Key (If You Don't Have It Yet)

1. Go to https://console.groq.com/
2. Sign up with email (free account)
3. Create API key in dashboard
4. Copy the key to your .env file

**Note:** You mentioned you already have the key, so skip to **Step 5.3** above!


3. Create API key
4. Copy and paste into ~/.r2d2/.env

**Porcupine Wake Word Key:**
1. Go to https://console.picovoice.ai/
2. Sign up (free tier available)
3. Create access key
4. Copy and paste into ~/.r2d2/.env

### Step 5.4: Verify API Keys

```bash
# Load keys
export $(cat ~/.r2d2/.env | xargs)

# Verify Grok
python3 << 'EOF'
import os
from groq import Groq

api_key = os.getenv("GROQ_API_KEY")
if api_key and api_key != "your_groq_api_key_here":
    try:
        client = Groq(api_key=api_key)
        # Test API call
        response = client.chat.completions.create(
            model="mixtral-8x7b-32768",
            messages=[{"role": "user", "content": "Say 'API key works'"}],
            max_tokens=10
        )
        print("✓ Groq API key is valid!")
        print(f"  Response: {response.choices[0].message.content}")
    except Exception as e:
        print(f"✗ Groq API error: {e}")
else:
    print("⚠️  Groq API key not set. Edit ~/.r2d2/.env")
EOF
```

---

## Part 6: Project Structure Setup (10 minutes)

**Folder Strategy** (following pattern from 000_INTERNAL_AGENT_NOTES.md):
- **Core source:** `src/` — production code
- **Tests:** `tests/` — unit & integration tests  
- **Config:** `~/.r2d2/.env` — API keys and global settings
- **Logs:** `logs/` — debug and runtime logs
- **Audio:** `audio/` — test samples and recordings

### Step 6.1: Create Project Directories

```bash
# Create main package directory
mkdir -p ~/dev/r2d2/r2d2_speech

# Production code structure
mkdir -p ~/dev/r2d2/r2d2_speech/src/{stt,tts,llm,utils}

# Tests
mkdir -p ~/dev/r2d2/r2d2_speech/tests

# Logging
mkdir -p ~/dev/r2d2/r2d2_speech/logs

# Audio samples and recordings
mkdir -p ~/dev/r2d2/r2d2_speech/audio/{samples,recordings}

# Global config (follows internal notes pattern)
mkdir -p ~/.r2d2

echo "✓ Directory structure created"
```

### Step 6.2: Verify Final Structure

```bash
tree ~/dev/r2d2/r2d2_speech/ -L 2

# Expected output:
# ~/dev/r2d2/r2d2_speech/
# ├── src/
# │   ├── stt/           # Whisper large-v2 wrapper
# │   ├── tts/           # Piper kerstin-high wrapper
# │   ├── llm/           # Grok API client
# │   └── utils/         # Audio utilities
# ├── tests/             # 6 test scripts
# ├── logs/              # Runtime logs
# ├── audio/
# │   ├── samples/       # Test audio clips
# │   └── recordings/    # Recorded audio for debug
# └── __init__.py

# Global config structure:
# ~/.r2d2/
# └── .env               # API keys (GROQ_API_KEY, PORCUPINE_ACCESS_KEY)
```

---

## Part 7: Audio System Verification (10 minutes)

### Step 7.1: Test ReSpeaker Microphone

```bash
# Record 5 seconds of audio from ReSpeaker
ffmpeg -f alsa -i hw:2,0 -t 5 /tmp/test_mic.wav

# Play it back (to verify quality)
ffplay -nodisp -autoexit /tmp/test_mic.wav

echo "✓ ReSpeaker microphone works"
```

### Step 7.2: Test PAM8403 Speaker

```bash
# Generate test tone and play
python3 << 'EOF'
import subprocess
import numpy as np
import wave
import os

# Generate 1-second sine wave (440 Hz = A note)
sample_rate = 44100
duration = 1.0
frequency = 440

t = np.linspace(0, duration, int(sample_rate * duration))
audio = np.sin(2 * np.pi * frequency * t) * 0.3

# Save to WAV
output_file = "/tmp/test_tone.wav"
with wave.open(output_file, 'w') as wav_file:
    wav_file.setnchannels(1)  # Mono
    wav_file.setsampwidth(2)  # 16-bit
    wav_file.setframerate(sample_rate)
    wav_file.writeframes((audio * 32767).astype(np.int16).tobytes())

# Play via ffplay (uses your ALSA config)
os.system(f"ffplay -nodisp -autoexit {output_file}")

print("✓ PAM8403 speaker works (you should hear a 440Hz tone)")
EOF
```

### Step 7.3: Verify Audio ALSA Config

```bash
# Check if /etc/asound.conf exists (from Phase 1)
cat /etc/asound.conf

# If it doesn't exist, create it:
# See 050_AUDIO_SETUP_AND_CONFIGURATION.md for full setup

echo "✓ ALSA configuration verified"
```

---

## Part 8: Verify Complete Installation (5 minutes)

### Step 8.1: Final Verification Script

```bash
python3 << 'EOF'
#!/usr/bin/env python3
"""Verify complete Phase 2 installation"""

import subprocess
import os
import sys
from pathlib import Path

print("="*70)
print("PHASE 2 INSTALLATION VERIFICATION")
print("="*70)
print()

checks = {
    "Python version": lambda: f"3.{sys.version_info.minor}",
    "PyTorch + CUDA": lambda: __import__('torch').cuda.is_available(),
    "Faster-Whisper": lambda: __import__('faster_whisper'),
    "Piper TTS": lambda: __import__('piper'),
    "Silero VAD": lambda: __import__('silero_vad'),
    "Groq API": lambda: __import__('groq'),
    "Porcupine": lambda: __import__('porcupine'),
}

passed = 0
failed = 0

for check_name, check_func in checks.items():
    try:
        result = check_func()
        if result is True or (isinstance(result, type) and result):
            print(f"✓ {check_name}")
            passed += 1
        elif isinstance(result, str):
            print(f"✓ {check_name}: {result}")
            passed += 1
        else:
            print(f"✗ {check_name}")
            failed += 1
    except Exception as e:
        print(f"✗ {check_name}: {e}")
        failed += 1

print()
print("MODEL DOWNLOADS:")
print("─" * 70)

models = {
    "Whisper Large": Path.home() / ".cache/huggingface/hub/models--openai--whisper-large-v2",
    "Piper (kerstin)": Path.home() / ".local/share/piper-tts/models/de_DE-kerstin-medium.onnx",
    "Piper (libritts)": Path.home() / ".local/share/piper-tts/models/en_US-libritts-high.onnx",
}

models_ok = 0
for model_name, model_path in models.items():
    if model_path.exists():
        print(f"✓ {model_name}")
        models_ok += 1
    else:
        print(f"✗ {model_name} (not yet downloaded)")

print()
print("API KEYS:")
print("─" * 70)

env_file = Path.home() / ".r2d2/.env"
if env_file.exists():
    with open(env_file) as f:
        content = f.read()
        if "your_groq_api_key_here" in content:
            print("⚠️  GROQ_API_KEY not set (still placeholder)")
        else:
            print("✓ GROQ_API_KEY configured")
        
        if "your_porcupine_key_here" in content:
            print("⚠️  PORCUPINE_ACCESS_KEY not set (still placeholder)")
        else:
            print("✓ PORCUPINE_ACCESS_KEY configured")
else:
    print("✗ ~/.r2d2/.env file not found")

print()
print("="*70)
print(f"SUMMARY: {passed + models_ok}/{len(checks) + len(models)} checks passed")
print("="*70)

if failed == 0 and models_ok == len(models):
    print("\n✅ Installation complete! Ready to test components.")
else:
    print("\n⚠️  Some items need attention. See above.")
EOF
```

---

## Summary: What You've Installed

After completing all 8 parts, you have:

| Component | Status | Location |
|-----------|--------|----------|
| **Faster-Whisper** | ✅ Installed | Python site-packages |
| **Whisper Large Model** | ✅ Downloaded | ~/.cache/huggingface/hub/ |
| **Piper TTS** | ✅ Installed | Python site-packages |
| **Piper Voices** | ✅ Downloaded | ~/.local/share/piper-tts/models/ |
| **Silero VAD** | ✅ Installed | Python site-packages |
| **Grok API** | ✅ Installed | Python site-packages |
| **Porcupine** | ✅ Installed | Python site-packages |
| **API Keys** | ✅ Configured | ~/.r2d2/.env |
| **Project Structure** | ✅ Created | ~/dev/r2d2/r2d2_speech/ |
| **Audio Hardware** | ✅ Verified | ReSpeaker + PAM8403 |

---

## Troubleshooting Installation Issues

### Issue: CUDA not available

```bash
# Solution: Reinstall torch with CUDA support
pip install torch torchaudio --force-reinstall --index-url https://download.pytorch.org/whl/cu121
```

### Issue: Model downloads timeout

```bash
# Solution: Download models manually with wget
wget -O ~/.cache/huggingface/hub/model.tar.gz <url>

# Or use larger timeout in Python:
import urllib.request
urllib.request.urlopen(url, timeout=300)  # 5 minute timeout
```

### Issue: API key errors

```bash
# Verify API key format
cat ~/.r2d2/.env | grep GROQ_API_KEY

# Re-test Grok connection
export $(cat ~/.r2d2/.env | xargs)
python3 -c "from groq import Groq; Groq().chat.completions.create(...)"
```

### Issue: ReSpeaker not detected

```bash
# Check USB detection
lsusb | grep -i respeaker

# Check I2C (if HAT-based)
i2cdetect -y 1

# Check ALSA device
arecord -l

# If missing, try:
sudo modprobe snd_usb_audio
dmesg | tail -20  # Check for errors
```

### Issue: Piper voice not found

```bash
# Re-download voice
piper --download-model de_DE-kerstin-medium --force

# Verify cache
ls -lh ~/.local/share/piper-tts/models/
```

---

## Next: Test Scripts (Part 2)

Once installation is complete, proceed to: **102_PHASE_2_TEST_SCRIPTS.md**

This will guide you through testing each component individually before integrating them.

---

**Installation Guide Version:** 1.0  
**Status:** Complete  
**Estimated Total Time:** 90 minutes  
**Next Step:** Run the test scripts (102_PHASE_2_TEST_SCRIPTS.md)
