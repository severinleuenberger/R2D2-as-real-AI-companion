# R2D2 Speech System - Installation Guide
## Step-by-Step Setup for OpenAI Realtime API Integration

**Date:** December 17, 2025  
**Status:** Complete Installation Guide  
**Platform:** NVIDIA Jetson AGX Orin 64GB + ROS 2 Humble  
**Estimated Time:** 60-90 minutes

---

## Prerequisites

Before starting installation, verify you have:

### Hardware Requirements

✅ **NVIDIA Jetson AGX Orin 64GB** (or compatible Jetson)
- JetPack 6.x installed
- Internet connection (for OpenAI API)
- At least 5GB free storage

✅ **HyperX QuadCast S USB Microphone**
- USB connection (plug-and-play)
- No special drivers needed (uses standard USB audio class)

✅ **PAM8403 Speaker Amplifier** (from Phase 1)
- Connected to J511 Audio Header
- Configured and tested (see Phase 1 audio setup docs)

**Historical Note:** Original planning documents specified ReSpeaker 2-Mic HAT with GPIO/I2S connection. The actual implementation uses HyperX QuadCast S USB for simplicity and quality.

### Software Requirements

✅ **ROS 2 Humble** installed and working
```bash
ros2 --version
# Should show: ROS 2 Humble
```

✅ **Python 3.10** (should be pre-installed on Jetson)
```bash
python3 --version
# Should show: Python 3.10.x
```

✅ **Git** (for cloning/updates)
```bash
git --version
```

### API Requirements

✅ **OpenAI API Key**
- Sign up at: https://platform.openai.com/
- Create API key in dashboard
- Requires credit card for paid tier (or use free tier limits)
- Free tier: Limited requests per day
- Paid tier: Pay-per-use (speech conversations are ~$0.01-0.05 each)

---

## Part 1: Hardware Setup (15 minutes)

### Step 1.1: Connect HyperX QuadCast S

1. **Physical Connection:**
   ```bash
   # Plug HyperX QuadCast S into USB port on Jetson
   # Use USB 3.0 port for best performance (blue connector)
   ```

2. **Verify USB Detection:**
   ```bash
   # Check USB devices
   lsusb | grep -i hyperx
   
   # Expected output: Something like:
   # Bus 001 Device 003: ID 0951:16d8 Kingston Technology HyperX QuadCast S
   ```

3. **Verify Audio Device:**
   ```bash
   # List all audio devices
   python3 << 'EOF'
   import pyaudio
   p = pyaudio.PyAudio()
   for i in range(p.get_device_count()):
       info = p.get_device_info_by_index(i)
       if info['maxInputChannels'] > 0:
           print(f"[{i}] {info['name']} - {info['maxInputChannels']} channels")
   p.terminate()
   EOF
   
   # Should see: HyperX QuadCast S in the list
   ```

### Step 1.2: Test Microphone

```bash
# Record 3 seconds of audio
arecord -D hw:CARD=QuadCast,DEV=0 -f S16_LE -r 48000 -c 2 -d 3 test_hyperx.wav

# Play it back (through HDMI or speaker)
aplay test_hyperx.wav

# If you hear your voice, microphone is working!
```

### Step 1.3: Verify Speaker (PAM8403)

```bash
# Test speaker with tone (should be already configured from Phase 1)
speaker-test -t wav -c 2

# Press Ctrl+C to stop

# If no sound, refer to Phase 1 audio setup documentation
```

---

## Part 2: Python Environment Setup (20 minutes)

### Step 2.1: Create Virtual Environment

```bash
# Create dedicated virtualenv for speech system
cd ~/dev/r2d2
python3 -m venv r2d2_speech_env

# Activate it
source r2d2_speech_env/bin/activate

# You should see: (r2d2_speech_env) in your prompt
```

**Important:** This virtualenv must be activated before using the speech system.

### Step 2.2: Upgrade pip

```bash
# Upgrade pip to latest version
pip install --upgrade pip setuptools wheel

# Verify
pip --version
# Should show: pip 24.x or later
```

### Step 2.3: Install Core Dependencies

```bash
# Install audio libraries
pip install pyaudio

# If pyaudio fails, install system dependencies first:
sudo apt-get update
sudo apt-get install -y portaudio19-dev python3-pyaudio

# Then retry:
pip install pyaudio
```

### Step 2.4: Install Python Packages

```bash
# Install all required packages
pip install \
  openai \
  websockets \
  pyaudio \
  numpy \
  scipy \
  python-dotenv

# This takes ~5-10 minutes on Jetson
```

### Step 2.5: Verify Installation

```bash
# Check all packages installed correctly
python3 << 'EOF'
print("Checking installations...")
try:
    import openai; print("✓ openai")
except: print("✗ openai")
try:
    import websockets; print("✓ websockets")
except: print("✗ websockets")
try:
    import pyaudio; print("✓ pyaudio")
except: print("✗ pyaudio")
try:
    import numpy; print("✓ numpy")
except: print("✗ numpy")
try:
    import scipy; print("✓ scipy")
except: print("✗ scipy")
try:
    from dotenv import load_dotenv; print("✓ python-dotenv")
except: print("✗ python-dotenv")
print("Done!")
EOF

# All packages should show ✓
```

---

## Part 3: Speech System Installation (15 minutes)

### Step 3.1: Verify File Structure

```bash
# Check core speech system exists
ls -la ~/dev/r2d2/r2d2_speech/

# Expected directories:
# - config/        (configuration management)
# - realtime/      (OpenAI client)
# - storage/       (database)
# - utils/         (audio pipeline)

# If missing, the system needs to be cloned/restored from git
```

### Step 3.2: Create Data Directory

```bash
# Create directory for conversation database
mkdir -p ~/dev/r2d2/r2d2_speech/data

# Set permissions
chmod 755 ~/dev/r2d2/r2d2_speech/data
```

### Step 3.3: Test Core System (without ROS2)

```bash
# Activate virtualenv
source ~/dev/r2d2/r2d2_speech_env/bin/activate

# Test microphone detection
python3 << 'EOF'
from r2d2_speech.utils.audio_stream import find_hyperx_device

device_index, device_info = find_hyperx_device()
if device_index is not None:
    print(f"✓ HyperX detected: {device_info['name']}")
    print(f"  Device index: {device_index}")
    print(f"  Sample rate: {device_info['defaultSampleRate']} Hz")
    print(f"  Channels: {device_info['maxInputChannels']}")
else:
    print("✗ HyperX not detected")
    print("  Make sure microphone is plugged in via USB")
EOF
```

---

## Part 4: ROS2 Integration (20 minutes)

### Step 4.1: Build ROS2 Package

```bash
# Navigate to ROS2 workspace
cd ~/dev/r2d2/ros2_ws

# Source ROS2
source /opt/ros/humble/setup.bash

# Build the speech package
colcon build --packages-select r2d2_speech --symlink-install

# This takes ~2-5 minutes
```

**Expected output:**
```
Starting >>> r2d2_speech
Finished <<< r2d2_speech [XXs]

Summary: 1 package finished [XXs]
```

### Step 4.2: Source Installation

```bash
# Source the newly built package
source ~/dev/r2d2/ros2_ws/install/setup.bash

# Verify package is available
ros2 pkg list | grep r2d2_speech

# Should show: r2d2_speech
```

### Step 4.3: Verify Launch File

```bash
# Check launch file exists
ls -la ~/dev/r2d2/ros2_ws/src/r2d2_speech/launch/speech_node.launch.py

# Check it's executable (should be -rwxr-xr-x)
```

---

## Part 5: Configuration (10 minutes)

### Step 5.1: Create Configuration Directory

```bash
# Create config directory (if not exists)
mkdir -p ~/.r2d2

# Set restrictive permissions (API keys are sensitive)
chmod 700 ~/.r2d2
```

### Step 5.2: Configure API Key

```bash
# Create .env file
nano ~/.r2d2/.env

# Add the following (replace with your actual API key):
```

**Content for `~/.r2d2/.env`:**
```bash
# OpenAI API Key (required)
OPENAI_API_KEY=sk-your-actual-api-key-here

# Model configuration (optional, has defaults)
REALTIME_MODEL=gpt-4o-realtime-preview-2024-12-17
REALTIME_VOICE=sage

# Audio configuration (optional, auto-detects HyperX)
MIC_DEVICE=
MIC_NATIVE_SAMPLE_RATE=48000
MIC_SAMPLE_RATE=24000

# Database configuration (optional, has default)
DB_PATH=/home/severin/dev/r2d2/r2d2_speech/data/conversations.db
```

**Save and exit:** Ctrl+O, Enter, Ctrl+X

### Step 5.3: Set File Permissions

```bash
# Protect API key file
chmod 600 ~/.r2d2/.env

# Verify permissions (should show -rw-------)
ls -la ~/.r2d2/.env
```

### Step 5.4: Verify API Key

```bash
# Load and verify API key
export $(cat ~/.r2d2/.env | xargs)

# Test API key format (should start with sk-)
echo $OPENAI_API_KEY | grep -q "^sk-" && echo "✓ API key format OK" || echo "✗ Invalid API key format"

# Test API connection
python3 << 'EOF'
import os
from openai import OpenAI

api_key = os.getenv('OPENAI_API_KEY')
if not api_key:
    print("✗ OPENAI_API_KEY not set")
    exit(1)

try:
    client = OpenAI(api_key=api_key)
    # Simple test: list models
    models = client.models.list()
    print("✓ API key is valid!")
    print(f"  Connected to OpenAI successfully")
except Exception as e:
    print(f"✗ API key test failed: {e}")
EOF
```

---

## Part 6: Launch Script Setup (5 minutes)

### Step 6.1: Verify Launch Script

```bash
# Check launch script exists
ls -la ~/dev/r2d2/launch_ros2_speech.sh

# If missing, create it:
cat > ~/dev/r2d2/launch_ros2_speech.sh << 'EOF'
#!/bin/bash
# R2D2 Speech System - ROS2 Launch Script

set -e

# Colors for output
GREEN='\033[0;32m'
RED='\033[0;31m'
NC='\033[0m' # No Color

echo "R2D2 Speech System - Launching..."

# 1. Activate virtualenv
VENV_PATH="$HOME/dev/r2d2/r2d2_speech_env"
if [ ! -d "$VENV_PATH" ]; then
    echo -e "${RED}✗ Virtualenv not found: $VENV_PATH${NC}"
    echo "  Run: python3 -m venv $VENV_PATH"
    exit 1
fi

source "$VENV_PATH/bin/activate"
echo -e "${GREEN}✓ Virtualenv activated${NC}"
echo "  Using Python: $(which python3)"

# 2. Check API key
if ! grep -q "OPENAI_API_KEY=sk-" ~/.r2d2/.env 2>/dev/null; then
    echo -e "${RED}✗ OPENAI_API_KEY not configured${NC}"
    echo "  Edit: ~/.r2d2/.env"
    echo "  Add: OPENAI_API_KEY=sk-..."
    exit 1
fi

# 3. Set environment
export $(cat ~/.r2d2/.env | xargs)
export PYTHONPATH="$HOME/dev/r2d2:$PYTHONPATH"

# 4. Source ROS2
source /opt/ros/humble/setup.bash
source "$HOME/dev/r2d2/ros2_ws/install/setup.bash"

echo -e "${GREEN}✓ ROS2 sourced${NC}"

# 5. Launch node
echo "Launching speech node..."
ros2 launch r2d2_speech speech_node.launch.py
EOF

# Make executable
chmod +x ~/dev/r2d2/launch_ros2_speech.sh
```

### Step 6.2: Test Launch Script (Dry Run)

```bash
# Test that script can find all dependencies
~/dev/r2d2/launch_ros2_speech.sh --help

# If script runs without errors finding files, you're ready!
# Press Ctrl+C to exit
```

---

## Part 7: Initial Testing (10 minutes)

### Step 7.1: Launch the Node

```bash
# Terminal 1: Launch speech node
cd ~/dev/r2d2
./launch_ros2_speech.sh
```

**Expected output:**
```
R2D2 Speech System - Launching...
✓ Virtualenv activated
  Using Python: /home/severin/dev/r2d2/r2d2_speech_env/bin/python3
✓ ROS2 sourced
Launching speech node...
[INFO] [speech_node]: Initializing Speech Processing Node...
[INFO] [speech_node]: Configuring...
[INFO] [speech_node]: ✓ Configuration complete
[INFO] [speech_node]: Activating...
[INFO] [speech_node]: Session: ros2-20251217-HHMMSS
[INFO] [speech_node]: ✓ Connected to OpenAI API
[INFO] [speech_node]: ✓ Speech system running
```

### Step 7.2: Monitor Transcripts

**Terminal 2: Monitor user transcripts**
```bash
source /opt/ros/humble/setup.bash
source ~/dev/r2d2/ros2_ws/install/setup.bash
ros2 topic echo /r2d2/speech/user_transcript
```

**Terminal 3: Monitor assistant transcripts**
```bash
source /opt/ros/humble/setup.bash
source ~/dev/r2d2/ros2_ws/install/setup.bash
ros2 topic echo /r2d2/speech/assistant_transcript
```

### Step 7.3: Test Conversation

1. **Speak into HyperX microphone:** "Hello, can you hear me?"
2. **Watch Terminal 2:** Should show your transcribed speech
3. **Watch Terminal 3:** Should show assistant's response
4. **Listen to speaker:** Should hear AI response played back

**If all three work, installation is successful! ✅**

### Step 7.4: Verify Database

```bash
# Check database was created
ls -la ~/dev/r2d2/r2d2_speech/data/conversations.db

# View recent messages
sqlite3 ~/dev/r2d2/r2d2_speech/data/conversations.db << 'EOF'
SELECT role, text, datetime(created_at) as time 
FROM messages 
ORDER BY id DESC 
LIMIT 5;
EOF

# Should show your recent conversation!
```

---

## Part 7B: Gesture Control Integration (Optional but Recommended)

The gesture intent service auto-starts and controls the speech system.

**Verify Gesture Service:**
```bash
sudo systemctl status r2d2-gesture-intent.service
# Should show: active (running)
```

**If not running:**
```bash
sudo systemctl enable r2d2-gesture-intent.service
sudo systemctl start r2d2-gesture-intent.service
```

**Test Gesture Control:**
1. Stand in front of camera (person recognized = RED status)
2. Raise index finger → Speech service starts + beep
3. Make fist → Speech service stops + beep
4. Walk away >35 seconds → Auto-shutdown + beep

**Train Gestures (if needed):**
```bash
cd ~/dev/r2d2/tests/face_recognition
source ~/depthai_env/bin/activate
python3 train_manager.py
# Select option 8: Train gestures for person
```

See [303_GESTURE_TRAINING_GUIDE.md](303_GESTURE_TRAINING_GUIDE.md) for complete training instructions.

---

## Part 8: Verification Checklist

Run through this checklist to ensure everything is working:

### Hardware
- [ ] HyperX QuadCast S detected via `lsusb`
- [ ] HyperX shows up in PyAudio device list
- [ ] PAM8403 speaker produces sound

### Software
- [ ] Python virtualenv created and activates
- [ ] All Python packages installed (openai, websockets, etc.)
- [ ] ROS2 package builds without errors
- [ ] Launch script runs without errors

### Configuration
- [ ] `~/.r2d2/.env` exists with OPENAI_API_KEY
- [ ] API key starts with "sk-"
- [ ] API key validates successfully

### Functionality
- [ ] Speech node launches and connects to API
- [ ] User transcript topic publishes when speaking
- [ ] Assistant transcript topic publishes with responses
- [ ] Audio plays through speaker
- [ ] Database records conversations

### ROS2 Integration
- [ ] Node shows in `ros2 node list`
- [ ] Topics show in `ros2 topic list`
- [ ] Lifecycle commands work (`ros2 lifecycle get /speech_node`)

---

## Troubleshooting

### Issue: HyperX not detected

**Symptoms:** Auto-detection fails, no audio input

**Solutions:**
```bash
# 1. Check USB connection
lsusb | grep -i hyperx

# 2. Try different USB port (prefer USB 3.0)

# 3. Check dmesg for USB errors
dmesg | tail -30 | grep -i usb

# 4. Manually specify device index
# Edit ~/.r2d2/.env and add:
# MIC_DEVICE=2  # Replace 2 with your device index

# 5. List all devices to find correct index
python3 -c "import pyaudio; p = pyaudio.PyAudio(); [print(f'{i}: {p.get_device_info_by_index(i)[\"name\"]}') for i in range(p.get_device_count())]"
```

### Issue: API connection fails

**Symptoms:** "Failed to connect to OpenAI API"

**Solutions:**
```bash
# 1. Check API key is set
cat ~/.r2d2/.env | grep OPENAI_API_KEY

# 2. Verify API key format (should start with sk-)
echo $OPENAI_API_KEY | grep "^sk-"

# 3. Test internet connection
ping -c 3 api.openai.com

# 4. Check for firewall blocking WebSocket (port 443)

# 5. Verify API key online:
# Visit https://platform.openai.com/api-keys
# Check if key is active
```

### Issue: No audio output

**Symptoms:** Transcripts work, but no sound from speaker

**Solutions:**
```bash
# 1. Test speaker directly
speaker-test -t wav -c 2

# 2. Check ALSA configuration
cat /etc/asound.conf

# 3. Check audio device is set correctly
# In ~/.r2d2/.env:
# SINK_DEVICE=  # Leave empty for default

# 4. Refer to Phase 1 audio setup docs
# PAM8403 configuration is part of Phase 1
```

### Issue: ROS2 package won't build

**Symptoms:** `colcon build` fails

**Solutions:**
```bash
# 1. Clean build
cd ~/dev/r2d2/ros2_ws
rm -rf build/r2d2_speech install/r2d2_speech log/r2d2_speech

# 2. Rebuild
source /opt/ros/humble/setup.bash
colcon build --packages-select r2d2_speech --symlink-install

# 3. Check for missing dependencies
rosdep install --from-paths src --ignore-src -r -y

# 4. Verify ROS2 is properly installed
ros2 --version
```

### Issue: Virtualenv packages not found

**Symptoms:** ImportError when launching

**Solutions:**
```bash
# 1. Activate virtualenv BEFORE sourcing ROS2
source ~/dev/r2d2/r2d2_speech_env/bin/activate
source /opt/ros/humble/setup.bash

# 2. Add to PYTHONPATH
export PYTHONPATH="$HOME/dev/r2d2:$PYTHONPATH"

# 3. Reinstall packages in virtualenv
pip install --force-reinstall openai websockets pyaudio numpy scipy python-dotenv
```

---

## Post-Installation

### Optional: System Startup Service

To start speech system on boot (optional):

```bash
# Create systemd service
sudo nano /etc/systemd/system/r2d2-speech.service
```

**Content:**
```ini
[Unit]
Description=R2D2 Speech ROS2 Node
After=network.target

[Service]
Type=simple
User=severin
WorkingDirectory=/home/severin/dev/r2d2
ExecStart=/home/severin/dev/r2d2/launch_ros2_speech.sh
Restart=on-failure
RestartSec=10

[Install]
WantedBy=multi-user.target
```

**Enable and start:**
```bash
sudo systemctl daemon-reload
sudo systemctl enable r2d2-speech.service
sudo systemctl start r2d2-speech.service

# Check status
sudo systemctl status r2d2-speech.service
```

### Optional: Bash Alias

Add convenient alias to `~/.bashrc`:

```bash
# Add to ~/.bashrc
echo "alias r2d2-speech='~/dev/r2d2/launch_ros2_speech.sh'" >> ~/.bashrc
source ~/.bashrc

# Now you can launch with:
r2d2-speech
```

---

## Next Steps

After successful installation:

1. **Read Quick Start Guide:** `203_SPEECH_SYSTEM_QUICK_START.md`
2. **Read Reference Documentation:** `200_SPEECH_SYSTEM_REFERENCE.md`
3. **Experiment with conversations:** Try different prompts
4. **View conversation history:** Use database viewer scripts
5. **Integrate with other R2D2 systems:** Connect to navigation, perception, etc.

---

## Support

If you encounter issues not covered in troubleshooting:

1. Check system logs:
   ```bash
   # ROS2 logs
   ros2 node list
   ros2 topic list
   ros2 lifecycle get /speech_node
   
   # System logs (if using systemd)
   sudo journalctl -u r2d2-speech.service -f
   ```

2. Check OpenAI API status: https://status.openai.com/

3. Review reference documentation for architecture details

---

**Installation Guide Version:** 1.0  
**Last Updated:** December 17, 2025  
**Estimated Installation Time:** 60-90 minutes  
**Status:** Complete and tested  
**Hardware:** HyperX QuadCast S USB + PAM8403 Speaker


