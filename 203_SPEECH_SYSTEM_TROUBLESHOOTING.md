# R2D2 Speech System - Troubleshooting Guide
## Debug Procedures for OpenAI Realtime API Integration

**Date:** December 21, 2025  
**Purpose:** Comprehensive troubleshooting for speech system  
**Platform:** NVIDIA Jetson AGX Orin 64GB + ROS 2 Humble

---

## Systematic Debug Procedure

Follow these steps in order when speech system is not working:

### Step 1: Check Speech Service

```bash
# Check service status
sudo systemctl status r2d2-speech-node --no-pager

# Should show "active (running)"
# If not, check logs:
sudo journalctl -u r2d2-speech-node --since "5 minutes ago"
```

### Step 2: Check ROS2 Node

```bash
cd ~/dev/r2d2/ros2_ws && source install/setup.bash
ros2 node list | grep speech_node

# EXPECT: /speech_node

# Check lifecycle state
ros2 lifecycle get /speech_node
# EXPECT: "active [3]"
```

### Step 3: Check Services Available

```bash
ros2 service list | grep speech

# EXPECT:
# /r2d2/speech/start_session
# /r2d2/speech/stop_session
```

### Step 4: Check Topics

```bash
# Session status
timeout 3 ros2 topic echo /r2d2/speech/session_status --once
# EXPECT: {"status": "inactive"} when idle

# Transcripts
timeout 10 ros2 topic echo /r2d2/speech/transcripts
# Only publishes during active session
```

### Step 5: Test Hardware

**Microphone:**
```bash
# Check USB microphone detected
arecord -l | grep -i hyperx
# EXPECT: card X: QuadCastS

# Test recording
arecord -D hw:3,0 -f S16_LE -r 48000 -c 2 -d 3 test.wav
# Should create test.wav file
```

**Speaker:**
```bash
# Test playback
ffplay -nodisp -autoexit test.wav
# Should hear recorded audio
```

---

## Common Issues and Solutions

### Issue: Speech Service Won't Start

**Symptoms:**
- Service shows "failed" or "inactive"
- Node not in lifecycle list
- Service call hangs

**Diagnosis:**
```bash
# Check service logs
sudo journalctl -u r2d2-speech-node -n 100

# Look for errors:
# - "OpenAI API key not found"
# - "Failed to import"
# - "Device not found"
# - "Connection refused"
```

**Solutions:**

1. **OpenAI API key not configured:**
   ```bash
   # Check if key exists
   if [ -z "$OPENAI_API_KEY" ]; then
     echo "❌ API key not set"
   else
     echo "✅ API key set"
   fi
   
   # Set API key
   echo "export OPENAI_API_KEY='sk-proj-...'" >> ~/.bashrc
   source ~/.bashrc
   
   # Restart service
   sudo systemctl restart r2d2-speech-node
   ```

2. **Missing dependencies:**
   ```bash
   # Activate virtualenv and install
   source ~/depthai_env/bin/activate
   pip install openai websocket-client pyaudio scipy numpy
   
   # Rebuild package
   cd ~/dev/r2d2/ros2_ws
   colcon build --packages-select r2d2_speech
   sudo systemctl restart r2d2-speech-node
   ```

3. **Microphone not detected:**
   ```bash
   # List audio devices
   arecord -l
   
   # Check HyperX specifically
   python3 -c "import pyaudio; p = pyaudio.PyAudio(); [print(f'{i}: {p.get_device_info_by_index(i)[\"name\"]}') for i in range(p.get_device_count())]"
   
   # If HyperX not found, check USB connection
   lsusb | grep -i hyperx
   ```

---

### Issue: "Works Once Then Stops"

**Symptoms:**
- First gesture after boot doesn't start speech
- Must stop first before starting works
- Session already active on boot

**Root Cause:** `auto_start=true` in config (speech auto-starts on boot)

**Diagnosis:**
```bash
# Check auto_start parameter
ros2 param get /speech_node auto_start
# Should be: False

# Check session status after boot
ros2 topic echo /r2d2/speech/session_status --once
# Should be: {"status": "inactive"}
```

**Solution:**
```bash
# Fix config
nano ~/dev/r2d2/ros2_ws/src/r2d2_speech/config/speech_params.yaml
# Change line 18: auto_start: false

# Rebuild
cd ~/dev/r2d2/ros2_ws
colcon build --packages-select r2d2_speech

# Restart service
sudo systemctl restart r2d2-speech-node

# Verify
ros2 param get /speech_node auto_start
# EXPECT: False
```

---

### Issue: Service Call Hangs/Times Out

**Symptoms:**
- `ros2 service call` never returns
- Gesture triggers but no response
- No error messages

**Diagnosis:**
```bash
# Test service manually with timeout
timeout 10 ros2 service call /r2d2/speech/start_session std_srvs/srv/Trigger

# Check if service is processing
sudo journalctl -u r2d2-speech-node -f | grep "Service:"

# Check lifecycle state
ros2 lifecycle get /speech_node
# Must be "active [3]"
```

**Solutions:**

1. **Node not in active state:**
   ```bash
   # Check current state
   ros2 lifecycle get /speech_node
   
   # If not active, activate:
   ros2 lifecycle set /speech_node configure
   ros2 lifecycle set /speech_node activate
   ```

2. **OpenAI API connection issue:**
   ```bash
   # Check network connectivity
   ping -c 3 api.openai.com
   
   # Test API key manually
   curl https://api.openai.com/v1/models \
     -H "Authorization: Bearer $OPENAI_API_KEY"
   
   # Should return list of models, not 401 error
   ```

3. **Service executor blocked:**
   ```bash
   # Restart node
   sudo systemctl restart r2d2-speech-node
   
   # Wait 10 seconds for initialization
   sleep 10
   
   # Retry service call
   ```

---

### Issue: No Audio Heard from AI

**Symptoms:**
- Service starts successfully
- Transcripts show up
- No audio playback

**Diagnosis:**
```bash
# Check speaker device
aplay -l | grep "card 1"

# Test speaker manually
speaker-test -t wav -c 2 -D hw:1,0

# Check logs for audio playback
sudo journalctl -u r2d2-speech-node -f | grep -i "audio\|playback\|speaker"

# Check audio stream manager logs
sudo journalctl -u r2d2-speech-node -f | grep "AudioStreamManager"
```

**Solutions:**

1. **Speaker not configured:**
   ```bash
   # Check ALSA config
   cat /etc/asound.conf
   # Should point to hw:1,0
   
   # Test ffplay
   ffplay -nodisp -autoexit ~/dev/r2d2/ros2_ws/src/r2d2_audio/r2d2_audio/assets/audio/Voicy_R2-D2\ -\ 2.mp3
   ```

2. **Audio device mismatch:**
   ```bash
   # Check configured device in speech params
   cat ~/dev/r2d2/ros2_ws/src/r2d2_speech/config/speech_params.yaml | grep speaker_device
   
   # Should match ALSA device
   # Update if needed, then rebuild and restart
   ```

3. **PyAudio issue:**
   ```bash
   # Reinstall PyAudio
   source ~/depthai_env/bin/activate
   pip uninstall pyaudio
   pip install pyaudio
   
   # Restart service
   sudo systemctl restart r2d2-speech-node
   ```

---

### Issue: Microphone Not Working

**Symptoms:**
- Service starts but doesn't respond to speech
- No transcripts published
- "No audio detected" in logs

**Diagnosis:**
```bash
# Check microphone device
arecord -l | grep -i hyperx

# Test recording
arecord -D hw:3,0 -f S16_LE -r 48000 -c 2 -d 5 test.wav
# Speak during recording, check file size
ls -lh test.wav
# Should be ~1-2 MB for 5 seconds

# Play back recording
ffplay -nodisp -autoexit test.wav
# Should hear your voice

# Check logs for audio capture
sudo journalctl -u r2d2-speech-node -f | grep -i "capture\|microphone\|hyperx"
```

**Solutions:**

1. **HyperX not detected:**
   ```bash
   # Check USB connection
   lsusb | grep -i hyperx
   # Should show: Kingston Technology HyperX QuadCast S
   
   # Try different USB port
   # Avoid USB hubs if possible
   ```

2. **Muted or low gain:**
   ```bash
   # Check ALSA mixer
   alsamixer
   # Press F6 to select HyperX device
   # Ensure capture not muted (MM = muted)
   # Press spacebar to unmute
   # Adjust gain with arrow keys
   
   # Or command line:
   amixer -c 3 set Mic 80%
   amixer -c 3 set Mic cap
   ```

3. **Wrong device configured:**
   ```bash
   # Auto-detect should find HyperX
   # If not, check device index
   python3 -c "import pyaudio; p = pyaudio.PyAudio(); [print(f'{i}: {p.get_device_info_by_index(i)[\"name\"]}') for i in range(p.get_device_count())]"
   
   # Update config if needed
   nano ~/dev/r2d2/ros2_ws/src/r2d2_speech/config/speech_params.yaml
   # Set microphone_device to correct index
   ```

---

### Issue: Poor Transcription Quality

**Symptoms:**
- Speech recognized but words wrong
- Many transcription errors
- Partial phrases missed

**Diagnosis:**
```bash
# Check microphone quality
arecord -D hw:3,0 -f S16_LE -r 48000 -c 2 -d 5 test.wav
ffplay -nodisp -autoexit test.wav
# Listen for clarity

# Check background noise
# Record in silent room vs. noisy environment

# Check logs for audio levels
sudo journalctl -u r2d2-speech-node -f | grep -i "audio level\|amplitude"
```

**Solutions:**

1. **Microphone too far:**
   - Move closer to microphone (30-50 cm recommended)
   - HyperX QuadCast S is sensitive, but not omnidirectional

2. **Background noise:**
   - Reduce ambient noise
   - Use noise gate feature (if available)
   - Check HyperX gain settings

3. **Microphone gain too low:**
   ```bash
   # Increase capture volume
   alsamixer
   # Select HyperX device (F6)
   # Increase gain with up arrow
   
   # Test with higher gain
   arecord -D hw:3,0 -f S16_LE -r 48000 -c 2 -d 3 test_loud.wav
   ```

4. **Sample rate mismatch:**
   - System should auto-resample 48kHz → 24kHz
   - Check logs for resampling errors

---

### Issue: High Latency (>2 seconds)

**Symptoms:**
- Long delay between speech and response
- Conversation feels slow
- Multiple seconds lag

**Diagnosis:**
```bash
# Check network latency
ping -c 10 api.openai.com
# Should be <100ms typically

# Check CPU usage
top -bn1 | grep python

# Monitor logs for timing
sudo journalctl -u r2d2-speech-node -f | grep -i "latency\|took\|duration"
```

**Solutions:**

1. **Network issue:**
   ```bash
   # Check bandwidth
   speedtest-cli
   
   # Check for packet loss
   ping -c 100 api.openai.com | grep loss
   
   # If high loss, check WiFi signal or use Ethernet
   ```

2. **CPU overload:**
   ```bash
   # Check other processes
   top -bn1 | head -20
   
   # Reduce perception system load if needed
   ros2 param set /image_listener recognition_frame_skip 5
   ```

3. **Audio buffering:**
   - Check buffer sizes in config
   - Smaller buffers = lower latency but more CPU
   - Default should be optimized

---

### Issue: Session Disconnects Unexpectedly

**Symptoms:**
- Session starts but drops mid-conversation
- "Connection closed" errors
- Must restart to work again

**Diagnosis:**
```bash
# Check logs for disconnect reason
sudo journalctl -u r2d2-speech-node -n 200 | grep -i "disconnect\|closed\|error"

# Check OpenAI API status
curl https://status.openai.com/api/v2/status.json

# Check network stability
ping -c 100 api.openai.com
```

**Solutions:**

1. **API key issue:**
   ```bash
   # Verify API key valid
   curl https://api.openai.com/v1/models \
     -H "Authorization: Bearer $OPENAI_API_KEY"
   
   # Check for quota limits
   # Check OpenAI dashboard for usage
   ```

2. **Network interruption:**
   - Check WiFi stability
   - Use Ethernet if available
   - Check router logs

3. **API timeout:**
   - OpenAI may timeout inactive sessions
   - Normal behavior, restart session
   - Check timeout parameters in config

---

### Issue: Conversation Interrupts During Face Loss

**Symptoms:**
- Speech stops when looking away briefly
- Session disconnects when not in RED status
- Watchdog timer too aggressive

**Diagnosis:**
```bash
# Monitor SPEAKING state
sudo journalctl -u r2d2-gesture-intent -f | grep "SPEAKING"

# Check watchdog protection
ros2 param get /gesture_intent_node auto_shutdown_timeout_seconds
# EXPECT: 35.0

# Monitor person status during conversation
ros2 topic echo /r2d2/audio/person_status | grep status
```

**Solutions:**

1. **Verify SPEAKING protection:**
   ```bash
   # Check parameter
   ros2 param get /gesture_intent_node auto_shutdown_timeout_seconds
   # Should be: 35.0
   
   # If wrong, update config
   nano ~/dev/r2d2/ros2_ws/src/r2d2_gesture/config/gesture_params.yaml
   # Set: auto_shutdown_timeout_seconds: 35.0
   
   # Rebuild and restart
   cd ~/dev/r2d2/ros2_ws
   colcon build --packages-select r2d2_gesture
   sudo systemctl restart r2d2-gesture-intent
   ```

2. **Check logs show protection:**
   ```bash
   # During conversation, look away
   sudo journalctl -u r2d2-gesture-intent -f | grep "timer"
   # EXPECT: "Person non-RED, starting 35s timer"
   # When face returns: "Person returned to RED, timer RESET"
   ```

---

## Database Issues

### Issue: Conversations Not Saved

**Symptoms:**
- Transcripts work but not in database
- Database file missing or empty
- Query returns no results

**Diagnosis:**
```bash
# Check database exists
ls -la ~/dev/r2d2/data/conversations.db

# Check database has data
sqlite3 ~/dev/r2d2/data/conversations.db "SELECT COUNT(*) FROM conversations;"

# Check logs for database errors
sudo journalctl -u r2d2-speech-node -f | grep -i "database\|sqlite"
```

**Solutions:**

1. **Database not created:**
   ```bash
   # Manually initialize
   cd ~/dev/r2d2/ros2_ws/src/r2d2_speech/r2d2_speech
   python3 -c "from utils.sqlite_store import SQLiteStore; store = SQLiteStore(); print('Database initialized')"
   ```

2. **Permissions issue:**
   ```bash
   # Check ownership
   ls -la ~/dev/r2d2/data/conversations.db
   # Should be owned by severin:severin
   
   # Fix permissions
   sudo chown severin:severin ~/dev/r2d2/data/conversations.db
   sudo chmod 644 ~/dev/r2d2/data/conversations.db
   ```

3. **Path misconfigured:**
   ```bash
   # Check config
   cat ~/dev/r2d2/ros2_ws/src/r2d2_speech/config/speech_params.yaml | grep database
   
   # Should point to correct path
   # Update if needed, rebuild, restart
   ```

---

## Recovery Procedures

### Quick Service Restart

```bash
sudo systemctl restart r2d2-speech-node

# Wait 10 seconds for initialization
sleep 10

# Verify running
sudo systemctl status r2d2-speech-node
ros2 lifecycle get /speech_node
```

### Full Reset

```bash
# Stop service
sudo systemctl stop r2d2-speech-node

# Kill any hanging Python processes
pkill -f "r2d2_speech"

# Wait 5 seconds
sleep 5

# Start service
sudo systemctl start r2d2-speech-node

# Wait 10 seconds
sleep 10

# Verify
ros2 lifecycle get /speech_node
# EXPECT: active [3]
```

### Rebuild Package

```bash
cd ~/dev/r2d2/ros2_ws
source /opt/ros/humble/setup.bash

# Clean build
rm -rf build/r2d2_speech install/r2d2_speech

# Rebuild
colcon build --packages-select r2d2_speech

# Restart service
sudo systemctl restart r2d2-speech-node
```

---

## Monitoring Commands

### Real-Time Monitoring

```bash
# Terminal 1: Service logs
sudo journalctl -u r2d2-speech-node -f

# Terminal 2: Session status
ros2 topic echo /r2d2/speech/session_status

# Terminal 3: Transcripts
ros2 topic echo /r2d2/speech/transcripts

# Terminal 4: Service calls
ros2 service list | grep speech
```

### Performance Monitoring

```bash
# Check CPU usage
top -bn1 | grep -E "python.*speech"

# Check network usage
iftop -i wlan0

# Check API call timing
sudo journalctl -u r2d2-speech-node -f | grep -E "took|latency|duration"
```

---

## Quick System Check Script

Save as `~/dev/r2d2/check_speech_system.sh`:

```bash
#!/bin/bash
echo "=== R2D2 Speech System Check ==="
echo ""

echo "Service:"
status=$(systemctl is-active r2d2-speech-node)
if [ "$status" = "active" ]; then
  echo "  ✅ r2d2-speech-node"
else
  echo "  ❌ r2d2-speech-node: $status"
fi

echo ""
echo "ROS2 Node:"
cd ~/dev/r2d2/ros2_ws && source install/setup.bash
if ros2 node list | grep -q speech_node; then
  state=$(ros2 lifecycle get /speech_node 2>&1 | grep -o "active\|inactive\|unconfigured")
  echo "  ✅ Node running (state: $state)"
else
  echo "  ❌ Node NOT FOUND"
fi

echo ""
echo "Hardware:"
if arecord -l | grep -qi hyperx; then
  echo "  ✅ HyperX microphone detected"
else
  echo "  ❌ HyperX microphone NOT FOUND"
fi

if aplay -l | grep -q "card 1"; then
  echo "  ✅ Speaker device available"
else
  echo "  ❌ Speaker device NOT FOUND"
fi

echo ""
echo "Configuration:"
if [ -n "$OPENAI_API_KEY" ]; then
  echo "  ✅ OPENAI_API_KEY set"
else
  echo "  ❌ OPENAI_API_KEY not set"
fi

echo ""
echo "Network:"
if ping -c 1 -W 2 api.openai.com &>/dev/null; then
  echo "  ✅ OpenAI API reachable"
else
  echo "  ❌ Cannot reach OpenAI API"
fi

echo ""
echo "If all checks pass, system is ready!"
```

```bash
chmod +x ~/dev/r2d2/check_speech_system.sh
```

---

## Related Documentation

**Reference:**
- `200_SPEECH_SYSTEM_REFERENCE.md` - Technical reference

**Installation:**
- `201_SPEECH_SYSTEM_INSTALLATION.md` - Setup guide

**Quick Start:**
- `202_SPEECH_SYSTEM_QUICK_START.md` - Daily operations

**Integration:**
- `100_PERCEPTION_STATUS_REFERENCE.md` - Gesture triggering
- `001_ARCHITECTURE_OVERVIEW.md` - Overall system

---

**Document Version:** 1.0  
**Last Updated:** December 21, 2025  
**Status:** Production troubleshooting guide

