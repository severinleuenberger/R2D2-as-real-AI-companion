# R2D2 System Integration - Troubleshooting Guide

**Document ID:** 009_SYSTEM_INTEGRATION_TROUBLESHOOTING.md  
**Date:** December 18, 2025  
**Purpose:** Comprehensive troubleshooting procedures for system integration issues  
**For:** Debugging, problem resolution, system recovery

---

## Systematic Debug Procedure

Follow this order when system not working:

### Step 1: Check Services

```bash
systemctl is-active r2d2-camera-perception r2d2-audio-notification \
  r2d2-gesture-intent r2d2-speech-node

# All should show "active"
# If any shows "inactive" or "failed":
systemctl status r2d2-SERVICE-NAME --no-pager
journalctl -u r2d2-SERVICE-NAME --since "5 minutes ago"
```

### Step 2: Check ROS2 Nodes

```bash
cd ~/dev/r2d2/ros2_ws && source install/setup.bash
ros2 node list

# EXPECT to see:
# /camera_node or /oak_d_camera
# /image_listener
# /audio_notification_node
# /gesture_intent_node
# /speech_node
```

### Step 3: Check Topics Publishing

```bash
# Person ID (face recognition)
timeout 3 ros2 topic echo /r2d2/perception/person_id --once
# EXPECT: "severin" (when in front of camera)

# Person status (RED/BLUE/GREEN)
timeout 3 ros2 topic echo /r2d2/audio/person_status --once
# EXPECT: {"status": "red",...} (when recognized)

# Gesture events
timeout 5 ros2 topic echo /r2d2/perception/gesture_event
# Make gesture, EXPECT: "index_finger_up" or "fist"

# Session status
timeout 3 ros2 topic echo /r2d2/speech/session_status --once
# EXPECT: {"status": "inactive"} or {"status": "connected"}
```

### Step 4: Test Audio

```bash
# Test manual beep
ffplay -nodisp -autoexit -v quiet \
  ~/dev/r2d2/ros2_ws/src/r2d2_audio/r2d2_audio/assets/audio/Voicy_R2-D2\ -\ 2.mp3

# If you hear it, audio works
# If not, check volume and speaker connections
```

### Step 5: Test Gesture Detection

```bash
# Monitor gesture events
ros2 topic echo /r2d2/perception/gesture_event

# Make gestures, should see events
# If no events, check camera-perception service
```

---

## Common Issues and Solutions

### Issue: "Works Once Then Stops"

**Symptom:** First gesture after boot doesn't work, but works after stopping first

**Root Cause:** auto_start=true (speech_node auto-starts on boot)

**Solution:**
```bash
# Check auto_start
ros2 param get /speech_node auto_start
# Should be: False

# If True, fix:
nano ~/dev/r2d2/ros2_ws/src/r2d2_speech/config/speech_params.yaml
# Change line 18: auto_start: false

cd ~/dev/r2d2/ros2_ws
colcon build --packages-select r2d2_speech --symlink-install
sudo systemctl restart r2d2-speech-node
```

### Issue: No Beeps Heard

**Symptom:** Recognition/gestures work but no audio feedback

**Diagnosis:**
```bash
# 1. Check audio files exist
ls -la ~/dev/r2d2/ros2_ws/src/r2d2_audio/r2d2_audio/assets/audio/*.mp3

# 2. Check volume
ros2 param get /audio_notification_node audio_volume
# If < 0.1, increase:
ros2 param set /audio_notification_node audio_volume 0.5

# 3. Test manual playback
ffplay -nodisp -autoexit ~/dev/r2d2/ros2_ws/src/r2d2_audio/r2d2_audio/assets/audio/Voicy_R2-D2\ -\ 2.mp3

# 4. Check logs for "Playing audio"
sudo journalctl -u r2d2-audio-notification -f | grep "Playing"
```

**Solutions:**
- Increase volume if too quiet
- Check speaker connections
- Verify audio files exist
- Check ALSA configuration

**Note:** Beeps only play on STATUS TRANSITIONS
- "Hello!" = BLUE → RED (must walk away first to trigger)
- "Start"/"Stop" = session status changes

### Issue: Gestures Ignored

**Symptom:** Gestures detected but no action

**Check Gates:**
```bash
# Monitor gesture_intent logs
sudo journalctl -u r2d2-gesture-intent -f

# Make gesture, check for:
# "Gesture ignored: person_status=..." → Not RED (stand in camera)
# "Gesture ignored: session already active" → Make fist first to stop
# "Gesture ignored: cooldown" → Wait 5 seconds

# Verify person_status is RED
ros2 topic echo /r2d2/audio/person_status --once | grep status
# EXPECT: "red"
```

### Issue: Conversation Interrupts

**Symptom:** Speech stops when looking away briefly

**Check SPEAKING State:**
```bash
# Monitor SPEAKING state
sudo journalctl -u r2d2-gesture-intent -f | grep "SPEAKING"

# During conversation, EXPECT:
# "Entered SPEAKING state"
# "Person non-RED, starting 35s timer" (if look away)
# "Person returned to RED, timer RESET" (when face returns)

# Verify speaking_protection parameter
ros2 param get /gesture_intent_node speaking_protection_seconds
# Should be: 35.0
```

### Issue: RED Status Unstable

**Symptom:** LED flickers, status changes rapidly

**Check RED Timer:**
```bash
# Monitor RED timer resets
sudo journalctl -u r2d2-audio-notification -f | grep "Timer reset"

# Should see frequent resets while in camera
# If not, face recognition may be failing

# Check face recognition confidence
ros2 topic echo /r2d2/perception/face_confidence --once
# Lower is better, should be < 150 for recognized person

# Verify red_status_timeout
ros2 param get /audio_notification_node red_status_timeout_seconds
# Should be: 15.0
```

---

## Debug Mode: Step-by-Step Verification

### 1. Verify Services

```bash
echo "Checking services..."
for service in r2d2-camera-perception r2d2-audio-notification \
               r2d2-gesture-intent r2d2-speech-node; do
  status=$(systemctl is-active $service)
  echo "$service: $status"
  if [ "$status" != "active" ]; then
    echo "  ❌ Not running! Check: journalctl -u $service -n 50"
  fi
done
```

### 2. Verify Nodes

```bash
echo "Checking ROS2 nodes..."
cd ~/dev/r2d2/ros2_ws && source install/setup.bash

required_nodes=("audio_notification_node" "gesture_intent_node" "speech_node" "image_listener" "camera_node")
for node in "${required_nodes[@]}"; do
  if ros2 node list | grep -q "$node"; then
    echo "✅ $node running"
  else
    echo "❌ $node NOT FOUND"
  fi
done
```

### 3. Verify Parameters

```bash
echo "Checking parameters..."
cd ~/dev/r2d2/ros2_ws && source install/setup.bash

echo "auto_start: $(ros2 param get /speech_node auto_start 2>&1 | grep -o 'False\|True')"
echo "red_status_timeout: $(ros2 param get /audio_notification_node red_status_timeout_seconds 2>&1 | grep -o '[0-9.]*')"
echo "speaking_protection: $(ros2 param get /gesture_intent_node speaking_protection_seconds 2>&1 | grep -o '[0-9.]*')"
```

### 4. Test Recognition

```bash
echo "Testing recognition..."
echo "Stand in front of camera for this test"
ros2 topic echo /r2d2/perception/person_id --once
ros2 topic echo /r2d2/audio/person_status --once | grep status
# Both should show recognition
```

### 5. Test Gestures

```bash
echo "Testing gestures..."
echo "Make a gesture now"
timeout 10 ros2 topic echo /r2d2/perception/gesture_event | head -5
# Should see gesture events
```

---

## Recovery Procedures

### Full System Reset

```bash
# Stop all services
sudo systemctl stop r2d2-gesture-intent r2d2-speech-node \
  r2d2-audio-notification r2d2-camera-perception

# Wait 5 seconds
sleep 5

# Start in order
sudo systemctl start r2d2-audio-notification
sleep 2
sudo systemctl start r2d2-camera-perception
sleep 3
sudo systemctl start r2d2-gesture-intent
sudo systemctl start r2d2-speech-node

# Wait 10 seconds for full initialization
sleep 10

# Verify all running
systemctl is-active r2d2-*
```

### Rebuild All Packages

```bash
cd ~/dev/r2d2/ros2_ws
source /opt/ros/humble/setup.bash

# Clean build (if needed)
rm -rf build install log

# Build all R2D2 packages
colcon build --packages-select r2d2_camera r2d2_perception \
  r2d2_audio r2d2_gesture r2d2_speech --symlink-install

# Restart all services
sudo systemctl restart r2d2-camera-perception r2d2-audio-notification \
  r2d2-gesture-intent r2d2-speech-node
```

---

## Known Issues and Workarounds

### Issue: First Gesture After Boot Blocked

**Status:** ✅ FIXED (auto_start=false)

**If still occurs:**
- Verify auto_start=false in all 4 config locations
- Check session_status shows "inactive" after boot
- Make fist first to stop auto-started session, then try finger

### Issue: Conversation Interrupts During Face Loss

**Status:** ✅ FIXED (SPEAKING state protection)

**If still occurs:**
- Verify speaking_protection_seconds = 35.0
- Check logs show "Entered SPEAKING state"
- Verify timer resets when face returns

### Issue: RED Status Changes Too Quickly

**Status:** ✅ FIXED (15s timer)

**If still occurs:**
- Verify red_status_timeout_seconds = 15.0
- Check logs show "Timer reset" frequently
- May need to retrain face recognition if false negatives

---

## Monitoring Commands

### Real-Time Monitoring

```bash
# Open 4 terminals:

# Terminal 1: Gesture events
ros2 topic echo /r2d2/perception/gesture_event

# Terminal 2: Person status
ros2 topic echo /r2d2/audio/person_status | grep status

# Terminal 3: Session status
ros2 topic echo /r2d2/speech/session_status

# Terminal 4: Gesture intent logs
sudo journalctl -u r2d2-gesture-intent -f
```

### Performance Monitoring

```bash
# Check CPU usage
top -bn1 | grep -E "python|Cpu" | head -10

# Check memory
free -h

# Check topic rates
ros2 topic hz /r2d2/perception/person_id
ros2 topic hz /r2d2/audio/person_status
```

---

## Error Messages Reference

### Gesture Intent Node

| Message | Meaning | Action |
|---------|---------|--------|
| "Gesture ignored: person_status=blue" | Not recognized | Stand in camera |
| "Start gesture ignored: session already active" | Can't start twice | Make fist first |
| "Stop gesture ignored: no active session" | Nothing to stop | Raise finger first |
| "Start gesture ignored: cooldown" | Too soon after last | Wait 5 seconds |
| "Service not available" | speech_node not ready | Wait or restart speech_node |
| "Entered SPEAKING state" | Conversation started | Normal operation |
| "Exited SPEAKING state" | Conversation ended | Normal operation |

### Audio Notification Node

| Message | Meaning | Action |
|---------|---------|--------|
| "✓ severin recognized (blue → RED)" | First recognition | Normal, LED ON |
| "✗ severin lost (timer expired: 15s)" | Person left | Normal, LED OFF |
| "RED: Timer reset" | Still recognized | Normal, continuous |
| "🟢 Unknown person detected" | Not target person | Normal, GREEN state |

### Speech Node

| Message | Meaning | Action |
|---------|---------|--------|
| "Service: start_session" | Start requested | Normal |
| "Started" or "Already running" | Session active | Normal |
| "Service: stop_session" | Stop requested | Normal |
| "Stopped" or "No active session" | Session inactive | Normal |
| "Start failed: ..." | Error starting | Check OpenAI API, network, audio |

---

## Diagnostic Scripts

### Quick System Check Script

Save as `~/dev/r2d2/CHECK_SYSTEM_INTEGRATION.sh`:

```bash
#!/bin/bash
echo "=== R2D2 System Integration Check ==="
echo ""

echo "Services:"
systemctl is-active r2d2-camera-perception r2d2-audio-notification \
  r2d2-gesture-intent r2d2-speech-node | \
  paste <(echo "camera-perception") \
        <(echo "audio-notification") \
        <(echo "gesture-intent") \
        <(echo "speech-node") - | column -t

echo ""
echo "Nodes:"
cd ~/dev/r2d2/ros2_ws && source install/setup.bash
ros2 node list | grep -E "audio|gesture|speech|image|camera" | wc -l
echo " critical nodes found"

echo ""
echo "Parameters:"
echo "auto_start: $(ros2 param get /speech_node auto_start 2>&1 | grep -o 'False\|True')"
echo "red_timeout: $(ros2 param get /audio_notification_node red_status_timeout_seconds 2>&1 | grep -o '[0-9.]*')s"
echo "speaking_protection: $(ros2 param get /gesture_intent_node speaking_protection_seconds 2>&1 | grep -o '[0-9.]*')s"

echo ""
echo "If all services active and nodes running, system is ready!"
```

---

## PersonRegistry Troubleshooting

### Issue: "Person Not Resolved" / "target_person" Showing Instead of Actual Name

**Symptom:** System shows "target_person" instead of actual person name

**Root Cause:** PersonRegistry is empty or database not found

**Diagnosis:**
```bash
# Check if persons are registered
cd ~/dev/r2d2/tests/face_recognition
python3 -c "from person_registry import PersonRegistry; r = PersonRegistry(); print(r.list_persons())"

# Check database exists
ls -la ~/dev/r2d2/data/persons.db
```

**Solution:**
```bash
# Run auto-migrate to populate from existing models
cd ~/dev/r2d2/tests/face_recognition
python3 -c "from person_registry import PersonRegistry; r = PersonRegistry(); print(r.auto_migrate())"

# Verify registration
python3 -c "from person_registry import PersonRegistry; r = PersonRegistry(); persons = r.list_persons(); [print(f'{p[\"display_name\"]}: face={\"✓\" if p[\"face_model_path\"] else \"✗\"}, gesture={\"✓\" if p[\"gesture_model_path\"] else \"✗\"}') for p in persons]"
```

**Expected:** At least one person should be listed with face/gesture model paths.

---

## Contact and References

**For comprehensive system details:**
- [007_SYSTEM_INTEGRATION_REFERENCE.md](007_SYSTEM_INTEGRATION_REFERENCE.md)

**For quick commands:**
- [008_SYSTEM_INTEGRATION_QUICK_START.md](008_SYSTEM_INTEGRATION_QUICK_START.md)

**For architecture:**
- [001_ARCHITECTURE_OVERVIEW.md](001_ARCHITECTURE_OVERVIEW.md)

**For subsystems:**
- 100_PERSON_RECOGNITION_REFERENCE.md
- 200_SPEECH_SYSTEM_REFERENCE.md
- 300_GESTURE_SYSTEM_OVERVIEW.md
- 250_PERSON_MANAGEMENT_SYSTEM_REFERENCE.md (PersonRegistry)

---

**Document Version:** 1.0  
**Date:** December 18, 2025  
**Purpose:** Troubleshooting and debug procedures  
**Status:** Based on actual debugging session December 18, 2025

