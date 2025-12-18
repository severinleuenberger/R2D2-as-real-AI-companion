# R2D2 System Integration - Quick Start Guide

**Document ID:** 008_SYSTEM_INTEGRATION_QUICK_START.md  
**Date:** December 18, 2025  
**Purpose:** Quick reference for common system integration tasks  
**For:** Operators, testers, developers doing daily work

---

## Quick Status Check

```bash
# Check all services
systemctl is-active r2d2-camera-perception r2d2-audio-notification \
  r2d2-gesture-intent r2d2-speech-node
# EXPECT: active active active active

# Check all nodes
cd ~/dev/r2d2/ros2_ws && source install/setup.bash
ros2 node list | grep -E "audio|gesture|speech|image|camera"
# EXPECT: 5+ nodes listed
```

---

## Quick Parameter Verification

```bash
cd ~/dev/r2d2/ros2_ws && source install/setup.bash

# Critical parameters
ros2 param get /speech_node auto_start
# EXPECT: False (waits for gesture, not auto-start)

ros2 param get /audio_notification_node red_status_timeout_seconds
# EXPECT: 15.0 (simplified RED timer)

ros2 param get /gesture_intent_node speaking_protection_seconds
# EXPECT: 35.0 (conversation protection)
```

---

## Quick Topic Monitoring

```bash
cd ~/dev/r2d2/ros2_ws && source install/setup.bash

# Check person recognition
ros2 topic echo /r2d2/perception/person_id --once
# EXPECT: "severin" (when in front of camera)

# Check person status
ros2 topic echo /r2d2/audio/person_status --once
# EXPECT: {"status": "red",...} (when recognized)

# Check gesture events
timeout 5 ros2 topic echo /r2d2/perception/gesture_event
# Make a gesture, EXPECT: "index_finger_up" or "fist"

# Check session status
ros2 topic echo /r2d2/speech/session_status --once
# EXPECT: {"status": "inactive"} when idle
# EXPECT: {"status": "connected"} when speaking
```

---

## Quick Gesture Test

**Prerequisites:** Stand in front of camera, LED should be ON

```bash
# Monitor gesture events and actions
cd ~/dev/r2d2/ros2_ws && source install/setup.bash
ros2 topic echo /r2d2/perception/gesture_event &
ros2 topic echo /r2d2/speech/session_status &

# Now make gestures:
# 1. Raise index finger ☝️
#    EXPECT: "index_finger_up" event
#    EXPECT: session_status = "connected"
#    EXPECT: "Start" beep heard

# 2. Make fist ✊
#    EXPECT: "fist" event
#    EXPECT: session_status = "disconnected"
#    EXPECT: "Stop" beep heard

# Kill background jobs
pkill -f "ros2 topic echo"
```

---

## Quick Restart Commands

```bash
# Restart single service
sudo systemctl restart r2d2-gesture-intent

# Restart all R2D2 services
sudo systemctl restart r2d2-camera-perception \
  r2d2-audio-notification r2d2-gesture-intent r2d2-speech-node

# Rebuild packages after code changes
cd ~/dev/r2d2/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select r2d2_gesture r2d2_speech r2d2_audio --symlink-install

# Then restart services
sudo systemctl restart r2d2-gesture-intent r2d2-speech-node r2d2-audio-notification
```

---

## Quick Log Monitoring

```bash
# Monitor gesture intent (for gesture actions and SPEAKING state)
sudo journalctl -u r2d2-gesture-intent -f | grep --line-buffered -E "Index finger|Fist|SPEAKING|Session"

# Monitor audio notification (for RED/BLUE transitions)
sudo journalctl -u r2d2-audio-notification -f | grep --line-buffered -E "recognized|lost|Timer reset"

# Monitor speech node (for session start/stop)
sudo journalctl -u r2d2-speech-node -f | grep --line-buffered -E "Service:|Speech system|Connected"

# Monitor all relevant logs
sudo journalctl -f -u r2d2-gesture-intent -u r2d2-audio-notification -u r2d2-speech-node
```

---

## Quick Diagnosis

### No Beeps

```bash
# Check audio playback works
ffplay -nodisp -autoexit -v quiet \
  ~/dev/r2d2/ros2_ws/src/r2d2_audio/r2d2_audio/assets/audio/Voicy_R2-D2\ -\ 2.mp3
# Should hear "Hello!" beep

# Check volume
ros2 param get /audio_notification_node audio_volume
# If too low (< 0.1), increase:
ros2 param set /audio_notification_node audio_volume 0.5
```

### Gestures Not Working

```bash
# Check person_status is RED
ros2 topic echo /r2d2/audio/person_status --once | grep status
# EXPECT: "red"

# Check gestures are detected
timeout 5 ros2 topic echo /r2d2/perception/gesture_event
# Make gesture, EXPECT: event published

# Check gesture_intent logs for blocking
sudo journalctl -u r2d2-gesture-intent --since "1 minute ago" | grep "ignored"
# If "person_status" → Not recognized (stand in camera)
# If "already active" → Make fist first to stop
# If "cooldown" → Wait 5 seconds
```

### Speech Not Starting

```bash
# Check speech_node is Active
ros2 lifecycle get /speech_node
# EXPECT: "active"

# Check services available
ros2 service list | grep speech
# EXPECT: /r2d2/speech/start_session and stop_session

# Test service manually
ros2 service call /r2d2/speech/start_session std_srvs/srv/Trigger
# EXPECT: Response within 5s, success=True or "Already running"
```

---

## Quick State Verification

```bash
# Check RED status timer
sudo journalctl -u r2d2-audio-notification -f | grep "Timer reset"
# EXPECT: Messages every ~154ms while recognized

# Check SPEAKING state
sudo journalctl -u r2d2-gesture-intent -f | grep "SPEAKING"
# After index finger gesture, EXPECT: "Entered SPEAKING state"
# After fist gesture, EXPECT: "Exited SPEAKING state"
```

---

## Key Numbers to Remember

| Metric | Value | What it means |
|--------|-------|---------------|
| **15 seconds** | RED status timeout | Must see face within 15s or RED→BLUE |
| **35 seconds** | SPEAKING protection | Must be non-RED for 35s consecutive to auto-stop |
| **5 seconds** | Start gesture cooldown | Wait 5s after start before next start gesture |
| **3 seconds** | Stop gesture cooldown | Wait 3s after stop before next stop gesture |
| **6.5 Hz** | person_id rate | Face recognition publishes ~6-7 times/second |
| **10 Hz** | person_status rate | Status published 10 times/second |

---

## Quick Recovery Procedures

### System Not Responding

```bash
# Full restart
sudo systemctl restart r2d2-camera-perception
sleep 3
sudo systemctl restart r2d2-audio-notification
sleep 2
sudo systemctl restart r2d2-gesture-intent
sudo systemctl restart r2d2-speech-node

# Wait 10 seconds, then verify
systemctl is-active r2d2-*
```

### First Gesture Blocked

```bash
# If "already active" message:
# 1. Make fist gesture to stop
# 2. Wait 5 seconds
# 3. Raise finger to start

# Or check auto_start:
ros2 param get /speech_node auto_start
# Should be: False
# If True, need to rebuild and restart
```

---

## Quick Reference Links

**For comprehensive details:**
- [007_SYSTEM_INTEGRATION_REFERENCE.md](007_SYSTEM_INTEGRATION_REFERENCE.md) - Complete system integration
- [001_ARCHITECTURE_OVERVIEW.md](001_ARCHITECTURE_OVERVIEW.md) - System architecture

**For subsystem details:**
- [100_PERSON_RECOGNITION_REFERENCE.md](100_PERSON_RECOGNITION_REFERENCE.md) - Face recognition
- [200_SPEECH_SYSTEM_REFERENCE.md](200_SPEECH_SYSTEM_REFERENCE.md) - Speech-to-speech
- [300_GESTURE_SYSTEM_OVERVIEW.md](300_GESTURE_SYSTEM_OVERVIEW.md) - Gesture control

**For troubleshooting:**
- [009_SYSTEM_INTEGRATION_TROUBLESHOOTING.md](009_SYSTEM_INTEGRATION_TROUBLESHOOTING.md) - Debug procedures

---

**Document Version:** 1.0  
**Date:** December 18, 2025  
**For:** Quick daily reference  
**See Also:** 007 (comprehensive reference), 009 (troubleshooting)

