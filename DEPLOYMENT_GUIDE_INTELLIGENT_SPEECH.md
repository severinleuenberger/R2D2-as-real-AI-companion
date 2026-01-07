# Intelligent Speech Node - Deployment Guide

## ✅ Implementation Complete

All code changes have been implemented and built successfully. This guide covers the final deployment steps that require manual intervention.

---

## 📋 What Was Implemented

### 1. **New Node: `intelligent_speech_node`**
   - Location: `ros2_ws/src/r2d2_speech/r2d2_speech_ros/intelligent_speech_node.py`
   - Uses OpenAI Realtime API (same as Fast Mode)
   - Separate service endpoints: `/r2d2/speech/intelligent/start_session`, `/r2d2/speech/intelligent/stop_session`
   - Separate topics: `/r2d2/speech/intelligent/voice_activity`, `/r2d2/speech/intelligent/session_status`
   - Voice: "nova" (bright, clear AI assistant)
   - Instructions: Intelligent AI assistant personality

### 2. **Launch File**
   - Location: `ros2_ws/src/r2d2_speech/launch/intelligent_speech_node.launch.py`
   - Same structure as Fast Mode launch file
   - Loads shared `speech_params.yaml` configuration

### 3. **Configuration Updates**
   - Location: `ros2_ws/src/r2d2_speech/config/speech_params.yaml`
   - Added `intelligent_realtime_voice: 'nova'`
   - Added `intelligent_instructions` with AI assistant personality

### 4. **Gesture Intent Node Updates**
   - Location: `ros2_ws/src/r2d2_gesture/r2d2_gesture/gesture_intent_node.py`
   - **Removed:** REST API service clients (`process_turn`, complex turn loop)
   - **Added:** Realtime API service clients for intelligent mode
   - **Added:** VAD tracking for intelligent mode (same as Fast Mode)
   - **Added:** Session status subscription for intelligent mode
   - **Simplified:** State management - now matches Fast Mode pattern
   - Open-hand gesture now triggers Realtime API instead of REST API

### 5. **ROS2 Bridge Updates**
   - Location: `ros2_ws/src/r2d2_speech/r2d2_speech_ros/ros2_bridge.py`
   - Added `topic_prefix` parameter to all publisher classes
   - Default prefix: `/r2d2/speech` (backwards compatible)
   - Intelligent mode uses: `/r2d2/speech/intelligent`

### 6. **Systemd Service**
   - Location: `/home/severin/dev/r2d2/r2d2-intelligent-speech.service`
   - Ready to be installed to `/etc/systemd/system/`

### 7. **Documentation Updates**
   - Updated `000_UX_AND_FUNCTIONS.md` - Two-mode operation
   - Updated `204_SPEECH_SYSTEM_VOICE_CONFIGURATION.md` - Intelligent Mode, rest_speech_node note

### 8. **Package Build**
   - ✅ Both packages built successfully without errors
   - ✅ No linter errors

---

## 🚀 Final Deployment Steps (Manual)

### Step 1: Install Systemd Service

```bash
cd /home/severin/dev/r2d2
sudo cp r2d2-intelligent-speech.service /etc/systemd/system/
sudo systemctl daemon-reload
```

### Step 2: Enable and Start Service

```bash
# Enable service to start on boot
sudo systemctl enable r2d2-intelligent-speech.service

# Start service now
sudo systemctl start r2d2-intelligent-speech.service

# Check status
sudo systemctl status r2d2-intelligent-speech.service
```

### Step 3: Restart Gesture Intent Node

Since the gesture_intent_node has been updated, restart it to use the new code:

```bash
sudo systemctl restart r2d2-gesture-intent.service
sudo systemctl status r2d2-gesture-intent.service
```

### Step 4: Verify Services Running

```bash
# Check all R2D2 speech services
systemctl list-units --type=service | grep r2d2-speech

# Expected output (intelligent-speech should be active):
# r2d2-speech-node.service              loaded active running R2D2 Speech Node Service
# r2d2-intelligent-speech.service       loaded active running R2D2 Intelligent Speech Node Service (Open Hand Gesture)
```

---

## 🧪 Testing Checklist

### Pre-Test Checks
- [ ] Verify gesture intent service is running: `sudo systemctl status r2d2-gesture-intent.service`
- [ ] Verify fast speech service is running: `sudo systemctl status r2d2-speech-node.service`
- [ ] Verify intelligent speech service is running: `sudo systemctl status r2d2-intelligent-speech.service`
- [ ] Check for errors in logs: `sudo journalctl -u r2d2-intelligent-speech.service -f`

### Test 1: Fast Mode (Index Finger) - Should Work Unchanged
- [ ] Stand in front of camera, wait for LED ON (person recognized)
- [ ] Show index finger pointing up (☝️)
- [ ] Hear acknowledgment beep (~350ms)
- [ ] Hear ready beep (~750ms)
- [ ] Speak to test conversation
- [ ] Verify R2-D2 personality (chatty, efficient)
- [ ] Verify "sage" voice
- [ ] Make fist gesture to stop (two-stage)
- [ ] Hear warning beep, continue holding
- [ ] Hear stop beep, session ends

### Test 2: Intelligent Mode (Open Hand) - NEW
- [ ] Stand in front of camera, wait for LED ON
- [ ] Show open hand (🖐️)
- [ ] Hear acknowledgment beep (~350ms)
- [ ] Hear ready beep (~750ms)
- [ ] Speak to test conversation
- [ ] Verify Intelligent AI assistant personality (helpful, professional)
- [ ] Verify "nova" voice (bright, clear)
- [ ] Make fist gesture to stop (two-stage)
- [ ] Hear warning beep, continue holding
- [ ] Hear stop beep, session ends

### Test 3: Service Isolation
- [ ] Verify both services can run simultaneously (check systemctl status)
- [ ] Verify gestures gate correctly (can't start Fast Mode while Intelligent Mode active)
- [ ] Verify fist stops appropriate mode

### Test 4: Topics and Logging
```bash
# Monitor intelligent mode topics
ros2 topic echo /r2d2/speech/intelligent/session_status
ros2 topic echo /r2d2/speech/intelligent/voice_activity
ros2 topic echo /r2d2/speech/intelligent/user_transcript
ros2 topic echo /r2d2/speech/intelligent/assistant_transcript

# Check logs
sudo journalctl -u r2d2-intelligent-speech.service -f
sudo journalctl -u r2d2-gesture-intent.service -f
```

---

## 🔧 Troubleshooting

### Service Won't Start
```bash
# Check detailed logs
sudo journalctl -u r2d2-intelligent-speech.service -n 50 --no-pager

# Check if port conflicts exist
sudo netstat -tulpn | grep LISTEN

# Verify ROS2 workspace sourced correctly
source /home/severin/dev/r2d2/ros2_ws/install/setup.bash
ros2 pkg list | grep r2d2_speech
```

### Gesture Not Triggering
```bash
# Check gesture intent node logs
sudo journalctl -u r2d2-gesture-intent.service -f

# Verify person status is RED
ros2 topic echo /r2d2/perception/person_status

# Verify gesture recognition
ros2 topic echo /r2d2/perception/gesture_event
```

### OpenAI Connection Issues
```bash
# Check API key is loaded
cat ~/.r2d2/.env | grep OPENAI_API_KEY

# Verify network connectivity
ping api.openai.com

# Check service logs for connection errors
sudo journalctl -u r2d2-intelligent-speech.service | grep -i "error\|failed"
```

### Service Clients Not Connecting
```bash
# List available services
ros2 service list | grep intelligent

# Expected services:
# /r2d2/speech/intelligent/start_session
# /r2d2/speech/intelligent/stop_session

# Manually test service call
ros2 service call /r2d2/speech/intelligent/start_session std_srvs/srv/Trigger
```

---

## 📊 Architecture Summary

### Current State (After Implementation)

```
Index Finger (☝️)  ──→  speech_node (Realtime API)
                         ├─ Voice: "sage"
                         ├─ Personality: R2-D2 astromech (chatty)
                         └─ Topics: /r2d2/speech/*

Open Hand (🖐️)    ──→  intelligent_speech_node (Realtime API)
                         ├─ Voice: "nova"
                         ├─ Personality: Intelligent AI assistant
                         └─ Topics: /r2d2/speech/intelligent/*

rest_speech_node (REST API)
 └─ Available but disabled (can be reactivated if needed)
```

### Key Differences Between Modes

| Aspect | Fast Mode | Intelligent Mode |
|--------|-----------|------------------|
| Gesture | Index Finger ☝️ | Open Hand 🖐️ |
| Voice | sage | nova |
| Personality | R2-D2 astromech | AI assistant |
| API | Realtime | Realtime |
| Latency | 700-1200ms | 700-1200ms |
| Topics | /r2d2/speech/* | /r2d2/speech/intelligent/* |

---

## 🔄 Rollback Plan (If Needed)

If issues arise and you need to revert to REST API mode:

```bash
# 1. Stop intelligent speech service
sudo systemctl stop r2d2-intelligent-speech.service
sudo systemctl disable r2d2-intelligent-speech.service

# 2. Revert gesture_intent_node changes
cd /home/severin/dev/r2d2/ros2_ws/src/r2d2_gesture/r2d2_gesture
git checkout gesture_intent_node.py

# 3. Re-enable rest_speech_node service (if installed)
sudo systemctl enable r2d2-rest-speech-node.service
sudo systemctl start r2d2-rest-speech-node.service

# 4. Rebuild and restart
cd /home/severin/dev/r2d2/ros2_ws
colcon build --packages-select r2d2_gesture
sudo systemctl restart r2d2-gesture-intent.service
```

---

## 📝 Notes

- **rest_speech_node preservation:** The REST API implementation remains in the codebase and can be reactivated by re-enabling the systemd service and updating gesture routing.
- Both Realtime API services share the same underlying implementation, ensuring identical behavior.
- The only differences are instructions and voice, which are configuration parameters.
- Gesture intent node gates prevent both services from being active simultaneously.

---

## ✅ Success Criteria

Implementation is successful when:
1. ✅ All services start without errors
2. ✅ Index finger triggers Fast Mode (R2-D2 personality, sage voice)
3. ✅ Open hand triggers Intelligent Mode (AI assistant, nova voice)
4. ✅ Fist stops both modes correctly
5. ✅ Topics publish data correctly
6. ✅ VAD timeouts work (30s silence)
7. ✅ No service conflicts or crashes

---

**Deployment Guide Version:** 1.0  
**Date:** January 7, 2026  
**Implementation Status:** ✅ Complete - Ready for Deployment

