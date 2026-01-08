# Intelligent Speech Mode - Test Protocol

**Feature:** Open-hand gesture triggered intelligent AI assistant speech service  
**Implementation Date:** January 7, 2026  
**Status:** ⏳ AWAITING TESTING  
**Test Protocol Version:** 1.0

---

## 📋 Test Overview

This protocol validates the new Intelligent Mode speech service triggered by the open-hand gesture. The implementation uses OpenAI Realtime API (identical to Fast Mode) but with a professional AI assistant personality and "nova" voice.

**What Changed:**
- New `intelligent_speech_node` using Realtime API (replaces REST API for open-hand gesture)
- Updated `gesture_intent_node` to route open-hand to Realtime service
- New configuration: `intelligent_realtime_voice` (nova) and `intelligent_instructions`
- REST API node (`rest_speech_node`) preserved but disabled

---

## ⚙️ Pre-Test Setup

### Required Services

```bash
# Verify all required services are installed and enabled
systemctl list-units "r2d2-*" --type=service | grep speech

# Expected output:
# r2d2-speech-node.service           loaded active running
# r2d2-intelligent-speech.service    loaded active running
```

### Installation (If Not Done)

```bash
# Install intelligent speech service
cd /home/severin/dev/r2d2
sudo cp r2d2-intelligent-speech.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable r2d2-intelligent-speech.service
sudo systemctl start r2d2-intelligent-speech.service

# Restart gesture intent node (uses updated code)
sudo systemctl restart r2d2-gesture-intent.service

# Verify status
systemctl status r2d2-intelligent-speech.service
systemctl status r2d2-gesture-intent.service
```

### Pre-Test Verification

```bash
# All services running?
sudo systemctl status r2d2-speech-node.service
sudo systemctl status r2d2-intelligent-speech.service
sudo systemctl status r2d2-gesture-intent.service
sudo systemctl status r2d2-image-listener.service

# Check for startup errors
sudo journalctl -u r2d2-intelligent-speech.service -n 50 --no-pager
sudo journalctl -u r2d2-gesture-intent.service -n 50 --no-pager

# Verify ROS2 services exist
ros2 service list | grep intelligent
# Expected:
# /r2d2/speech/intelligent/start_session
# /r2d2/speech/intelligent/stop_session

# Verify ROS2 topics exist
ros2 topic list | grep intelligent
# Expected:
# /r2d2/speech/intelligent/assistant_transcript
# /r2d2/speech/intelligent/commands
# /r2d2/speech/intelligent/assistant_prompt
# /r2d2/speech/intelligent/session_status
# /r2d2/speech/intelligent/user_transcript
# /r2d2/speech/intelligent/voice_activity
```

**✅ Pre-Test Checklist:**
- [ ] All required services are `active (running)`
- [ ] No errors in startup logs
- [ ] ROS2 services are available
- [ ] ROS2 topics are available
- [ ] OpenAI API key is configured (`cat ~/.r2d2/.env | grep OPENAI_API_KEY`)

---

## 🧪 Test Suite

### Test 1: Fast Mode Baseline (Regression Test)

**Purpose:** Verify Fast Mode (index finger) still works after gesture_intent_node changes

**Prerequisites:**
- [ ] Stand 1-2 meters from camera
- [ ] Ensure good lighting
- [ ] Wait for RED LED (person recognized)

**Procedure:**
1. Show index finger pointing up (☝️)
2. Wait for acknowledgment beep (~350ms)
3. Wait for ready beep (~750ms)
4. Say: "Hello R2, are you working correctly?"
5. Listen for response
6. Make fist gesture (hold 1.5s)
7. Wait for warning beep
8. Continue holding fist (1.5s more)
9. Wait for stop beep

**Expected Results:**
- [ ] Index finger triggers Fast Mode
- [ ] Acknowledgment beep plays (~350ms after gesture)
- [ ] Ready beep plays (~750ms after acknowledgment)
- [ ] R2-D2 personality responds (chatty, friendly, efficient)
- [ ] Voice is "sage" (slightly robotic)
- [ ] Response mentions "R2" or uses droid-like language
- [ ] Fist warning beep plays after ~1.5s
- [ ] Fist stop beep plays after additional ~1.5s
- [ ] Session stops cleanly

**Log Verification:**
```bash
# Monitor Fast Mode logs during test
sudo journalctl -u r2d2-speech-node.service -f

# Expected log entries:
# "Service: start_session (warm start)"
# "✓ Streaming started"
# "VAD: User speaking"
# "Published user transcript: Hello R2..."
# "Published assistant transcript: [response]"
# "Service: stop_session"
# "✓ Streaming stopped"
```

**Result:** ⬜ PASS / ⬜ FAIL / ⬜ SKIPPED  
**Notes:**

---

### Test 2: Intelligent Mode Basic Functionality

**Purpose:** Verify open-hand gesture triggers Intelligent Mode with AI assistant personality

**Prerequisites:**
- [ ] Stand 1-2 meters from camera
- [ ] Wait for RED LED (person recognized)
- [ ] No active speech session running

**Procedure:**
1. Show open hand (🖐️) with palm facing camera
2. Wait for acknowledgment beep (~350ms)
3. Wait for ready beep (~750ms)
4. Say: "Hello, can you help me with a programming question?"
5. Listen for response
6. Make fist gesture (hold 1.5s)
7. Wait for warning beep
8. Continue holding fist (1.5s more)
9. Wait for stop beep

**Expected Results:**
- [ ] Open hand triggers Intelligent Mode
- [ ] Acknowledgment beep plays (~350ms after gesture)
- [ ] Ready beep plays (~750ms after acknowledgment)
- [ ] AI assistant personality responds (professional, helpful, clear)
- [ ] Voice is "nova" (bright, clear, NOT robotic)
- [ ] Response is professional (no [beeps], no R2-D2 character)
- [ ] Response addresses the question helpfully
- [ ] Fist warning beep plays after ~1.5s
- [ ] Fist stop beep plays after additional ~1.5s
- [ ] Session stops cleanly

**Log Verification:**
```bash
# Monitor Intelligent Mode logs during test
sudo journalctl -u r2d2-intelligent-speech.service -f

# Expected log entries:
# "Service: start_session (Intelligent Mode)"
# "✓ Streaming started (Intelligent Mode)"
# "Intelligent VAD: User speaking"
# "✓ Connected to OpenAI API"
# "✓ Session configured (Intelligent Mode)"

# Also check gesture intent logs
sudo journalctl -u r2d2-gesture-intent.service -f

# Expected:
# "🖐️  Open hand detected → Starting Intelligent Mode conversation"
# "📡 Intelligent session status: connected"
# "🎤 Intelligent VAD: User speaking"
# "✊ Fist confirmed → Stopping Intelligent Mode conversation"
```

**Result:** ⬜ PASS / ⬜ FAIL / ⬜ SKIPPED  
**Notes:**

---

### Test 3: Voice and Personality Verification

**Purpose:** Confirm distinct voices and personalities between modes

**Test 3A: Fast Mode Voice/Personality**

**Procedure:**
1. Trigger Fast Mode (index finger)
2. Say: "R2, tell me about yourself"
3. Listen carefully to voice and content

**Expected:**
- [ ] Voice sounds robotic/synthetic (sage)
- [ ] Response uses R2-D2 character (may include [beeps])
- [ ] Short, punchy sentences
- [ ] Friendly, chatty tone
- [ ] May reference being a droid/astromech

**Test 3B: Intelligent Mode Voice/Personality**

**Procedure:**
1. Stop Fast Mode (fist)
2. Wait 5 seconds (cooldown)
3. Trigger Intelligent Mode (open hand)
4. Say: "Hello, tell me about yourself"
5. Listen carefully to voice and content

**Expected:**
- [ ] Voice sounds human-like, clear, bright (nova)
- [ ] Response is professional AI assistant
- [ ] NO [beeps], [chirps], or droid language
- [ ] Thoughtful, helpful tone
- [ ] Clear articulation and pacing
- [ ] Describes being an AI assistant (not a droid)

**Comparison:**
- [ ] Voices are distinctly different
- [ ] Personalities are clearly distinct
- [ ] "sage" voice is more robotic than "nova"
- [ ] "nova" voice is brighter and clearer

**Result:** ⬜ PASS / ⬜ FAIL / ⬜ SKIPPED  
**Notes:**

---

### Test 4: Service Isolation (Gating)

**Purpose:** Verify gesture gating prevents mode conflicts

**Test 4A: Intelligent Mode Blocks Fast Mode**

**Procedure:**
1. Trigger Intelligent Mode (open hand)
2. Immediately try to trigger Fast Mode (index finger)
3. Observe behavior

**Expected:**
- [ ] Fast Mode gesture is rejected
- [ ] Log shows: "❌ Index finger ignored: intelligent mode session active"
- [ ] Intelligent Mode continues uninterrupted
- [ ] No acknowledgment beep for index finger

**Test 4B: Fast Mode Blocks Intelligent Mode**

**Procedure:**
1. Stop Intelligent Mode (fist)
2. Wait 5 seconds
3. Trigger Fast Mode (index finger)
4. Immediately try to trigger Intelligent Mode (open hand)
5. Observe behavior

**Expected:**
- [ ] Intelligent Mode gesture is rejected
- [ ] Log shows: "❌ Open hand ignored: fast mode session active"
- [ ] Fast Mode continues uninterrupted
- [ ] No acknowledgment beep for open hand

**Result:** ⬜ PASS / ⬜ FAIL / ⬜ SKIPPED  
**Notes:**

---

### Test 5: Multi-Turn Conversations

**Purpose:** Verify context is maintained across multiple exchanges

**Test 5A: Intelligent Mode Multi-Turn**

**Procedure:**
1. Trigger Intelligent Mode (open hand)
2. Say: "I'm working on a Python project"
3. Wait for response
4. Follow up: "What's the best way to handle errors?"
5. Wait for response
6. Follow up: "Can you give me an example?"
7. Wait for response
8. Stop (fist)

**Expected:**
- [ ] Each response acknowledges previous context
- [ ] Second question assumes Python context
- [ ] Third question gets Python error handling example
- [ ] No need to re-trigger between questions
- [ ] Context flows naturally across turns

**Test 5B: Fast Mode Multi-Turn (Baseline)**

**Procedure:**
1. Trigger Fast Mode (index finger)
2. Say: "What's your favorite mission?"
3. Wait for response
4. Follow up: "Have you been to dangerous places?"
5. Wait for response
6. Stop (fist)

**Expected:**
- [ ] Responses maintain R2-D2 character
- [ ] Context flows across turns
- [ ] R2-D2 personality consistent

**Result:** ⬜ PASS / ⬜ FAIL / ⬜ SKIPPED  
**Notes:**

---

### Test 6: VAD Timeout Behavior

**Purpose:** Verify Voice Activity Detection auto-stops after silence

**Test 6A: Intelligent Mode VAD Timeout**

**Procedure:**
1. Trigger Intelligent Mode (open hand)
2. Say: "Hello"
3. Wait for response
4. Stay completely silent for 30+ seconds
5. Observe auto-stop

**Expected:**
- [ ] Session continues normally during conversation
- [ ] After ~30s of silence, session auto-stops
- [ ] Log shows: "Intelligent VAD: No speech detected for 30s"
- [ ] Stop beep plays
- [ ] Service status shows "disconnected"

**Test 6B: VAD Pause During Speaking**

**Procedure:**
1. Trigger Intelligent Mode (open hand)
2. Start speaking and continue for 60+ seconds
3. Observe timeout does NOT trigger

**Expected:**
- [ ] Timeout counter pauses while user speaks
- [ ] Session does NOT auto-stop during continuous speech
- [ ] Log shows: "Intelligent VAD: User speaking (silence timer paused)"

**Result:** ⬜ PASS / ⬜ FAIL / ⬜ SKIPPED  
**Notes:**

---

### Test 7: Cooldown Behavior

**Purpose:** Verify gesture cooldown prevents rapid re-triggering

**Procedure:**
1. Trigger Intelligent Mode (open hand)
2. Immediately stop (fist - hold for full 3s)
3. Immediately try to re-trigger (open hand)
4. Observe behavior

**Expected:**
- [ ] Second open-hand gesture is rejected
- [ ] Log shows: "❌ Open hand ignored: cooldown (Xs < 3.0s)"
- [ ] No acknowledgment beep for second gesture
- [ ] Wait 3+ seconds, then gesture works

**Result:** ⬜ PASS / ⬜ FAIL / ⬜ SKIPPED  
**Notes:**

---

### Test 8: ROS2 Topics Publishing

**Purpose:** Verify all topics publish correct data

**Procedure:**
```bash
# Terminal 1: Monitor user transcript
ros2 topic echo /r2d2/speech/intelligent/user_transcript

# Terminal 2: Monitor assistant transcript
ros2 topic echo /r2d2/speech/intelligent/assistant_transcript

# Terminal 3: Monitor session status
ros2 topic echo /r2d2/speech/intelligent/session_status

# Terminal 4: Monitor VAD
ros2 topic echo /r2d2/speech/intelligent/voice_activity

# Perform test:
1. Trigger Intelligent Mode (open hand)
2. Say: "Testing topic publishing"
3. Wait for response
4. Stop (fist)
```

**Expected Results:**
- [ ] `session_status` shows: `{"status": "connected", ...}` on start
- [ ] `user_transcript` shows: "Testing topic publishing"
- [ ] `assistant_transcript` shows AI response
- [ ] `voice_activity` toggles: `{"state": "speaking"}` and `{"state": "silent"}`
- [ ] `session_status` shows: `{"status": "disconnected"}` on stop
- [ ] All timestamps are reasonable

**Result:** ⬜ PASS / ⬜ FAIL / ⬜ SKIPPED  
**Notes:**

---

### Test 9: Error Recovery

**Purpose:** Verify system handles errors gracefully

**Test 9A: Network Interruption Simulation**

**Procedure:**
1. Trigger Intelligent Mode (open hand)
2. Say: "Test message"
3. **Simulate network issue:** Disconnect WiFi briefly
4. Observe behavior
5. Reconnect WiFi

**Expected:**
- [ ] Service detects connection loss
- [ ] Log shows connection error
- [ ] Service attempts reconnection OR cleanly stops
- [ ] No crash or hang
- [ ] Can re-trigger after network restored

**Test 9B: Service Restart During Session**

**Procedure:**
1. Trigger Intelligent Mode (open hand)
2. Say: "Testing restart"
3. **In another terminal:** `sudo systemctl restart r2d2-intelligent-speech.service`
4. Observe behavior

**Expected:**
- [ ] Session terminates cleanly
- [ ] Gesture intent node detects session disconnect
- [ ] No zombie processes
- [ ] Service restarts successfully
- [ ] Can re-trigger after restart

**Result:** ⬜ PASS / ⬜ FAIL / ⬜ SKIPPED  
**Notes:**

---

### Test 10: System Reboot Persistence

**Purpose:** Verify services survive reboot and auto-start correctly

**⚠️ CRITICAL TEST - Required for production**

**Procedure:**
```bash
# Verify services are enabled
systemctl is-enabled r2d2-intelligent-speech.service  # Must show: enabled
systemctl is-enabled r2d2-gesture-intent.service      # Must show: enabled

# Reboot system
sudo reboot

# After reboot, check services
systemctl status r2d2-intelligent-speech.service
systemctl status r2d2-gesture-intent.service

# Check startup logs
sudo journalctl -u r2d2-intelligent-speech.service -b -n 50
sudo journalctl -u r2d2-gesture-intent.service -b -n 50

# Functional test
# Try triggering Intelligent Mode (open hand)
```

**Expected Results:**
- [ ] Both services auto-start after reboot
- [ ] Services show `active (running)` status
- [ ] No errors in startup logs
- [ ] Intelligent Mode triggers successfully post-reboot
- [ ] Fast Mode still works (regression check)

**Result:** ⬜ PASS / ⬜ FAIL / ⬜ SKIPPED  
**Notes:**

---

## 📊 Test Results Summary

**Test Date:** _______________  
**Tester:** _______________  
**System Version:** _______________

| Test # | Test Name | Result | Critical? | Notes |
|--------|-----------|--------|-----------|-------|
| 1 | Fast Mode Baseline | ⬜ PASS ⬜ FAIL | ✅ | |
| 2 | Intelligent Mode Basic | ⬜ PASS ⬜ FAIL | ✅ | |
| 3 | Voice/Personality | ⬜ PASS ⬜ FAIL | ✅ | |
| 4 | Service Isolation | ⬜ PASS ⬜ FAIL | ✅ | |
| 5 | Multi-Turn Conversations | ⬜ PASS ⬜ FAIL | ⚠️ | |
| 6 | VAD Timeout | ⬜ PASS ⬜ FAIL | ⚠️ | |
| 7 | Cooldown Behavior | ⬜ PASS ⬜ FAIL | ⚠️ | |
| 8 | ROS2 Topics | ⬜ PASS ⬜ FAIL | ⚠️ | |
| 9 | Error Recovery | ⬜ PASS ⬜ FAIL | ⚠️ | |
| 10 | Reboot Persistence | ⬜ PASS ⬜ FAIL | ✅ | |

**Overall Result:** ⬜ PASS / ⬜ FAIL / ⬜ INCOMPLETE

**Critical Tests (Must Pass):** 1, 2, 3, 4, 10  
**Important Tests (Should Pass):** 5, 6, 7, 8, 9

---

## 🐛 Known Issues / Limitations

*(Document any issues found during testing)*

**Issue #1:**  
**Severity:** ⬜ Critical ⬜ Major ⬜ Minor  
**Description:**  
**Workaround:**  
**Status:** ⬜ Open ⬜ Fixed ⬜ Deferred

---

## 📝 Post-Test Actions

### If All Tests Pass:
- [ ] Update `000_INTERNAL_AGENT_NOTES.md` with any new patterns
- [ ] Mark feature as "✅ OPERATIONAL" in documentation
- [ ] Update this protocol status to "✅ COMPLETE"
- [ ] Archive this test protocol for future reference

### If Tests Fail:
- [ ] Document all failures in "Known Issues" section
- [ ] Create issue tickets for each critical failure
- [ ] Determine if rollback is needed
- [ ] Re-test after fixes applied

---

## 🔧 Troubleshooting Reference

### Service Not Starting

```bash
# Check detailed error
sudo journalctl -u r2d2-intelligent-speech.service -n 100 --no-pager

# Common causes:
# - Missing ROS2 workspace sourcing
# - Wrong Python environment
# - OpenAI API key not loaded
# - Port already in use
```

### Gesture Not Triggering

```bash
# Verify person status
ros2 topic echo /r2d2/perception/person_status
# Must show: RED

# Verify gesture recognition
ros2 topic echo /r2d2/perception/gesture_event
# Should show open_hand when gesture is made

# Check gesture intent logs
sudo journalctl -u r2d2-gesture-intent.service -f
```

### Voice/Personality Wrong

```bash
# Verify config loaded correctly
ros2 param get /intelligent_speech_node intelligent_realtime_voice
# Must show: nova

ros2 param get /intelligent_speech_node intelligent_instructions
# Must show AI assistant instructions (not R2-D2)
```

### VAD Not Working

```bash
# Monitor VAD topic
ros2 topic echo /r2d2/speech/intelligent/voice_activity

# Check OpenAI connection
sudo journalctl -u r2d2-intelligent-speech.service | grep -i "vad\|voice"
```

---

## 📖 Related Documentation

- [DEPLOYMENT_GUIDE_INTELLIGENT_SPEECH.md](DEPLOYMENT_GUIDE_INTELLIGENT_SPEECH.md) - Installation guide
- [200_SPEECH_SYSTEM_REFERENCE.md](200_SPEECH_SYSTEM_REFERENCE.md) - Architecture reference
- [204_SPEECH_SYSTEM_VOICE_CONFIGURATION.md](204_SPEECH_SYSTEM_VOICE_CONFIGURATION.md) - Voice and personality configuration
- [000_UX_AND_FUNCTIONS.md](000_UX_AND_FUNCTIONS.md) - User experience documentation

---

**Test Protocol Version:** 1.0  
**Created:** January 7, 2026  
**Status:** ⏳ AWAITING TESTING  
**Next Action:** Execute this test protocol and complete all test cases


