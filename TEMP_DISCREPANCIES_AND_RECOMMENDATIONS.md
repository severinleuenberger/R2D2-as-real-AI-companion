# R2D2 System - Documentation vs Implementation Discrepancies

**Date:** December 21, 2025  
**Analysis Type:** Logic Redesign & Code Verification  
**Status:** ⚠️ 6 CRITICAL DISCREPANCIES / IMPROVEMENTS FOUND

---

## Complete System Flow with Proposed Robust RED Entry Logic

```mermaid
sequenceDiagram
    participant OAK as OAK-D Camera
    participant ImgList as image_listener
    participant AudioNot as audio_notification_node
    participant StatusLED as status_led_node
    participant GestInt as gesture_intent_node
    participant Speech as speech_node
    participant OpenAI as OpenAI API

    Note over OAK,OpenAI: PHASE 1 - System Idle in BLUE State

    rect rgb(200, 220, 255)
        Note over OAK,OpenAI: No person present - LED OFF - Gestures disabled
        AudioNot->>StatusLED: person_status BLUE 10Hz
        StatusLED->>StatusLED: GPIO 17 LOW - LED OFF
    end

    Note over OAK,OpenAI: PHASE 2 - Person Approaches Camera

    OAK->>ImgList: Raw frame 30Hz
    ImgList->>ImgList: Downscale 640x360 - Haar Cascade
    ImgList->>ImgList: Face detected raw

    rect rgb(255, 255, 200)
        Note over ImgList: Hysteresis 0.3s presence threshold CODE
        ImgList->>ImgList: Wait 0.3s continuous detection
        ImgList->>AudioNot: face_count 1 stable presence
    end

    ImgList->>ImgList: LBPH recognition 6.5Hz frame_skip 2
    ImgList->>ImgList: Confidence 42 below 150 threshold CODE

    Note over OAK,OpenAI: PHASE 3 - NEW Rolling Window Filter for RED Entry

    rect rgb(255, 230, 200)
        Note over AudioNot: Rolling 1s window - Need 3 matches PROPOSED
        ImgList->>AudioNot: person_id severin Match 1
        AudioNot->>AudioNot: Buffer add - Count 1 of 3
        ImgList->>AudioNot: person_id severin Match 2
        AudioNot->>AudioNot: Buffer add - Count 2 of 3
        ImgList->>AudioNot: person_id severin Match 3
        AudioNot->>AudioNot: Buffer add - Count 3 of 3 THRESHOLD MET
    end

    Note over OAK,OpenAI: PHASE 4 - Transition to RED State

    rect rgb(255, 200, 200)
        AudioNot->>AudioNot: State BLUE to RED
        Note over AudioNot: RED timer starts 15s resets on recognition
        AudioNot->>StatusLED: person_status RED
        StatusLED->>StatusLED: GPIO 17 HIGH - LED ON
        AudioNot->>AudioNot: Play Hello beep 2mp3 volume 0.02 CODE
        AudioNot->>GestInt: person_status RED 10Hz continuous
    end

    Note over OAK,OpenAI: PHASE 5 - Gesture Start Conversation

    rect rgb(200, 255, 200)
        Note over GestInt: Gestures now authorized - person_status RED
        ImgList->>ImgList: MediaPipe Hands detect hand
        ImgList->>ImgList: SVM classify index_finger_up conf 0.87
        ImgList->>GestInt: gesture_event index_finger_up

        GestInt->>GestInt: Gate 1 - person_status RED PASS
        GestInt->>GestInt: Gate 2 - session_active false PASS
        GestInt->>GestInt: Gate 3 - cooldown 2s elapsed CODE PASS
        GestInt->>Speech: start_session service call
    end

    Note over OAK,OpenAI: PHASE 6 - Speech Session Connects

    rect rgb(220, 255, 220)
        Speech->>Speech: Lifecycle Inactive to Active
        Speech->>OpenAI: WebSocket connect wss api openai
        OpenAI-->>Speech: Connection established
        Speech->>Speech: Start mic stream 48kHz to 24kHz
        Speech->>GestInt: session_status connected

        GestInt->>GestInt: session_active true SPEAKING state
        Note over GestInt: Grace period 5s ignore fist gestures
        GestInt->>GestInt: Play Start beep 16mp3
    end

    Note over OAK,OpenAI: PHASE 7 - Active Conversation with VAD

    rect rgb(230, 230, 255)
        Note over Speech,OpenAI: Conversation Active - VAD monitoring

        Speech->>OpenAI: Audio stream user speaking
        OpenAI->>OpenAI: Whisper STT - GPT4o - TTS
        OpenAI-->>Speech: Audio response 24kHz
        Speech->>Speech: Resample 24kHz to 44.1kHz play
        Speech->>GestInt: voice_activity speaking
        Note over GestInt: VAD speaking - 60s silence timer PAUSED

        Speech->>GestInt: voice_activity silent
        Note over GestInt: VAD silent - 60s timer STARTS
    end

    Note over OAK,OpenAI: PHASE 8 - Manual Stop with Fist Gesture

    rect rgb(255, 220, 255)
        ImgList->>GestInt: gesture_event fist
        GestInt->>GestInt: Gate 1 - person_status RED PASS
        GestInt->>GestInt: Gate 2 - session_active true PASS
        GestInt->>GestInt: Gate 3 - grace 5s elapsed PASS
        GestInt->>GestInt: Gate 4 - cooldown 1s elapsed CODE PASS
        GestInt->>Speech: stop_session service call

        Speech->>OpenAI: WebSocket disconnect
        Speech->>Speech: Lifecycle Active to Inactive
        Speech->>GestInt: session_status disconnected

        GestInt->>GestInt: session_active false IDLE state
        GestInt->>GestInt: Play Stop beep 20mp3
    end

    Note over OAK,OpenAI: PHASE 9 - User Leaves Camera View

    rect rgb(255, 240, 200)
        ImgList->>ImgList: Face lost raw detection
        Note over ImgList: Hysteresis 5s absence threshold CODE
        ImgList->>ImgList: Wait 5s continuous absence
        ImgList->>AudioNot: face_count 0 stable absence
        ImgList->>AudioNot: person_id no_person
    end

    Note over OAK,OpenAI: PHASE 10 - RED Timeout and Return to BLUE

    rect rgb(200, 220, 255)
        Note over AudioNot: RED timer 15s expires no recognition
        AudioNot->>AudioNot: Check face_count 0 no face
        AudioNot->>AudioNot: State RED to BLUE
        AudioNot->>AudioNot: Clear recognition_buffer PROPOSED
        AudioNot->>StatusLED: person_status BLUE
        StatusLED->>StatusLED: GPIO 17 LOW - LED OFF
        AudioNot->>AudioNot: Play Lost beep 5mp3
        AudioNot->>GestInt: person_status BLUE 10Hz

        Note over GestInt: Watchdog timer would start 300s CODE should be 35s
    end

    Note over OAK,OpenAI: System returns to IDLE BLUE state
```

---

## Timing Parameters Summary (Code Reality vs Documentation)

```mermaid
flowchart TB
    subgraph PERCEPTION["Perception Layer"]
        CAM[Camera 30Hz]
        PROC[Processing 13Hz]
        PRES[Face Presence 0.3s CODE<br>Doc says 2.0s]
        ABS[Face Absence 5.0s]
        REC[Recognition 6.5Hz<br>Threshold 150 CODE<br>Doc says 70]
    end

    subgraph STATUS["Status Manager"]
        ROLL[Rolling Window 1s<br>3 matches PROPOSED]
        RED[RED State 15s timer]
        GREEN[GREEN Entry 2s delay]
        BLUE[BLUE Entry 3s delay]
    end

    subgraph GESTURE["Gesture Control"]
        START[Start Cooldown 2s CODE<br>Doc says 5s]
        STOP[Stop Cooldown 1s CODE<br>Doc says 3s]
        GRACE[Grace Period 5s]
        VAD[VAD Timeout 60s]
        WATCH[Watchdog 300s CODE<br>Doc says 35s]
    end

    subgraph AUDIO["Audio Feedback"]
        VOL[Volume 0.02 CODE<br>Doc says 0.30]
        HELLO[Hello 2mp3]
        LOST[Lost 5mp3]
        SSTART[Start 16mp3]
        SSTOP[Stop 20mp3]
    end

    CAM --> PROC --> PRES
    PRES --> ABS
    PROC --> REC
    REC --> ROLL
    ROLL --> RED
    RED --> GREEN
    RED --> BLUE
    RED --> GESTURE
    GESTURE --> AUDIO

    style PRES fill:#ffcc00
    style REC fill:#ffcc00
    style START fill:#ffcc00
    style STOP fill:#ffcc00
    style WATCH fill:#ff6666
    style VOL fill:#ff6666
    style ROLL fill:#90EE90
```

---

## Executive Summary

Cross-verification of system documentation against actual implementation code revealed **6 major discrepancies** in timing parameters, configuration values, and core system logic.

**Impact Level:**
- 🔴 **CRITICAL:** System Logic - Robust RED Entry (NEW), Audio volume (15x difference), Watchdog timeout (8.5x difference)
- 🟡 **MODERATE:** Face presence threshold (6.7x difference), Recognition confidence (2.1x difference)
- 🟢 **MINOR:** Gesture cooldowns (shorter in code, better UX)

---

## Discrepancy 6: System Logic - Robust Targeted Person Entry (RED Status) ⚠️ CRITICAL

### Current Implementation
- **Logic:** Status switches to RED immediately upon the first recognition of the target person.
- **Location:** `audio_notification_node.py` lines 264-297
- **Vulnerability:** Single-frame misidentifications can trigger false positive RED status.

### Proposed Rock Solid Logic
- **Requirement:** 3 recognitions of the targeted person within a 1-second rolling window.
- **Trigger:** Status only switches to RED when this threshold is met.
- **Persistence:** Once in RED, the system stays RED for at least 15 seconds (existing logic).
- **Reset:** Every subsequent recognition of the targeted person resets the 15-second timer.

### Latency Impact Analysis
| Scenario | Current Logic | Proposed Logic | Difference |
|----------|--------------|----------------|------------|
| First recognition to RED | 0ms (instant) | ~460ms (3 matches at 6.5Hz) | +460ms |
| False positive risk | HIGH | LOW | ✅ Improved |
| User experience | Jittery | Deliberate | ✅ Improved |

---

## COMPLETE CODE CHANGE SPECIFICATION

### Overview: Files Requiring Changes

| File | Change Type | Scope |
|------|-------------|-------|
| `audio_notification_node.py` | **MODIFY** | Add rolling window filter for RED entry |
| `image_listener.py` | NO CHANGE | Already publishes at correct rate (6.5 Hz) |
| `gesture_intent_node.py` | NO CHANGE | Already subscribes to person_status |
| `status_led_node.py` | NO CHANGE | Already subscribes to person_status |
| `database_logger_node.py` | NO CHANGE | Already subscribes to person_status |

**Conclusion:** Only ONE file needs modification: `audio_notification_node.py`

---

### Detailed Changes for `audio_notification_node.py`

**File Path:** `ros2_ws/src/r2d2_audio/r2d2_audio/audio_notification_node.py`

---

#### CHANGE 1: Add New Parameters (After line 95)

**Current Code (lines 90-95):**
```python
self.declare_parameter('red_status_timeout_seconds', 15.0)  # Simple 15s timeout, resets on recognition
self.declare_parameter('cooldown_seconds', 2.0)   # Min between recognition alerts
self.declare_parameter('recognition_cooldown_after_loss_seconds', 5.0)  # Quiet period after loss alert
self.declare_parameter('recognition_audio_file', 'Voicy_R2-D2 - 2.mp3')  # Recognition alert audio
self.declare_parameter('loss_audio_file', 'Voicy_R2-D2 - 5.mp3')  # Loss alert audio
self.declare_parameter('enabled', True)
```

**Insert After line 95:**
```python
# Robust RED entry parameters (rolling window filter)
self.declare_parameter('red_entry_match_threshold', 3)  # Matches required in window
self.declare_parameter('red_entry_window_seconds', 1.0)  # Rolling window duration
```

---

#### CHANGE 2: Get New Parameter Values (After line 110)

**Current Code (lines 105-110):**
```python
self.red_status_timeout = self.get_parameter('red_status_timeout_seconds').value
self.cooldown_seconds = self.get_parameter('cooldown_seconds').value
self.recognition_cooldown_after_loss = self.get_parameter('recognition_cooldown_after_loss_seconds').value
recognition_audio_filename = self.get_parameter('recognition_audio_file').value
loss_audio_filename = self.get_parameter('loss_audio_file').value
self.enabled = self.get_parameter('enabled').value
```

**Insert After line 110:**
```python
# Robust RED entry parameters
self.red_entry_match_threshold = self.get_parameter('red_entry_match_threshold').value
self.red_entry_window_seconds = self.get_parameter('red_entry_window_seconds').value
```

---

#### CHANGE 3: Add State Variable for Rolling Buffer (After line 126)

**Current Code (lines 117-126):**
```python
# Status tracking (for LED, STT-LLM-TTS, database)
self.current_status = "blue"  # "red" (recognized) | "blue" (lost) | "green" (unknown)
self.current_person = "no_person"  # "severin" | "unknown" | "no_person"
self.status_changed_time = time.time()
self.unknown_person_detected = False
self.last_known_state = "unknown"

# Track face count to detect when no faces are visible
self.last_face_count = None
self.last_face_count_time = None
```

**Insert After line 126:**
```python
# Rolling buffer for robust RED entry (list of (timestamp, person_id) tuples)
self.recognition_buffer = []
```

---

#### CHANGE 4: Modify person_callback() - Replace Immediate RED Transition

**Current Code (lines 264-297) - REPLACE ENTIRELY:**
```python
# MULTI-USER: Any recognized person (not "unknown", not empty) triggers RED
# The training itself is the authorization - if LBPH recognizes them, they're authorized
if person_id and person_id != "unknown":
    # Trained person recognized - RESET 15s countdown timer
    was_red = (self.current_status == "red")
    self.last_recognition_time = current_time  # ← KEY: Reset timer
    
    # Reset smoothing timers when entering RED
    self.face_detected_start_time = None
    self.face_absent_start_time = None
    
    if not was_red:
        # Transition to RED (from any state)
        old_status = self.current_status
        self.current_status = "red"
        self.current_person = person_id  # Use actual person name
        self.status_changed_time = current_time
        self.unknown_person_detected = False
        
        # Publish status FIRST
        self._publish_status("red", person_id, confidence=0.95)
        
        # Play "Hello!" beep (with cooldown)
        if self.last_recognition_beep_time is None or \
           (current_time - self.last_recognition_beep_time) >= self.cooldown_seconds:
            self._play_audio_file(self.recognition_audio, alert_type="RECOGNITION")
            self.last_recognition_beep_time = current_time
            self._publish_event(f"🎉 Recognized {person_id}!")
        
        self.get_logger().info(f"✓ {person_id} recognized ({old_status} → RED)")
    else:
        # Already RED - just reset timer and update duration
        self._publish_status("red", self.current_person, confidence=0.95)
        self.get_logger().debug(f"RED: Timer reset ({person_id} seen)")
```

**NEW Code (Replace lines 264-297):**
```python
# MULTI-USER: Any recognized person (not "unknown", not empty) triggers RED
# The training itself is the authorization - if LBPH recognizes them, they're authorized
if person_id and person_id != "unknown":
    # Add to rolling buffer
    self.recognition_buffer.append((current_time, person_id))
    
    # Clean buffer: remove entries older than window
    cutoff_time = current_time - self.red_entry_window_seconds
    self.recognition_buffer = [(t, p) for t, p in self.recognition_buffer if t >= cutoff_time]
    
    # Count matches for THIS person in the window
    match_count = sum(1 for t, p in self.recognition_buffer if p == person_id)
    
    was_red = (self.current_status == "red")
    
    if was_red:
        # Already RED - just reset 15s timer (no threshold needed to stay RED)
        self.last_recognition_time = current_time
        self._publish_status("red", self.current_person, confidence=0.95)
        self.get_logger().debug(f"RED: Timer reset ({person_id} seen, {match_count} in window)")
    
    elif match_count >= self.red_entry_match_threshold:
        # THRESHOLD MET - Transition to RED
        self.last_recognition_time = current_time
        
        # Reset smoothing timers when entering RED
        self.face_detected_start_time = None
        self.face_absent_start_time = None
        
        # Transition to RED (from any state)
        old_status = self.current_status
        self.current_status = "red"
        self.current_person = person_id
        self.status_changed_time = current_time
        self.unknown_person_detected = False
        
        # Publish status FIRST
        self._publish_status("red", person_id, confidence=0.95)
        
        # Play "Hello!" beep (with cooldown)
        if self.last_recognition_beep_time is None or \
           (current_time - self.last_recognition_beep_time) >= self.cooldown_seconds:
            self._play_audio_file(self.recognition_audio, alert_type="RECOGNITION")
            self.last_recognition_beep_time = current_time
            self._publish_event(f"🎉 Recognized {person_id}!")
        
        self.get_logger().info(
            f"✓ {person_id} recognized ({old_status} → RED) "
            f"[{match_count}/{self.red_entry_match_threshold} matches in {self.red_entry_window_seconds}s]"
        )
    else:
        # Threshold NOT met yet - waiting for more matches
        self.get_logger().debug(
            f"Recognition buffered: {person_id} ({match_count}/{self.red_entry_match_threshold} in window)"
        )
```

---

#### CHANGE 5: Update Logging Output (Line ~237)

**Current Code (lines 230-245):**
```python
self.get_logger().info(
    f"Audio Notification Node initialized (Audio Files):\n"
    f"  Target person: {self.target_person}\n"
    f"  Recognition audio: {self.recognition_audio.name}\n"
    f"  Loss audio: {self.loss_audio.name}\n"
    f"  Audio volume: {self.audio_volume*100:.0f}%\n"
    f"  ALSA device: {self.alsa_device}\n"
    f"  RED status timeout: {self.red_status_timeout}s (resets on recognition)\n"
    f"  GREEN entry delay: {self.green_entry_delay}s (BLUE→GREEN smoothing)\n"
    f"  BLUE entry delay: {self.blue_entry_delay}s (GREEN→BLUE smoothing)\n"
    f"  Alert cooldown: {self.cooldown_seconds}s (min between alerts)\n"
    f"  Recognition cooldown after loss: {self.recognition_cooldown_after_loss}s (quiet period after loss alert)\n"
    f"  Enabled: {self.enabled}\n"
    f"  Audio player: {self.audio_player_path}\n"
    f"  State Machine: RED is primary (ignores non-target while active)"
)
```

**Add to logging (after "RED status timeout" line):**
```python
f"  RED entry threshold: {self.red_entry_match_threshold} matches in {self.red_entry_window_seconds}s window\n"
```

---

#### CHANGE 6: Clear Buffer on RED Exit (In check_loss_state, after line 480)

**Add after transitioning out of RED (both GREEN and BLUE paths):**
```python
# Clear recognition buffer on RED exit
self.recognition_buffer = []
```

**Specific locations:**
- After line 446 (GREEN transition): Add `self.recognition_buffer = []`
- After line 469 (BLUE transition): Add `self.recognition_buffer = []`

---

### Build and Deploy Commands

```bash
# 1. Edit the file
nano ~/dev/r2d2/ros2_ws/src/r2d2_audio/r2d2_audio/audio_notification_node.py

# 2. Rebuild the package
cd ~/dev/r2d2/ros2_ws
colcon build --packages-select r2d2_audio

# 3. Restart the service
sudo systemctl restart r2d2-audio-notification.service

# 4. Monitor logs
journalctl -u r2d2-audio-notification.service -f
```

---

### Testing Verification

1. **Test: Walk into camera view**
   - Expected: LED should NOT turn on immediately
   - Expected: LED turns on after ~0.5-1 second (3+ matches accumulated)
   - Log should show: `"3/3 matches in 1.0s window"`

2. **Test: Quick pass-by (< 0.5 seconds)**
   - Expected: LED should NOT turn on
   - Log should show: `"Recognition buffered: severin (1/3 in window)"`

3. **Test: Stay in view after RED**
   - Expected: LED stays on, resets 15s timer on each recognition
   - Log should show: `"RED: Timer reset (severin seen, X in window)"`

4. **Test: Leave camera view while RED**
   - Expected: After 15s, LED turns off
   - Expected: recognition_buffer is cleared

---

## All Discrepancies Summary

### Discrepancy 1: Audio Volume ⚠️ CRITICAL
| Aspect | Documentation | Code | Difference |
|--------|--------------|------|------------|
| Value | 0.30 (30%) | 0.02 (2%) | 15x quieter |
| Location | Multiple docs | `audio_params.yaml`, nodes | - |
| **Recommendation** | Update docs to 0.02 | - | - |

### Discrepancy 2: Watchdog Timeout ⚠️ CRITICAL
| Aspect | Documentation | Code | Difference |
|--------|--------------|------|------------|
| Value | 35 seconds | 300 seconds | 8.5x longer |
| Location | Multiple docs | `gesture_intent_node.py:58` | - |
| **Recommendation** | Change code to 35s | - | 💰 Cost savings |

### Discrepancy 3: Face Presence Threshold 🟡 MODERATE
| Aspect | Documentation | Code | Difference |
|--------|--------------|------|------------|
| Value | 2.0 seconds | 0.3 seconds | 6.7x faster |
| Location | Multiple docs | `image_listener.py:57` | - |
| **Recommendation** | Update docs to 0.3s | - | ✅ Better UX |

### Discrepancy 4: Recognition Confidence Threshold 🟡 MODERATE
| Aspect | Documentation | Code | Difference |
|--------|--------------|------|------------|
| Value | 70.0 | 150.0 | 2.1x more lenient |
| Location | Multiple docs | `image_listener.py:54` | - |
| **Recommendation** | Update docs to 150.0 | - | Note: lower=stricter |

### Discrepancy 5: Gesture Cooldowns 🟢 MINOR
| Aspect | Documentation | Code | Difference |
|--------|--------------|------|------------|
| Start Cooldown | 5.0 seconds | 2.0 seconds | 2.5x faster |
| Stop Cooldown | 3.0 seconds | 1.0 seconds | 3x faster |
| Location | Multiple docs | `gesture_intent_node.py:54-55` | - |
| **Recommendation** | Update docs to 2.0s/1.0s | - | ✅ Better UX |

### Discrepancy 6: RED Entry Logic 🔴 PROPOSED NEW
| Aspect | Current | Proposed | Benefit |
|--------|---------|----------|---------|
| Trigger | Instant (1 match) | 3 matches in 1s | Eliminates false positives |
| Latency | 0ms | ~460ms | Acceptable tradeoff |
| Location | `audio_notification_node.py` | - | - |
| **Recommendation** | Implement rolling window | - | ✅ Rock solid |

---

## Summary of All Required Code Changes

| Priority | File | Change | Lines |
|----------|------|--------|-------|
| 🔴 HIGH | `audio_notification_node.py` | Add rolling window RED entry filter | 95, 110, 126, 264-297, 237, 446, 469 |
| 🔴 HIGH | `gesture_intent_node.py` | Change watchdog 300→35s | Line 58 |
| 🟢 NONE | All other files | No changes required | - |

---

## Consistency Verification ✅

| Check | Status | Notes |
|-------|--------|-------|
| Rolling window at 6.5Hz recognition rate | ✅ VALID | 3 matches in 1s = ~460ms minimum |
| RED timer 15s still resets on recognition | ✅ UNCHANGED | Only entry logic changes |
| Downstream subscribers unaffected | ✅ VERIFIED | LED, Gesture, Database nodes unchanged |
| Hysteresis filters still apply | ✅ UNCHANGED | 0.3s presence, 5s absence |
| Grace period 5s still protects fist | ✅ UNCHANGED | Prevents false stop |
| VAD 60s timeout still works | ✅ UNCHANGED | Speech auto-stop |
| Buffer cleared on RED exit | ✅ PROPOSED | Prevents stale data |

---

**Document Status:** COMPLETE - Ready for implementation  
**Verified:** All downstream subscribers (LED, Gesture, Database) require NO changes  
**Created:** December 21, 2025  
**Author:** AI Assistant (Code Verification Analysis)
