# R2D2 SYSTEM STATUS ARCHITECTURE - COMPLETE REFERENCE
## All Events, States, Timings, Filters, and Audio Feedback

**Date:** December 21, 2025  
**Document Type:** Complete System Status Reference (Based on Documentation Specifications)  
**Status:** ✅ AUTHORITATIVE REFERENCE  
**Platform:** NVIDIA Jetson AGX Orin 64GB + ROS 2 Humble  
**Purpose:** Comprehensive description of system events, states, and multi-modal feedback

---

## Introduction

From an overall system perspective, the R2D2 architecture integrates multiple status subsystems with their events and continuous states. This includes:

- **Detection Events:** "person detected," "targeted person detected," "index-finger-up gesture detected"
- **Continuous Status States:** RED (recognized), BLUE (lost/idle), GREEN (unknown person)
- **Speech Status:** Active conversation vs disconnected
- **Outward-Facing Events:** LED states, audio beeps, service lifecycle

All these elements are interconnected as part of one large, integrated system where perception drives state machines, which drive multi-modal user feedback.

---

## SECTION 1: INPUT EVENTS (Perception Layer)

### 1.1 Camera-Based Detection Events

#### Event 1.1.1: Face Detection with Temporal Smoothing

**Topic:** `/r2d2/perception/face_count` (std_msgs/Int32, 13 Hz)  
**Source Node:** `image_listener` (r2d2_perception package)  
**Detection Method:** Haar Cascade classifier on grayscale 640×360 frames  
**Processing Rate:** 13 Hz (downscaled from 30 Hz camera input)

**Temporal Smoothing (Hysteresis Filter):**

The system applies a hysteresis filter to prevent flickering detections from causing state machine instability:

```
State Machine: NO_FACE ↔ TRANSITIONING ↔ FACE_PRESENT

Face Presence Threshold: 2.0 seconds
  - Must detect face continuously for 2 seconds before confirming presence
  - Effect: Filters out brief false positives (e.g., shadows, movements)
  
Face Absence Threshold: 5.0 seconds
  - Must lose face continuously for 5 seconds before confirming absence
  - Effect: Prevents loss alerts from brief occlusions
```

**Published Values (After Smoothing):**
- `0` = No face detected (confirmed absence after 5s continuous loss)
- `1` = Face present (confirmed presence after 2s continuous detection)
- `2+` = Multiple faces detected

**Purpose:** Stable face count prevents downstream state machines from experiencing jitter, ensuring smooth transitions and preventing premature "Lost you!" beeps.

---

#### Event 1.1.2: Person Recognition

**Topic:** `/r2d2/perception/person_id` (std_msgs/String, 6.5 Hz)  
**Source Node:** `image_listener` (r2d2_perception package)  
**Recognition Method:** LBPH (Local Binary Pattern Histogram) face recognizer  
**Processing Rate:** 6.5 Hz (every 2nd frame, frame_skip=2)

**Published Values:**
- **Trained person name** (e.g., `"severin"`, `"alice"`) - any trained person
- `"unknown"` - face detected but confidence below threshold
- `"no_person"` - no face detected

**Recognition Parameters:**

```
Confidence Threshold: 70.0 (distance metric, lower = better match)
  - LBPH returns distance score (NOT probability)
  - Typical good match: 35-50
  - Typical unknown: 80-120
  - Threshold: Values < 70.0 = recognized

Frame Skip: 2 (process every 2nd frame)
  - Balances CPU usage vs responsiveness
  - Results in 6.5 Hz recognition rate (13 Hz ÷ 2)
```

**Gating Logic:**
- Only runs when `face_count > 0` (smoothed value from hysteresis)
- Prevents wasted CPU on recognition when no face present

**Multi-User Authorization:**
- ANY trained person triggers recognition
- Training a person = authorizing them
- No hardcoded person names in code
- Model paths auto-resolved from PersonRegistry database

---

#### Event 1.1.3: Gesture Detection

**Topic:** `/r2d2/perception/gesture_event` (std_msgs/String, event-driven)  
**Source Node:** `image_listener` (r2d2_perception package)  
**Detection Method:** MediaPipe Hands + person-specific SVM classifier  
**Processing Rate:** ~10 Hz when active (frame_skip=3)

**Gesture Values:**
- `"index_finger_up"` - Start conversation gesture
- `"fist"` - Stop conversation gesture

**Multi-Level Gating:**

1. **Enable Check:** `enable_gesture_recognition` parameter must be true
2. **Frame Skip:** Only process every 3rd frame (reduces CPU load)
3. **Person Gate:** Only when `last_person_id` matches trained person
4. **Confidence Gate:** Only when SVM confidence > 0.7 (70%)
5. **State Change Gate:** Only publish on gesture state change

**Effect:** 
- Gestures ONLY detected when recognized person present (inherent security)
- Event-driven publishing (not continuous stream)
- Prevents gesture spam and unauthorized access

---

## SECTION 2: INTERNAL STATUS STATES (State Machines)

### 2.1 Person Recognition State Machine

**Node:** `audio_notification_node` (r2d2_audio package)  
**Status Topic:** `/r2d2/audio/person_status` (std_msgs/String, JSON, 10 Hz)  
**Publishing Rate:** 10 Hz continuous (even when no state changes)  
**Purpose:** Track person presence and drive audio/LED feedback

---

#### State 2.1.1: 🔴 RED State (Target Person Recognized)

**JSON Message Format:**
```json
{
  "status": "red",
  "person_identity": "severin",
  "timestamp_sec": 1765212914,
  "timestamp_nanosec": 949382424,
  "confidence": 0.95,
  "duration_seconds": 15.3,
  "is_loss_state": false,
  "audio_event": "recognition"
}
```

**Entry Conditions:**
- ANY trained person detected (person_id returns trained name)
- Transition is **instant** (no delay from BLUE or GREEN)

**RED Status Timer:**
```
Duration: 15.0 seconds (resets on EACH recognition)
Behavior: While person continuously recognized, timer keeps resetting → stay RED indefinitely
Exit: When 15s expires WITHOUT new recognition
```

**State Priority:**
- **RED is PRIMARY state** - highest priority
- While in RED, all other face detections (unknown faces) are **IGNORED**
- Provides immunity to camera flickers during active recognition
- Only exits on sustained absence (15s no recognition)

**Exit Conditions:**

When 15s timer expires without recognition:
- **If face_count > 0:** Transition to GREEN (unknown person visible)
- **If face_count = 0:** Transition to BLUE (no person visible)

---

#### State 2.1.2: 🔵 BLUE State (No Person / Lost)

**JSON Message Format:**
```json
{
  "status": "blue",
  "person_identity": "no_person",
  "timestamp_sec": 1765212914,
  "timestamp_nanosec": 949382424,
  "confidence": 0.0,
  "duration_seconds": 45.2,
  "is_loss_state": true,
  "audio_event": "loss"
}
```

**Entry Conditions:**

From GREEN state:
```
Blue Entry Delay: 3.0 seconds
  - Requires 3s continuous absence (no face) before GREEN→BLUE
  - Prevents flicker between GREEN and BLUE states
```

From RED timeout:
```
Immediate if face_count = 0
  - No delay when transitioning from RED to BLUE
  - Face already confirmed absent by perception hysteresis (5s)
```

**Total Time to Loss Alert (from RED):**
```
1. User leaves camera view
2. Perception hysteresis: 5.0s (face_absence_threshold)
3. RED status timer: up to 15.0s (until next recognition check)
4. Total: ~20 seconds from user leaving to BLUE transition
5. BLUE entry → "Lost you!" audio beep plays
```

**Exit Conditions:**
- **To RED:** Trained person detected (instant transition, target takes priority)
- **To GREEN:** Unknown face detected for 2s (green_entry_delay)

---

#### State 2.1.3: 🟢 GREEN State (Unknown Person)

**JSON Message Format:**
```json
{
  "status": "green",
  "person_identity": "unknown",
  "timestamp_sec": 1765212914,
  "timestamp_nanosec": 949382424,
  "confidence": 0.0,
  "duration_seconds": 8.7,
  "is_loss_state": false,
  "audio_event": "none"
}
```

**Entry Conditions:**

From BLUE state:
```
Green Entry Delay: 2.0 seconds
  - Requires 2s continuous face detection before BLUE→GREEN
  - Prevents flicker when unknown person appears
```

From RED timeout:
```
Immediate if face_count > 0
  - No delay when transitioning from RED to GREEN
  - Face already confirmed present by perception hysteresis (2s)
```

**Smoothing Behavior:**
- GREEN/BLUE transitions use hysteresis to prevent flicker
- After RED ends, system uses slower transitions (2s/3s delays)
- Asymmetric delays: faster to detect (2s), slower to lose (3s)

**Key Feature:**
- **Silent operation** - no audio alerts for unknown persons
- Only visual feedback via LED (if configured)
- Privacy-preserving for non-authorized individuals

**Exit Conditions:**
- **To RED:** Trained person detected (instant, target takes priority)
- **To BLUE:** No face for 3s (blue_entry_delay)

---

### 2.2 Speech Session State Machine

**Node:** `speech_node` (r2d2_speech package - lifecycle node)  
**Status Topic:** `/r2d2/speech/session_status` (std_msgs/String, JSON, on-change)  
**Publishing:** Only when state changes (not continuous)

**Important Distinction:**
- **Lifecycle State:** "unconfigured" → "inactive" → "active" (node readiness)
- **Session State:** "connected" vs "disconnected" (conversation status)

For gesture control, only **session state** matters.

---

#### State 2.2.1: Connected (Conversation Active)

**JSON Message Format:**
```json
{
  "status": "connected",
  "session_id": "uuid-string",
  "started_at": "2025-12-21T10:30:00",
  "model": "gpt-4o-realtime-preview-2024-12-17",
  "voice": "sage"
}
```

**Active Systems:**
- OpenAI Realtime API WebSocket connection (wss://api.openai.com/v1/realtime)
- HyperX QuadCast S microphone streaming (48kHz → 24kHz resampling)
- Speaker output active (PAM8403, 24kHz → 44.1kHz resampling)
- Voice Activity Detection (VAD) monitoring active
- Conversation history persisting to SQLite

**Entry Trigger:** Service call to `/r2d2/speech/start_session` (std_srvs/Trigger)  
**Typical Latency:** 500-2000ms (WebSocket connection setup)

**Exit Triggers:**
- Manual: Service call to `/r2d2/speech/stop_session`
- Automatic: VAD silence timeout (60s consecutive silence)
- Automatic: Watchdog timeout (35s person absence during IDLE state)

---

#### State 2.2.2: Disconnected (Conversation Inactive)

**JSON Message Format:**
```json
{
  "status": "disconnected",
  "session_id": null,
  "ended_at": "2025-12-21T10:35:00",
  "reason": "manual_stop" | "vad_timeout" | "watchdog" | "error"
}
```

**Inactive Systems:**
- No WebSocket connection
- No audio streaming
- No API charges

**Entry:** 
- Initial state (node starts disconnected)
- After service stop
- After automatic timeout

**Exit:** When `/r2d2/speech/start_session` called

---

### 2.3 Conversation Control State Machine

**Node:** `gesture_intent_node` (r2d2_gesture package)  
**Internal States:** IDLE / SPEAKING (not published as topic, internal only)  
**Purpose:** Translate gestures into service calls with strict gating

---

#### State 2.3.1: IDLE State

**Characteristics:**
- No conversation active (`session_active = false`)
- Gestures monitored (start gesture enabled if person_status="red")
- Camera only used for gesture gating (not for timeout)
- Watchdog timer inactive

**Gating for Start Gesture (Index Finger Up):**

All conditions must be true:
1. **Person Status:** `person_status == "red"` (must be recognized person)
2. **Session State:** `session_active == false` (speech not already running)
3. **Cooldown:** `time_since_last_trigger >= 5.0 seconds`

**Effect:** Only authorized person can start conversation, prevents accidental triggers

---

#### State 2.3.2: SPEAKING State (VAD-Protected)

**Characteristics:**
- Conversation active (`session_active = true`)
- VAD-only timeout: 60s consecutive silence
- Grace period: 5s after start (ignores fist gestures)
- **IMMUNE to camera status** (RED→BLUE does NOT stop conversation)

**Protection Mechanisms:**

**1. Speaking Start Grace Period:**
```
Duration: 5.0 seconds
Purpose: Prevent false fist detection during hand lowering after start gesture
Effect: All fist gestures IGNORED for 5s after index_finger_up accepted
Reasoning: Hand transitional movements often misclassified as fist
```

**2. VAD-Based Silence Timeout (Option 2 - Current Implementation):**
```
Duration: 60.0 seconds of CONSECUTIVE silence
Monitoring: OpenAI's built-in Voice Activity Detection (VAD)
Topic: /r2d2/speech/voice_activity (JSON: "speaking" or "silent")

Timer Behavior:
  - While user speaking: Timer PAUSED (no timeout)
  - When user silent: Timer STARTS/RESUMES from 0
  - If speech resumes: Timer CANCELLED and RESET
  
Effect: Only stops after extended silence, completely immune to camera flickers
Benefit: No premature termination from brief RED↔GREEN transitions
```

**3. Watchdog Idle Failsafe:**
```
Duration: 35.0 seconds
Purpose: Emergency auto-stop for forgotten sessions when person absent
Trigger: Person status != "red" for 35 continuous seconds
Effect: Auto-stops speech service to save OpenAI API costs
Note: Only active during IDLE state, NOT during SPEAKING
      SPEAKING state protected by VAD-only timeout
```

**Gating for Stop Gesture (Fist):**

All conditions must be true:
1. **Person Status:** `person_status == "red"` (must be recognized person)
2. **Session State:** `session_active == true` (speech must be running)
3. **Grace Period:** `time_since_speaking_start >= 5.0 seconds`
4. **Cooldown:** `time_since_last_trigger >= 3.0 seconds`

**Override:** Fist gesture ALWAYS works after grace period (immediate stop, no camera dependency)

---

## SECTION 3: OUTPUT EVENTS & FEEDBACK (Multi-Modal)

### 3.1 Audio Alerts (MP3 R2-D2 Beeps)

**Audio Assets Location:** `~/dev/r2d2/ros2_ws/src/r2d2_audio/r2d2_audio/assets/audio/`  
**Playback Command:** `ffplay -nodisp -autoexit -af volume={volume} {file_path}`  
**Global Volume:** 0.30 (30% of maximum)  
**Hardware:** PAM8403 amplifier (Class-D, 3W) + 8Ω speaker

---

#### Audio Alert 3.1.1: Recognition Beep ("Hello!")

**File:** `Voicy_R2-D2 - 2.mp3`  
**Duration:** ~2 seconds  
**Source Node:** `audio_notification_node`  
**Trigger:** Transition to RED state (trained person first recognized)

**Cooldown Protection:**
```
Recognition Alert Cooldown: 2.0 seconds
  - Minimum 2s between "Hello!" beeps
  - Prevents spam during recognition jitter
  - Prevents multiple beeps if person briefly leaves and returns

Post-Loss Quiet Period: 5.0 seconds
  - After "Lost you!" beep, wait 5s before next "Hello!"
  - Prevents rapid beep sequence (loss → immediate recognition)
  - Provides calm transition period
```

**Playback:**
```bash
ffplay -nodisp -autoexit -af volume=0.30 \
  ~/dev/r2d2/ros2_ws/src/r2d2_audio/r2d2_audio/assets/audio/Voicy_R2-D2\ -\ 2.mp3
```

---

#### Audio Alert 3.1.2: Loss Beep ("Oh, I lost you!")

**File:** `Voicy_R2-D2 - 5.mp3`  
**Duration:** ~5 seconds  
**Source Node:** `audio_notification_node`  
**Trigger:** Transition to BLUE state after person loss confirmed

**Timing Breakdown (Total ~20 seconds):**
```
1. Person stops being recognized
2. Perception hysteresis filter: 5.0s (face_absence_threshold)
3. RED status timer: continues for remaining time (up to 15s)
4. When 15s expires AND face_count=0: Transition to BLUE
5. BLUE entry → "Lost you!" beep plays immediately
```

**Cooldown Protection:**
```
Loss Alert Cooldown: 2.0 seconds
  - Minimum 2s between "Lost you!" beeps
  - Prevents multiple loss alerts
```

**Purpose:** Informs user that system has lost recognition, conversation may auto-stop soon

---

#### Audio Alert 3.1.3: Session Start Beep

**File:** `Voicy_R2-D2 - 16.mp3`  
**Duration:** Variable (R2-D2 startup sound)  
**Source Node:** `gesture_intent_node`  
**Trigger:** Speech session connected after index_finger_up gesture accepted

**Preconditions:**
- Gesture: `index_finger_up` detected by perception
- Gating: `person_status = "red"` (authorized person)
- State: `session_active = false` (can start)
- Cooldown: >= 5.0s since last start trigger
- Service call: `/r2d2/speech/start_session` succeeded

**Playback Timing:**
- Plays immediately after receiving `session_status = "connected"`
- Confirms to user that conversation has started
- User can begin speaking immediately

---

#### Audio Alert 3.1.4: Session Stop Beep

**File:** `Voicy_R2-D2 - 20.mp3`  
**Duration:** Variable (R2-D2 shutdown sound)  
**Source Node:** `gesture_intent_node`  
**Trigger:** Speech session disconnected

**Stop Conditions (Multiple Paths):**

1. **Manual Stop (Fist Gesture):**
   - Fist detected while `session_active = true`
   - Grace period elapsed (>5s since start)
   - Cooldown elapsed (>3s since last trigger)
   - Service call: `/r2d2/speech/stop_session`

2. **VAD Silence Timeout:**
   - 60s consecutive silence (no speech detected)
   - Automatic service stop by gesture_intent_node
   - Reason: "vad_silence_timeout"

3. **Watchdog Timeout (IDLE failsafe):**
   - 35s consecutive person absence (status != "red")
   - Only during IDLE state (not SPEAKING)
   - Automatic service stop
   - Reason: "watchdog"

4. **External Disconnect:**
   - OpenAI API connection lost
   - Network error
   - Manual service stop via ROS command

**Purpose:** Confirms conversation has ended, system ready for next interaction

---

### 3.2 Visual Feedback (LED Status)

**Node:** `status_led_node` (r2d2_audio package)  
**Hardware:** White LED panel (16 SMD LEDs, 3V DC, 20-50mA total)  
**Control Method:** Simple on/off via single GPIO pin  
**Update Rate:** 10 Hz (synchronized with person_status topic)

**GPIO Configuration:**
```
GPIO Pin: 17 (BCM numbering)
Physical Pin: 22 (on 40-pin expansion header)
Voltage: 3.3V logic level (Jetson GPIO output)
Current: ~20-50mA (current-limited by LED panel)

Wiring:
  - Red wire   → Pin 1 or 17 (3.3V power)
  - Blue wire  → Pin 22 (GPIO 17 control)
  - Black wire → Pin 6 (GND)
```

**LED State Mapping:**

| Person Status State | GPIO 17 State | Visual Effect | Meaning |
|---------------------|---------------|---------------|---------|
| 🔴 RED (recognized) | HIGH (3.3V) | 💡 LED ON | Trained person recognized and present |
| 🔵 BLUE (lost/idle) | LOW (0V) | ⚫ LED OFF | No person or person lost |
| 🟢 GREEN (unknown) | LOW (0V) | ⚫ LED OFF | Unknown person detected |

**Synchronization:**
- Subscribes to `/r2d2/audio/person_status` (10 Hz JSON)
- Updates LED on EVERY status message
- No delay between status change and LED update
- Instant response to state transitions

**Purpose:**
- Provides continuous visual feedback of recognition status
- LED ON = system recognizes you, gestures will work
- LED OFF = system doesn't recognize you OR no person present

**Alternative (RGB LED Mode - Legacy):**
If using RGB LED instead of white LED:
- RED state → RED LED (GPIO 17)
- GREEN state → GREEN LED (GPIO 27)
- BLUE state → BLUE LED (GPIO 22)

---

### 3.3 Speech System Audio (Conversational)

**Node:** `speech_node` (r2d2_speech package)  
**Purpose:** Bidirectional audio streaming for natural conversation  
**API:** OpenAI Realtime API (WebSocket)

**Audio Input Pipeline:**
```
HyperX QuadCast S USB Microphone
  ↓
PyAudio Capture (48000 Hz stereo, PCM16)
  ↓
Streaming Resampler (48000 Hz → 24000 Hz mono)
  ↓
Base64 Encoding
  ↓
WebSocket Stream to OpenAI
  ↓
OpenAI Whisper-1 (Speech-to-Text)
```

**Audio Output Pipeline:**
```
OpenAI TTS (Text-to-Speech)
  ↓
WebSocket Stream from OpenAI (24000 Hz mono, base64)
  ↓
Base64 Decoding
  ↓
Streaming Resampler (24000 Hz → 44100 Hz)
  ↓
PyAudio Playback (44100 Hz, device native)
  ↓
PAM8403 Amplifier + Speaker
```

**Latency Breakdown (End-to-End):**
```
Total: 700-1200ms (user speech → AI audio response)

Components:
1. Audio capture buffering: ~20ms
2. Resampling (48k→24k): ~10ms
3. Network upload to OpenAI: 50-100ms
4. Whisper transcription: 200-400ms
5. GPT-4o response generation: 200-400ms
6. TTS audio synthesis: 100-200ms
7. Network download from OpenAI: 50-100ms
8. Resampling (24k→44.1k): ~10ms
9. Audio playback buffering: ~50ms
```

**Voice Configuration:**
- Default Voice: `sage` (slightly synthetic, robotic)
- Available Voices: alloy, echo, fable, onyx, nova, shimmer, sage
- Personality: R2-D2 from Star Wars (efficient, machine-like cadence)

---

## SECTION 4: SYSTEM SERVICES & NODES

### 4.1 Auto-Start Services (Production)

All services configured via systemd with auto-start on boot.

---

#### Service 4.1.1: r2d2-camera-perception.service

**Systemd File:** `/etc/systemd/system/r2d2-camera-perception.service`  
**Type:** Forking (background launch script)  
**Dependencies:** network.target (requires network)

**Launches:**
- `camera_node` (OAK-D Lite camera driver)
- `image_listener` (perception pipeline)

**Functions:**
- Camera driver: 30 FPS RGB capture (1920×1080 → published)
- Face detection: Haar Cascade at 13 Hz with hysteresis (2s/5s)
- Face recognition: LBPH at 6.5 Hz when enabled (frame_skip=2, threshold=70.0)
- Gesture recognition: MediaPipe + SVM at ~10 Hz when enabled (frame_skip=3)

**Resource Usage:**
- CPU: 10-18% (combined camera + perception)
- RAM: ~250 MB
- Boot Time: 5-7 seconds to full operation

**Topics Published:**
- `/oak/rgb/image_raw` (sensor_msgs/Image, 30 Hz)
- `/r2d2/perception/brightness` (std_msgs/Float32, 13 Hz)
- `/r2d2/perception/face_count` (std_msgs/Int32, 13 Hz, smoothed)
- `/r2d2/perception/person_id` (std_msgs/String, 6.5 Hz when enabled)
- `/r2d2/perception/face_confidence` (std_msgs/Float32, 6.5 Hz when enabled)
- `/r2d2/perception/is_target_person` (std_msgs/Bool, 6.5 Hz when enabled)
- `/r2d2/perception/gesture_event` (std_msgs/String, event-driven when enabled)

**Control Commands:**
```bash
# Status
sudo systemctl status r2d2-camera-perception.service

# Logs
sudo journalctl -u r2d2-camera-perception.service -f

# Restart
sudo systemctl restart r2d2-camera-perception.service
```

---

#### Service 4.1.2: r2d2-audio-notification.service

**Systemd File:** `/etc/systemd/system/r2d2-audio-notification.service`  
**Type:** Forking  
**Dependencies:** network.target

**Launches:**
- `audio_notification_node` (state machine RED/BLUE/GREEN)
- `status_led_node` (GPIO LED control)
- `database_logger_node` (event logging)

**Functions:**
- Person status state machine (15s RED timer, 2s/3s GREEN/BLUE smoothing)
- MP3 audio alerts via ffplay (volume=0.30)
- GPIO LED control (Pin 17, 10 Hz updates)
- Event logging to console (future: SQLite)

**Resource Usage:**
- CPU: 2-4%
- RAM: ~100 MB

**Topics Published:**
- `/r2d2/audio/person_status` (std_msgs/String, JSON, 10 Hz)
- `/r2d2/audio/notification_event` (std_msgs/String, event-driven)
- `/r2d2/audio/status` (std_msgs/String, event-driven)

**Topics Subscribed:**
- `/r2d2/perception/person_id` (for state machine)
- `/r2d2/perception/face_count` (for GREEN/BLUE smoothing)

---

#### Service 4.1.3: r2d2-gesture-intent.service

**Systemd File:** `/etc/systemd/system/r2d2-gesture-intent.service`  
**Type:** Exec  
**Dependencies:** network.target, r2d2-camera-perception.service (required)

**Launches:**
- `gesture_intent_node` (gesture-to-speech control)

**Functions:**
- Gesture event processing with multi-level gating
- Service calls to speech system (start/stop)
- Watchdog timer (35s idle failsafe during IDLE)
- VAD-based timeout (60s silence during SPEAKING)
- Audio feedback (R2-D2 start/stop beeps)

**Resource Usage:**
- CPU: <1%
- RAM: ~50 MB

**Service Clients:**
- `/r2d2/speech/start_session` (std_srvs/Trigger)
- `/r2d2/speech/stop_session` (std_srvs/Trigger)

**Topics Subscribed:**
- `/r2d2/perception/gesture_event` (gesture commands)
- `/r2d2/audio/person_status` (gating + watchdog)
- `/r2d2/speech/session_status` (session state tracking)
- `/r2d2/speech/voice_activity` (VAD for timeout)

---

#### Service 4.1.4: r2d2-heartbeat.service

**Systemd File:** `/etc/systemd/system/r2d2-heartbeat.service`  
**Type:** Forking  
**Dependencies:** network.target

**Launches:**
- `heartbeat_node` (system alive ping)

**Functions:**
- Publishes lightweight heartbeat (timestamp + status)
- System metrics available via REST API (not in topic)

**Resource Usage:**
- CPU: <0.5%
- RAM: ~10 MB

**Topics Published:**
- `/r2d2/heartbeat` (std_msgs/String, JSON, 1 Hz)

**Message Format:**
```json
{
  "timestamp": "2025-12-21T10:30:00",
  "status": "running"
}
```

**Note:** System metrics (CPU%, GPU%, Disk%, Temperature) moved to REST API endpoint `/api/system/health` to reduce ROS topic overhead.

---

#### Service 4.1.5: r2d2-speech-node.service (Optional)

**Systemd File:** `/etc/systemd/system/r2d2-speech-node.service`  
**Type:** Forking  
**Dependencies:** network.target  
**Auto-Start:** Optional (can be enabled/disabled)

**Launches:**
- `speech_node` (OpenAI Realtime API integration - lifecycle node)

**Functions:**
- OpenAI Realtime API WebSocket connection
- Speech-to-Text (Whisper-1)
- LLM response (GPT-4o)
- Text-to-Speech (OpenAI TTS)
- Conversation persistence (SQLite)

**Resource Usage:**
- CPU: 10-15% when active, ~5% when idle
- RAM: ~150 MB

**Configuration:**
- `auto_start: false` (session remains disconnected until gesture trigger)
- Voice: `sage`
- Model: `gpt-4o-realtime-preview-2024-12-17`
- Database: `~/dev/r2d2/r2d2_speech/data/conversations.db`

**Services Provided:**
- `/r2d2/speech/start_session` (std_srvs/Trigger)
- `/r2d2/speech/stop_session` (std_srvs/Trigger)

**Topics Published:**
- `/r2d2/speech/session_status` (std_msgs/String, JSON, on-change)
- `/r2d2/speech/user_transcript` (std_msgs/String, event-driven)
- `/r2d2/speech/assistant_transcript` (std_msgs/String, event-driven)
- `/r2d2/speech/voice_activity` (std_msgs/String, JSON, on-change)

---

### 4.2 On-Demand Services

#### Service 4.2.1: r2d2-camera-stream.service

**Purpose:** MJPEG video stream for web dashboard  
**Port:** 8081  
**Exclusivity:** Cannot run with r2d2-camera-perception (device conflict)  
**Auto-Start:** No (manual or dashboard-triggered)

#### Service 4.2.2: r2d2-rosbridge.service

**Purpose:** WebSocket bridge to ROS topics  
**Port:** 9090  
**Auto-Start:** No (on-demand for web dashboard)

#### Service 4.2.3: r2d2-web-dashboard.service

**Purpose:** FastAPI REST API + web UI  
**Port:** 8080  
**Auto-Start:** No (on-demand for monitoring)

---

## SECTION 5: COMPLETE TIMING REFERENCE

### 5.1 Perception Layer Timings

| Parameter | Value | Unit | Purpose |
|-----------|-------|------|---------|
| **Camera Frame Rate** | 30 | Hz | OAK-D native capture rate |
| **Perception Processing** | 13 | Hz | Downscaled processing rate |
| **Face Presence Threshold** | 2.0 | seconds | Continuous detection before confirming presence |
| **Face Absence Threshold** | 5.0 | seconds | Continuous loss before confirming absence |
| **Recognition Frame Skip** | 2 | frames | Process every 2nd frame |
| **Recognition Rate** | 6.5 | Hz | Effective recognition rate (13 ÷ 2) |
| **Recognition Confidence** | 70.0 | threshold | LBPH distance threshold (lower = stricter) |
| **Gesture Frame Skip** | 3 | frames | Process every 3rd frame |
| **Gesture Rate** | ~10 | Hz | Effective gesture rate (30 ÷ 3) |
| **Gesture Confidence** | 0.7 | threshold | SVM probability threshold (0.0-1.0) |

---

### 5.2 State Machine Timings

| Parameter | Value | Unit | Purpose |
|-----------|-------|------|---------|
| **RED Status Timeout** | 15.0 | seconds | Timer resets on each recognition |
| **GREEN Entry Delay** | 2.0 | seconds | BLUE→GREEN transition smoothing |
| **BLUE Entry Delay** | 3.0 | seconds | GREEN→BLUE transition smoothing |
| **Recognition Cooldown** | 2.0 | seconds | Min between "Hello!" beeps |
| **Post-Loss Quiet Period** | 5.0 | seconds | Quiet after "Lost you!" before next "Hello!" |
| **Status Publish Rate** | 10 | Hz | Continuous person_status publishing |
| **Loss Check Timer** | 0.5 | seconds | State machine check interval |

---

### 5.3 Gesture Control Timings

| Parameter | Value | Unit | Purpose |
|-----------|-------|------|---------|
| **Start Gesture Cooldown** | 5.0 | seconds | Min between index_finger_up triggers |
| **Stop Gesture Cooldown** | 3.0 | seconds | Min between fist triggers |
| **Speaking Grace Period** | 5.0 | seconds | Ignore fist after start (hand lowering protection) |
| **VAD Silence Timeout** | 60.0 | seconds | Consecutive silence before auto-stop |
| **Watchdog Idle Timeout** | 35.0 | seconds | Person absence before auto-stop (IDLE only) |
| **Watchdog Check Interval** | 10.0 | seconds | Timer callback frequency |

---

### 5.4 Audio Timings

| Parameter | Value | Unit | Purpose |
|-----------|-------|------|---------|
| **Audio Volume** | 0.30 | ratio | Global volume (30% of max) |
| **Recognition Beep** | ~2.0 | seconds | Duration of "Hello!" beep |
| **Loss Beep** | ~5.0 | seconds | Duration of "Lost you!" beep |
| **Total Loss Latency** | ~20 | seconds | Time from leaving to loss beep |

---

### 5.5 Speech System Timings

| Parameter | Value | Unit | Purpose |
|-----------|-------|------|---------|
| **End-to-End Latency** | 700-1200 | ms | User speech → AI response |
| **Transcription (Whisper)** | 200-400 | ms | Speech-to-text processing |
| **Response (GPT-4o)** | 200-400 | ms | LLM generation |
| **TTS Synthesis** | 100-200 | ms | Text-to-speech |
| **Network Upload** | 50-100 | ms | Audio to OpenAI |
| **Network Download** | 50-100 | ms | Response from OpenAI |

---

## SECTION 6: COMPLETE EVENT TIMELINE

### Scenario: User Conversation with Manual Stop

**Timeline with exact timings based on documented specifications:**

```
t=0.0s   User appears in camera view
         → Raw detection: face detected (Haar Cascade)
         → Hysteresis: Start presence timer (need 2s continuous)

t=2.0s   Face presence threshold met (2.0s continuous detection)
         → Smoothed face_count: 0 → 1 (published to /r2d2/perception/face_count)
         → Recognition starts processing (frame_skip=2, every 2nd frame)

t=2.1s   First recognition frame processed (6.5 Hz rate)
         → LBPH confidence: 42.3 (< 70.0 threshold)
         → person_id: "no_person" → "severin" (published)

t=2.1s   audio_notification_node receives person_id="severin"
         → State: "blue" → "red" (instant transition)
         → RED timer starts (15s, resets on each recognition)
         → Check cooldowns: no previous beep, OK to play
         → Audio: Play "Voicy_R2-D2 - 2.mp3" ("Hello!" beep)
         → Publish: person_status = {"status": "red", ...}

t=2.1s   status_led_node receives person_status
         → GPIO 17: LOW → HIGH
         → Visual: LED OFF → ON
         → User sees: 💡 LED ON (system recognizes me)

t=4.0s   User raises index finger (pointing upward)
         → MediaPipe Hands detects hand pose
         → SVM classifies: "index_finger_up" (confidence: 0.87 > 0.7)
         → State change: None → "index_finger_up"
         → Publish: gesture_event = "index_finger_up"

t=4.0s   gesture_intent_node receives gesture_event
         → Gate 1: person_status = "red" ✓ (authorized)
         → Gate 2: session_active = false ✓ (can start)
         → Gate 3: time_since_last = infinity ✓ (no cooldown)
         → Action: Call service /r2d2/speech/start_session

t=4.5s   speech_node processes start_session request
         → Lifecycle: Inactive → Active
         → Connect WebSocket to OpenAI (wss://api.openai.com/v1/realtime)
         → Initialize audio streams (mic + speaker)
         → Publish: session_status = {"status": "connected"}
         → Return: success=true, message="Session started"

t=4.5s   gesture_intent_node receives session_status
         → Update: session_active = false → true
         → Enter SPEAKING state
         → speaking_start_time = now (grace period starts)
         → Audio: Play "Voicy_R2-D2 - 16.mp3" (start beep)
         → VAD monitoring active

t=4.5-9.5s   Speaking grace period active (5 seconds)
         → Any fist gestures → IGNORED
         → Log: "Fist ignored: grace period (X.Xs < 5.0s)"
         → Purpose: Prevent false positive from hand lowering

t=10.0s  User speaks: "Hello R2D2, how are you today?"
         → HyperX mic captures audio (48kHz)
         → Resampled to 24kHz, base64 encoded
         → Streamed to OpenAI via WebSocket
         → OpenAI VAD: speech_started event
         → Publish: voice_activity = {"status": "speaking"}
         → gesture_intent_node: VAD timer PAUSED

t=12.0s  User finishes speaking
         → OpenAI VAD: speech_stopped event
         → Publish: voice_activity = {"status": "silent"}
         → gesture_intent_node: VAD timer STARTS (60s countdown)

t=12.3s  OpenAI Whisper transcribes speech
         → Publish: user_transcript = "Hello R2D2, how are you today?"

t=12.5s  GPT-4o generates response
         → Publish: assistant_transcript = "I'm functioning optimally..."

t=12.7s  OpenAI TTS synthesizes audio response
         → Audio chunks streamed via WebSocket (24kHz base64)
         → Resampled to 44.1kHz, played through speaker

t=13.5s  AI response playback completes
         → User can respond immediately (conversation continues)

t=30.0s  User makes fist gesture
         → MediaPipe + SVM: "fist" (confidence: 0.81 > 0.7)
         → Publish: gesture_event = "fist"

t=30.0s  gesture_intent_node receives fist
         → Gate 1: person_status = "red" ✓
         → Gate 2: session_active = true ✓
         → Gate 3: grace period = 25.5s > 5.0s ✓
         → Gate 4: cooldown = 25.5s > 3.0s ✓
         → Action: Call service /r2d2/speech/stop_session

t=30.1s  speech_node processes stop_session
         → Stop audio streaming
         → Disconnect WebSocket
         → Lifecycle: Active → Inactive
         → Publish: session_status = {"status": "disconnected", "reason": "manual_stop"}

t=30.1s  gesture_intent_node receives disconnect
         → Update: session_active = true → false
         → Exit SPEAKING state (reason: "manual_stop")
         → Audio: Play "Voicy_R2-D2 - 20.mp3" (stop beep)
         → Reset timers, enter IDLE state

t=45.0s  User walks away from camera
         → Face detection continues for a moment
         → Recognition: "severin" → "unknown"
         → Hysteresis: Start absence timer (need 5s continuous)

t=50.0s  Face absence threshold met (5.0s continuous loss)
         → Smoothed face_count: 1 → 0 (published)
         → person_id: "unknown" → "no_person"

t=17.1s  RED timeout expires (15s since last recognition at t=2.1s)
         → Check: face_count = 0 (no face present)
         → State: "red" → "blue" (immediate, no delay since face_count=0)
         → Audio: Play "Voicy_R2-D2 - 5.mp3" ("Lost you!" beep)
         → Publish: person_status = {"status": "blue", ...}

t=17.1s  status_led_node receives BLUE status
         → GPIO 17: HIGH → LOW
         → Visual: LED ON → OFF
         → User perception: System lost recognition

t=17.1s  gesture_intent_node receives BLUE status
         → Watchdog: last_red_status_time = now (35s timer starts)
         → Note: Speech already stopped at t=30.1s, so no action needed
```

**Key Observations:**
- Total time from user leaving (t=45s) to loss beep: ~25s
  - 5s perception hysteresis (t=45-50s)
  - RED timer already expired at t=17.1s (15s from last recognition at t=2.1s)
- Grace period (5s) prevented accidental fist during hand lowering
- VAD-based timeout never triggered (user stopped manually)
- Watchdog timer started but was unnecessary (session already stopped)

---

## SECTION 7: SYSTEM HEALTH MONITORING

### 7.1 Live Topic Monitoring

**Person Recognition Stream:**
```bash
# Raw person ID
ros2 topic echo /r2d2/perception/person_id --no-arr

# Color-coded person status
ros2 topic echo /r2d2/audio/person_status --no-arr | \
  grep -oP '"status":\s*"\K\w+' --line-buffered | \
  while read status; do
    case $status in
      red)   echo -e "\033[1;31m🔴 RED - Target person recognized (LED ON)\033[0m" ;;
      blue)  echo -e "\033[1;34m🔵 BLUE - No person detected (LED OFF)\033[0m" ;;
      green) echo -e "\033[1;32m🟢 GREEN - Unknown person detected (LED OFF)\033[0m" ;;
      *)     echo "⚪ $status" ;;
    esac
  done
```

**Gesture Events:**
```bash
# Raw gesture events
ros2 topic echo /r2d2/perception/gesture_event --no-arr

# Color-coded with icons
ros2 topic echo /r2d2/perception/gesture_event --no-arr | \
  grep -oP "data: '\K[^']+" --line-buffered | \
  while read gesture; do
    case $gesture in
      index_finger_up) echo -e "\033[1;36m☝️  INDEX FINGER UP - Start conversation\033[0m" ;;
      fist)            echo -e "\033[1;35m✊ FIST - Stop conversation\033[0m" ;;
      *)               echo "👋 $gesture" ;;
    esac
  done
```

**Speech Session Status:**
```bash
# Session status
ros2 topic echo /r2d2/speech/session_status --no-arr

# Color-coded
ros2 topic echo /r2d2/speech/session_status --no-arr | \
  grep -oP '"status":\s*"\K\w+' --line-buffered | \
  while read status; do
    case $status in
      connected)    echo -e "\033[1;32m🎤 CONNECTED - Conversation active\033[0m" ;;
      disconnected) echo -e "\033[1;31m🔇 DISCONNECTED - Conversation ended\033[0m" ;;
      *)            echo "? $status" ;;
    esac
  done
```

---

### 7.2 Service Status Monitoring

```bash
# Check all auto-start services
systemctl is-active r2d2-camera-perception \
                     r2d2-audio-notification \
                     r2d2-gesture-intent \
                     r2d2-heartbeat

# Follow logs in real-time
sudo journalctl -u r2d2-gesture-intent -f
sudo journalctl -u r2d2-audio-notification -f
sudo journalctl -u r2d2-camera-perception -f

# Combined service status
systemctl status r2d2-*.service
```

---

### 7.3 Performance Baselines

| Metric | Expected Value | Issue When | Notes |
|--------|----------------|------------|-------|
| Camera FPS | 30 Hz | <25 Hz | USB bandwidth or camera issue |
| Perception Rate | 13 Hz | <10 Hz | CPU overload |
| Recognition Rate | 6.5 Hz | <5 Hz | Recognition enabled but slow |
| Status Publish | 10 Hz | <5 Hz | Audio node performance issue |
| Total CPU | 15-25% | >50% | System overload |
| Total RAM | ~500 MB | >2 GB | Memory leak |
| GPU | 0% | N/A | Not used in Phase 1 |

---

## SECTION 8: INTEGRATION SUMMARY

### 8.1 Data Flow Overview

```
Hardware Layer (Physical Sensors/Actuators)
  ↓
Camera (30 FPS RGB) + Microphone (48kHz) + LED (GPIO 17) + Speaker (I2S)
  ↓
ROS 2 Nodes (Perception + State Machines + Control)
  ↓
Topics (Event Streams + Status Updates)
  ↓
Multi-Modal Feedback (Audio Beeps + LED + Speech)
  ↓
User Experience (Seamless Interaction)
```

---

### 8.2 Configuration Files

**Audio Configuration:**
- `audio_params.yaml` - Global volume (0.30)
- `audio_notification.launch.py` - State machine parameters
- Service files reference audio assets directory

**Speech Configuration:**
- `speech_params.yaml` - Voice, model, instructions
- `.env` - OpenAI API key (secure)

**Gesture Configuration:**
- `gesture_intent.launch.py` - Cooldowns, watchdog, grace period
- Auto-resolved gesture models from PersonRegistry

**Person Registry:**
- `persons.db` - SQLite database (~/dev/r2d2/data/)
- Links persons to face/gesture models
- Auto-migration for existing models

---

### 8.3 Key Design Principles

1. **Event-Driven Architecture:** Gestures publish only on state change, not continuous
2. **Strict Gating:** Multiple layers prevent unauthorized access and false triggers
3. **Temporal Smoothing:** Hysteresis filters prevent flicker in all state machines
4. **Multi-Modal Feedback:** Audio + LED + speech for comprehensive UX
5. **Cost Optimization:** Watchdog prevents wasted API calls during absence
6. **State Protection:** SPEAKING state immune to camera flickers (VAD-only timeout)

---

## DOCUMENT METADATA

**Created:** December 21, 2025  
**Based On:** System documentation specifications (not code implementation)  
**Purpose:** Authoritative reference for all system events, states, and feedback  
**Scope:** Complete status architecture from sensors to user feedback  
**Related Documents:**
- `001_ARCHITECTURE_OVERVIEW.md` - Overall system architecture
- `007_SYSTEM_INTEGRATION_REFERENCE.md` - Integration reference
- `100_PERSON_RECOGNITION_REFERENCE.md` - Recognition system details
- `200_SPEECH_SYSTEM_REFERENCE.md` - Speech system details
- `300_GESTURE_SYSTEM_OVERVIEW.md` - Gesture system details

**Cross-Reference:** See `TEMP_DISCREPANCIES_AND_RECOMMENDATIONS.md` for differences between this documented specification and actual code implementation.

---

END OF DOCUMENT

