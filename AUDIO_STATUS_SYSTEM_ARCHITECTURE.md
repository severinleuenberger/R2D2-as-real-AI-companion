# R2D2 Audio Status System - Architecture & Implementation

## 1. Overview

The R2D2 Audio Status System provides real-time person recognition state management with integrated visual feedback and conversation context tracking. The system implements a three-state recognition model (RED/BLUE/GREEN) that drives both immediate visual/audio feedback and future dialogue context.

**Status: ✅ IMPLEMENTED & TESTED**

---

## 2. System Architecture

### 2.1 Three-State Recognition Model

```
┌─────────────────────────────────────────────────────────────────┐
│                       RECOGNITION STATES                         │
├─────────────────────────────────────────────────────────────────┤
│                                                                   │
│  🔴 RED    = Target Person Recognized (ACTIVE ENGAGEMENT)       │
│             └─ Duration: 15+ seconds (continuously updated)      │
│             └─ LED: Solid RED                                    │
│             └─ Context: Known person in conversation             │
│                                                                   │
│  🔵 BLUE   = No Person Recognized (IDLE/WAITING)                │
│             └─ Triggered: After 5s jitter + 15s confirmation    │
│             └─ LED: Solid BLUE                                   │
│             └─ Context: System idle, awaiting recognition        │
│                                                                   │
│  🟢 GREEN  = Unknown Person Detected (CAUTION)                  │
│             └─ Duration: While unknown person visible            │
│             └─ LED: Solid GREEN                                  │
│             └─ Context: Non-target person present                │
│                                                                   │
└─────────────────────────────────────────────────────────────────┘
```

### 2.2 System Flow Diagram

```
Face Recognition Input
/r2d2/perception/person_id (String)
        │
        │ "severin" | "unknown" | other
        │
        ▼
┌─────────────────────────────────────┐
│  Audio Notification Node            │
│  (State Machine + Core Logic)       │
│                                     │
│  • Tracks: is_currently_recognized  │
│  • Manages: timers, jitter windows  │
│  • Emits: Status transitions        │
└──────────────┬──────────────────────┘
               │
               │ Publishes JSON Status
               │ /r2d2/audio/person_status
               │ {status, person_identity,
               │  timestamp, confidence, duration}
               │
       ┌───────┴──────────┬──────────┬──────────┐
       │                  │          │          │
       ▼                  ▼          ▼          ▼
    ┌─────┐           ┌───────┐  ┌─────────┐ ┌──────┐
    │ LED │           │ STT   │  │ Context │ │Future│
    │ RGB │           │ LLM   │  │ Logger  │ │  DB  │
    │ GPIO│           │ TTS   │  │(Console)│ │(impl)│
    └─────┘           └───────┘  └─────────┘ └──────┘
    Visual            Dialogue    Console   Persistence
    Feedback         Pipeline     Logs     (SQLite)
```

### 2.3 Node Responsibilities

| Node | Purpose | Input | Output | Status |
|------|---------|-------|--------|--------|
| **audio_notification_node** | Core state machine, audio alerts | `/r2d2/perception/person_id` | `/r2d2/audio/person_status` (JSON) | ✅ Implemented |
| **status_led_node** | Visual feedback controller | `/r2d2/audio/person_status` | GPIO pins (RED/GREEN/BLUE) | ✅ Implemented |
| **database_logger_node** | Event logging (future DB) | `/r2d2/audio/person_status` | Console logs + JSON records | ✅ Structure only |

---

## 3. State Machine Details

### 3.1 Timing Configuration

```python
# Timing parameters (in audio_notification_node)
jitter_tolerance_seconds = 5.0        # Brief gap tolerance
loss_confirmation_seconds = 15.0      # Confirmation window AFTER jitter
cooldown_seconds = 2.0                # Min between repeated alerts
```

### 3.2 State Transitions

```
INITIAL (BLUE)
    │
    ├─ "severin" detected ────────────────────→ RECOGNIZED (RED)
    │                                          • Publish: RED status
    │                                          • Play: "Hello!" beep
    │                                          • Update: start 15s window
    │
    └─ "unknown" detected ──────────────────→ UNKNOWN (GREEN)
                                              • Publish: GREEN status
                                              • NO beep
                                              • Does NOT affect RED/BLUE


RECOGNIZED (RED)
    │
    ├─ "severin" continuously visible ─→ Stay RED (no action)
    │  (< 5s gap)                        (Update duration timer)
    │
    ├─ Brief gap (5-20s total) ────────→ Stay RED
    │  (Jitter tolerance)                (Waiting to confirm loss)
    │
    └─ No "severin" for 20s total ─────→ LOST (BLUE)
       (5s jitter + 15s confirmation)    • Publish: BLUE status
                                          • Play: "Oh, I lost you!" beep
                                          • Reset all timers


UNKNOWN (GREEN)
    │
    ├─ "severin" appears ──────────────→ RECOGNIZED (RED)
    │                                    (Takes priority over GREEN)
    │
    └─ Unknown person leaves ──────────→ Back to previous state
                                         (BLUE or RED)
```

### 3.3 Beep Behavior (Anti-Spam)

| Event | Condition | Beep? | Cooldown | Next Trigger |
|-------|-----------|-------|----------|--------------|
| Recognition | Transition BLUE→RED | ✅ "Hello!" | 2s | Not within 15s window |
| Re-recognition | Transition BLUE→RED after loss | ✅ "Hello!" | 2s | Same as above |
| Loss | 5s jitter + 15s confirmed | ✅ "Lost you!" | 2s | Not until re-recognized |
| Jitter gap | Brief interruption (< 5s) | ❌ No beep | — | Transparent to user |

**Key**: Once RED status active, NO recognition beeps for 15 seconds. Only state transitions trigger beeps.

---

## 4. Person Status Message Format

### 4.1 JSON Message Structure

Publishes to `/r2d2/audio/person_status` as `std_msgs/String` containing JSON:

```json
{
  "status": "red|blue|green",
  "person_identity": "severin|unknown|no_person",
  "timestamp_sec": 1765209394,
  "timestamp_nanosec": 621522319,
  "confidence": 0.95,
  "duration_in_state": 15.3
}
```

### 4.2 Field Descriptions

| Field | Type | Range | Purpose |
|-------|------|-------|---------|
| `status` | string | red/blue/green | Current recognition state |
| `person_identity` | string | severin/unknown/no_person | Identified person |
| `timestamp_sec` | int | 0-inf | ROS 2 timestamp (seconds) |
| `timestamp_nanosec` | int | 0-999999999 | ROS 2 timestamp (nanoseconds) |
| `confidence` | float | 0.0-1.0 | Face detection confidence |
| `duration_in_state` | float | 0.0-inf | How long in current state (seconds) |

### 4.3 ROS 2 Topic

```bash
# Subscribe to status
ros2 topic echo /r2d2/audio/person_status

# Example output:
data: '{"status": "red", "person_identity": "severin", "timestamp_sec": 1765209394, "timestamp_nanosec": 621522319, "confidence": 0.95, "duration_in_state": 15.3}'
```

---

## 5. LED Controller (status_led_node)

### 5.1 GPIO Pin Configuration

```python
led_pin_red = 17      # GPIO 17 for RED LED
led_pin_green = 27    # GPIO 27 for GREEN LED
led_pin_blue = 22     # GPIO 22 for BLUE LED
```

### 5.2 Color Mapping

| Status | LED Color | Meaning |
|--------|-----------|---------|
| `red` | 🔴 RED solid | Target person actively recognized |
| `blue` | 🔵 BLUE solid | No one recognized, system idle |
| `green` | 🟢 GREEN solid | Unknown/non-target person present |

### 5.3 Features

- ✅ Real-time response to status changes
- ✅ Hardware GPIO support (Jetson AGX Orin)
- ✅ Automatic fallback to simulation mode if GPIO unavailable
- ✅ Brightness control parameter (0.0-1.0)
- ✅ Enable/disable parameter

---

## 6. Database Logger (database_logger_node)

### 6.1 Current Implementation

**Status**: Structure only - ready for future SQLite implementation

- ✅ Listens to status messages
- ✅ Logs events to console
- ✅ Tracks state transitions
- ✅ Counts events with sequence numbers

### 6.2 Future Database Schema

When database implementation is enabled, the following tables will be created:

#### recognition_events Table

```sql
CREATE TABLE recognition_events (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    timestamp DATETIME NOT NULL,
    person_identity TEXT NOT NULL,
    status TEXT NOT NULL,
    confidence REAL,
    duration_in_state REAL,
    created_at DATETIME DEFAULT CURRENT_TIMESTAMP,
    INDEX idx_person (person_identity),
    INDEX idx_timestamp (timestamp),
    INDEX idx_status (status)
);
```

**Example data:**
```sql
| timestamp           | person_identity | status | confidence | duration |
|---------------------|-----------------|--------|------------|----------|
| 2025-12-08 14:30:00 | severin         | red    | 0.95       | 15.3     |
| 2025-12-08 14:30:20 | no_person       | blue   | 0.0        | 25.1     |
| 2025-12-08 14:30:45 | severin         | red    | 0.92       | 8.7      |
```

#### conversation_context Table

```sql
CREATE TABLE conversation_context (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    timestamp DATETIME NOT NULL,
    person_identity TEXT NOT NULL,
    person_status TEXT NOT NULL,
    transcript TEXT,
    response TEXT,
    conversation_turn INTEGER,
    created_at DATETIME DEFAULT CURRENT_TIMESTAMP,
    FOREIGN KEY (person_identity) REFERENCES recognition_events(person_identity)
);
```

**Example data (for STT-LLM-TTS integration):**
```sql
| timestamp           | person_identity | person_status | transcript              | response |
|---------------------|-----------------|---------------|-------------------------|----------|
| 2025-12-08 14:30:05 | severin         | red           | "Hello R2D2!"          | "Hello Severin, what can I do for you?" |
| 2025-12-08 14:30:15 | severin         | red           | "What's the weather?"  | "It's sunny today..." |
| 2025-12-08 14:31:00 | unknown         | green         | [Unknown person nearby] | [No response, waiting for severin] |
```

### 6.3 Query Examples

**Get all conversations with severin (RED status):**
```sql
SELECT timestamp, transcript, response FROM conversation_context
WHERE person_identity = 'severin' AND person_status = 'red'
ORDER BY timestamp DESC
LIMIT 10;
```

**Get recognition timeline for today:**
```sql
SELECT timestamp, person_identity, status, duration_in_state FROM recognition_events
WHERE DATE(timestamp) = DATE('now')
ORDER BY timestamp;
```

**Get conversation history from last 30 minutes:**
```sql
SELECT person_identity, person_status, transcript, response FROM conversation_context
WHERE timestamp > datetime('now', '-30 minutes')
ORDER BY timestamp;
```

**Count interactions per person:**
```sql
SELECT person_identity, COUNT(*) as interaction_count FROM recognition_events
WHERE person_status = 'red'
GROUP BY person_identity
ORDER BY interaction_count DESC;
```

---

## 7. Integration with STT-LLM-TTS Pipeline

### 7.1 Context Injection

The STT-LLM-TTS pipeline should subscribe to `/r2d2/audio/person_status` to inject person context:

```python
def status_callback(self, msg: String):
    """Inject person context into LLM prompts."""
    status_data = json.loads(msg.data)
    person = status_data['person_identity']
    state = status_data['status']
    
    if state == 'red':
        # Known person, maintain conversation context
        self.current_context = f"Conversing with {person}"
    elif state == 'green':
        # Unknown person, separate context
        self.current_context = f"Unknown person nearby, waiting for {person}"
    elif state == 'blue':
        # No one, reset context
        self.current_context = "Idle, no one around"
```

### 7.2 Conversation Tagging

All STT-LLM-TTS messages should be tagged with status:

```python
# When publishing conversation to database:
conversation_record = {
    'timestamp': timestamp,
    'person_identity': status['person_identity'],
    'person_status': status['status'],  # "red"|"blue"|"green"
    'transcript': user_input,
    'response': llm_output,
    'confidence': status['confidence']
}
```

### 7.3 Memory & Recall

Status enables person-specific memory:

```python
# Query similar conversations with same person:
SELECT response FROM conversation_context
WHERE person_identity = 'severin' AND status = 'red'
  AND DATE(timestamp) = DATE('now')
ORDER BY timestamp DESC
LIMIT 5;

# Use these to inform LLM about conversation history with Severin
```

---

## 8. Global ROS 2 Parameters

All parameters can be set at runtime via ROS 2 parameter server:

```bash
# Audio Notification Node
ros2 param set /audio_notification_node target_person "severin"
ros2 param set /audio_notification_node audio_volume 0.05
ros2 param set /audio_notification_node jitter_tolerance_seconds 5.0
ros2 param set /audio_notification_node loss_confirmation_seconds 15.0
ros2 param set /audio_notification_node cooldown_seconds 2.0
ros2 param set /audio_notification_node recognition_audio_file "Voicy_R2-D2 - 2.mp3"
ros2 param set /audio_notification_node loss_audio_file "Voicy_R2-D2 - 5.mp3"
ros2 param set /audio_notification_node enabled true

# Status LED Node
ros2 param set /status_led_controller led_pin_red 17
ros2 param set /status_led_controller led_pin_green 27
ros2 param set /status_led_controller led_pin_blue 22
ros2 param set /status_led_controller brightness 1.0
ros2 param set /status_led_controller enabled true
ros2 param set /status_led_controller simulate_gpio false

# Database Logger Node
ros2 param set /database_logger db_path "/home/severin/dev/r2d2/r2d2_conversations.db"
ros2 param set /database_logger enabled true
ros2 param set /database_logger log_to_console true
```

---

## 9. Launching the System

### 9.1 Launch All Services

```bash
cd ~/dev/r2d2/ros2_ws
source install/setup.bash

# Launch all audio services (LED enabled, no GPIO simulation)
ros2 launch r2d2_audio all_audio_services.launch.py

# Or with custom parameters
ros2 launch r2d2_audio all_audio_services.launch.py \
  target_person:=severin \
  audio_volume:=0.05 \
  enable_led:=true \
  simulate_gpio:=false
```

### 9.2 Individual Node Launch

```bash
# Terminal 1: Core audio notification
python3 -m r2d2_audio.audio_notification_node

# Terminal 2: LED controller
python3 -m r2d2_audio.status_led_node

# Terminal 3: Database logger
python3 -m r2d2_audio.database_logger_node
```

---

## 10. Testing & Validation

### 10.1 Test Recognition

```bash
# Terminal 1: Launch nodes
ros2 launch r2d2_audio all_audio_services.launch.py

# Terminal 2: Test with recognition message
ros2 topic pub --once /r2d2/perception/person_id std_msgs/String "{data: severin}"

# Expected:
# 1. Audio: "Hello!" beep plays
# 2. LED: RED lights up
# 3. Status: Published to /r2d2/audio/person_status
# 4. Logs: "✓ severin recognized!"
```

### 10.2 Test Loss Detection

```bash
# Let the system run for 25+ seconds without recognition
# Expected:
# 1. LED: Stays RED for 15s (confirmation window)
# 2. At 20s total: Audio plays "Oh, I lost you!" beep
# 3. LED: Turns BLUE
# 4. Status: Changes to blue/no_person
# 5. Logs: "✗ severin lost (after X.Xs absence, 15.0s in loss window)"
```

### 10.3 Test LED Simulation

```bash
# Run with GPIO simulation:
ros2 launch r2d2_audio all_audio_services.launch.py simulate_gpio:=true

# Expected:
# • No hardware errors
# • LED updates logged to console
# • All statuses work correctly
```

---

## 11. Files & Structure

```
ros2_ws/src/r2d2_audio/
├── r2d2_audio/
│   ├── __init__.py
│   ├── audio_notification_node.py      [Core state machine, status publishing]
│   ├── status_led_node.py              [LED control via GPIO]
│   ├── database_logger_node.py         [Event logging (structure)]
│   ├── audio_player.py                 [Existing MP3 playback]
│   ├── audio_beep_node.py              [Existing beep generation]
│   └── assets/audio/                   [MP3 files]
│       ├── Voicy_R2-D2 - 2.mp3        ["Hello!"]
│       └── Voicy_R2-D2 - 5.mp3        ["Oh, I lost you!"]
│
├── msg/
│   └── PersonStatus.msg                [Message definition - for reference]
│
├── launch/
│   └── all_audio_services.launch.py    [Launch all 3 nodes]
│
├── package.xml                         [ROS 2 package descriptor]
├── setup.py                            [Python package setup]
└── CMakeLists.txt                      [Build configuration]
```

---

## 12. Future Enhancements

### 12.1 Database Implementation

- [ ] Implement SQLite database creation in `database_logger_node.py`
- [ ] Add `_log_to_database()` method with proper error handling
- [ ] Create migration system for schema updates
- [ ] Add database cleanup/archival policies

### 12.2 Multi-Person Support

- [ ] Extend `target_person` to list of `target_persons`
- [ ] Add `person_role` field (family member, friend, visitor)
- [ ] Track interaction patterns per person
- [ ] Customize LED patterns for different people

### 12.3 Advanced Features

- [ ] LED blinking patterns on state transitions
- [ ] Sound visualization (LED reacts to beep intensity)
- [ ] Integration with motion detection
- [ ] Conversation memory with semantic search
- [ ] Analytics dashboard for engagement patterns
- [ ] Voice command to change parameters at runtime

---

## 13. Troubleshooting

| Issue | Cause | Solution |
|-------|-------|----------|
| Status messages not published | Audio node not running | Check logs: `ros2 run r2d2_audio audio_notification_node` |
| LED not changing color | GPIO pins wrong | Check `/dev/gpiomem` permissions, validate GPIO numbers |
| Loss alert fires too early | Loss_confirmation too short | Increase `loss_confirmation_seconds` parameter |
| Loss alert repeats | Cooldown too short | Increase `cooldown_seconds` to 3.0+ |
| Database logger errors | Console logging not enabled | Set `log_to_console:=true` parameter |

---

## 14. Summary

The R2D2 Audio Status System provides:

✅ **Three-state recognition model** (RED/BLUE/GREEN) for clear person context
✅ **Real-time visual feedback** via RGB LED (GPIO-controlled)
✅ **Audio alerts** with proper timing and cooldowns (minimal beeping)
✅ **Status message format** ready for STT-LLM-TTS integration
✅ **Event logging structure** ready for SQLite implementation
✅ **Full parameterization** via ROS 2 parameter server
✅ **Production deployment** via systemd service

**Status**: Ready for integration with STT-LLM-TTS pipeline. Database implementation follows after dialogue system is in place.
