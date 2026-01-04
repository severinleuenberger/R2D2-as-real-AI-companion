# R2D2 User Experience and Functions
## Interactive Mobile Robot System

**Document Version:** 1.1  
**Last Updated:** January 2, 2026 (Added two-stage fist stop documentation)  
**Platform:** NVIDIA Jetson AGX Orin 64GB + ROS 2 Humble

---

## Executive Summary

The R2D2 system is a fully functional interactive mobile robot designed for natural human-robot interaction through multiple modalities. The system provides real-time person recognition, voice-based conversations, intelligent tutoring, and comprehensive remote monitoring capabilities.

**Core Capabilities:**
- Recognizes and tracks people with visual and audio feedback
- Engages in natural spoken conversations triggered by gestures
- Functions as an educational tutor for robotics and programming concepts
- Provides remote monitoring and control via web interface
- Maintains conversation history and learning progress

**Access Methods:**
- Physical interaction (face-to-face)
- Voice commands (hands-free)
- Gesture controls (index finger, fist)
- Web dashboard (remote access via Tailscale VPN)
- Physical buttons (power control)

---

## 1. Person Recognition and Awareness

### What It Does
The robot continuously monitors its environment to detect and recognize people, providing immediate feedback when someone is identified.

### Recognition System ✅ OPERATIONAL

**Face Detection and Recognition**
- Automatically detects faces in camera view within ~460ms
- Recognizes trained individuals with 90-95% accuracy
- Supports multiple trained users without code changes
- Distinguishes between known persons, unknown persons, and no person present

**Visual Feedback**
- White LED panel provides instant visual confirmation
  - LED ON (bright): Person recognized
  - LED OFF: Person lost or unknown
- LED mounted prominently for easy visibility from across the room

**Audio Feedback**
- Recognition alert: Friendly R2D2 beep when person identified
- Loss alert: Different R2D2 sound when person leaves field of view
- Volume adjustable (default: quiet, 2% volume)
- Configurable cooldown periods prevent annoying repetition

**Status States**
- 🔴 **RED (Recognized):** Known person detected - system ready for interaction
- 🟢 **GREEN (Unknown):** Face detected but not recognized - passive monitoring
- 🔵 **BLUE (No Person):** No one in view - idle state

**Multi-User Support**
- Train the robot to recognize multiple people
- Each person gets their own recognition profile
- System automatically authorizes any trained person
- No manual configuration needed after training

### Training System ✅ OPERATIONAL

**Face Recognition Training**
- Interactive training session via camera
- Captures 80+ images from different angles and lighting
- Training takes 2-3 minutes per person
- Model stored permanently in database

**Training Access Methods:**
1. Web dashboard training interface (recommended)
2. Command-line training script
3. Remote training via Tailscale VPN

**For technical details:** See `100_PERCEPTION_STATUS_REFERENCE.md`

**Technical Implementation:** See [`001_ARCHITECTURE_OVERVIEW.md`](001_ARCHITECTURE_OVERVIEW.md) Sections 1-5 (Hardware, Data Flow, Nodes, Audio System, Processing Pipeline)

---

## 2. Natural Language Interaction

### What It Does
Engage in spoken conversations with the robot using natural language. Conversations are triggered by gestures and continue until you explicitly stop or walk away.

### Speech-to-Speech Conversations ✅ OPERATIONAL

**Starting a Conversation**
- Show index finger pointing up (☝️) when robot recognizes you (LED ON)
- Robot responds with acknowledgment beep (~350ms)
- Ready confirmation beep follows (~750ms)
- Speak naturally - robot listens and processes in real-time

**During Conversation**
- Robot transcribes your speech to text (Whisper-1)
- Processes meaning and generates intelligent response (GPT-4o)
- Speaks response back to you with natural voice
- Multi-turn conversations supported - no need to re-trigger
- Robot maintains conversation context across multiple exchanges

**Ending a Conversation**
- Show closed fist (✊) - Two-stage confirmation stop (see below)
- Walk away for 35 seconds - Automatic timeout (watchdog)
- Stay silent for 60 seconds - Voice Activity Detection timeout
- Robot confirms end with closing beep

**Conversation Protection**
- Voice Activity Detection prevents premature stops
- System stays active while you're speaking
- 60-second silence timer only counts when you're quiet
- Immune to camera detection flickers during conversation

**Performance**
- End-to-end latency: 700-1200ms (speech → response)
- Natural voice synthesis (configurable personalities)
- High transcription accuracy (99%+)
- No interruption of your speech

### Conversation History ✅ OPERATIONAL

**Automatic Persistence**
- Every conversation automatically saved to database
- Includes full transcript (your words + robot responses)
- Timestamped and linked to recognized person
- Searchable conversation history
- Privacy: stored locally only, not sent to cloud

**For technical details:** See `200_SPEECH_SYSTEM_REFERENCE.md`

**Technical Implementation:** See [`001_ARCHITECTURE_OVERVIEW.md`](001_ARCHITECTURE_OVERVIEW.md) Section 9.2 (Phase 2 Architecture - Speech/Conversation)

---

## 3. Learning and Tutoring

### What It Does
The robot functions as an interactive tutor, helping you learn robotics, ROS 2, and programming concepts through two specialized modes.

### Coding Tutor Mode ✅ OPERATIONAL

**Real-Time Code Narration**
- Robot watches your coding session in real-time
- Speaks explanations of code changes as AI agent works
- Uses Business Intelligence analogies (for BI professionals)
- Explains "what this means" in plain language

**Activation**
- Say: "Turn on learning mode"
- Robot confirms: "[chirp] Learning mode activated!"
- Robot now narrates all significant code changes

**What Gets Explained**
- File edits and their purpose
- ROS 2 concepts (topics, subscribers, nodes)
- System architecture decisions
- Why specific approaches were chosen

**Example Narration:**
> "A subscriber listens to a topic and executes a callback function whenever new data arrives. It's like a SQL trigger that fires when a row is inserted into a table."

**Deactivation**
- Say: "Learning off" or "Stop learning"

### General Tutor Mode ✅ OPERATIONAL

**Interactive Q&A Teaching**
- Robot becomes a patient, educational teacher
- Answers questions about any robotics/programming topic
- Checks your understanding ("Does that make sense?")
- Builds concepts progressively
- Encourages questions and exploration

**Activation**
- Say: "Be my tutor" or "Teach me about [topic]"
- Robot shifts to teaching personality

**Teaching Style**
- Uses BI-to-Robotics concept mappings
- Patient and thorough explanations
- Progressive concept building
- Interactive follow-up questions
- Positive encouragement

**Example Teaching:**
> "Great topic! A ROS 2 service is like a stored procedure in SQL. You call it, wait for execution, and get a result back. Unlike topics which are fire-and-forget, services are request-response. Does that make sense?"

**Deactivation**
- Say: "Tutor off" or "Normal mode"

### Learning Progress Tracking ✅ OPERATIONAL

**Automatic Tracking**
- System tracks which concepts you've encountered
- Records understanding level (1-5 scale)
- Logs learning sessions with duration and topics
- Stores BI analogies used for each concept
- Database persists across sessions

**Progress Visibility**
- Query learning history via SQL commands
- See topics needing review (understanding < 3)
- Review recent learning sessions
- Track improvement over time

**For technical details:** See `300_AI_TUTOR.md`

**Technical Implementation:** See [`001_ARCHITECTURE_OVERVIEW.md`](001_ARCHITECTURE_OVERVIEW.md) Section 9.2 (Speech System with tutor mode personality)

---

## 4. Remote Monitoring and Control

### What It Does
Monitor and control the robot from anywhere in the world through a web-based dashboard, accessible via secure VPN connection.

### Web Dashboard ✅ OPERATIONAL

**Access**
- URL: `http://100.x.x.x:8080` (via Tailscale VPN)
- Works from any device with web browser
- No installation required
- Secure encrypted connection

**Real-Time Monitoring**
- Person recognition status (RED/GREEN/BLUE)
- Live camera stream (MJPEG)
- System health metrics (CPU, GPU, temperature)
- Active services status
- Current audio volume
- Event log stream

**Service Management**
- Start/stop/restart any R2D2 service
- View service status and logs
- Enable/disable auto-start
- Individual service control

**Audio Control**
- Adjust volume with slider (0-100%)
- Quick presets (Quiet, Medium, Loud)
- Live parameter updates (no restart needed)
- Applies to all audio feedback

**Training Interface**
- Complete face recognition training via web
- Capture images remotely
- Train new person models
- Test recognition accuracy
- Manage trained persons
- Delete person profiles

**System Health**
- CPU usage percentage
- GPU usage percentage
- System temperature (°C)
- Disk space utilization
- On-demand metrics (saves resources)

**Camera Stream**
- Live MJPEG video stream
- Configurable quality and frame rate
- On-demand activation (not always running)
- Direct view of robot's camera perspective

**Dashboard Design**
- Dark futuristic Star Wars theme
- Single-page layout (no scrolling on 1920×1200)
- Color-coded status indicators
- Responsive real-time updates
- Professional UX

**System Diagnostics** ✅ OPERATIONAL (Added January 4, 2026)
- Comprehensive diagnostics page at `/diagnostics`
- Real-time monitoring of all 12 system services
- Live status indicators (confidence, LED, VAD, timers, watchdog)
- Button-activated topic monitoring (14 UX-relevant topics)
- Safe diagnostic tests (PulseAudio, Bluetooth, audio, logs)
- Read-only mode by default with service protection
- Zero interference with core robot functionality

### Service Mode Architecture ✅ OPERATIONAL

**Efficient Resource Usage**
- Minimal "Wake API" always running (port 8079)
- Full dashboard started on-demand only
- Automatic resource cleanup when not in use
- Check status and start UI from wake endpoint

**For technical details:** See `110_WEB_UI_REFERENCE.md`

**Technical Implementation:** See [`001_ARCHITECTURE_OVERVIEW.md`](001_ARCHITECTURE_OVERVIEW.md) Section 8 (Web Dashboard System)

---

## 5. Physical Controls

### What It Does
Direct physical interaction with the robot through buttons for power management.

### Power Management ✅ OPERATIONAL

**Shutdown Button**
- Physical momentary button on robot chassis
- Press to initiate graceful shutdown
- System saves state before powering down
- Safe shutdown prevents data corruption
- Connected to GPIO Pin 32

**Wake/Boot Button**
- Physical momentary button on robot chassis
- Press to wake from shutdown
- Press to boot after power loss
- Connected to J42 Automation Header
- Instant response to button press

**Status Indication**
- LED confirms system state
- Audio feedback on shutdown initiation
- Clean shutdown process takes ~30 seconds

**For technical details:** See `020_POWER_BUTTON_FINAL_DOCUMENTATION.md`

**Technical Implementation:** See [`001_ARCHITECTURE_OVERVIEW.md`](001_ARCHITECTURE_OVERVIEW.md) Section 7.1 (Power Button System)

---

## 6. Gesture Recognition

### What It Does
Control robot functions using hand gestures detected by the camera.

### Gesture Control ✅ OPERATIONAL

**Supported Gestures**
- ☝️ **Index Finger Up:** Start conversation (instant trigger)
- ✊ **Fist:** Stop conversation (two-stage confirmation)
  - Stage 1: Hold fist ~1.5s → Warning beep (chance to cancel)
  - Stage 2: Continue holding ~1.5s → Stop beep + session ends
  - Release fist at any time → Cancel stop, conversation continues
- More gestures trainable per person

**Two-Stage Fist Stop Protection**
- **Purpose:** Prevents accidental conversation termination
- **How it works:**
  1. Hold fist for ~1.5 seconds → Robot plays warning beep
  2. Release fist now → Conversation continues (no stop)
  3. Keep holding ~1.5 seconds more → Robot plays stop beep and ends session
- **Total time to stop:** ~3 seconds of deliberate hold
- **Benefit:** Completely eliminates false positives from brief fist gestures

**Gesture Training**
- Person-specific gesture models
- 15 seconds capture per gesture
- MediaPipe Hands detection
- SVM classification for accuracy
- Training integrated with face recognition

**Gating Logic**
- Gestures only work when you're recognized (RED status)
- Prevents false triggers from strangers
- Cooldown periods prevent accidental re-triggers
- 35-second watchdog auto-shutdown when you leave

**Performance**
- 100-200ms latency (detection → action)
- 70-90% accuracy with good training
- 15 Hz gesture detection rate
- Real-time response

**For technical details:** See `300_GESTURE_SYSTEM_OVERVIEW.md`, `303_GESTURE_TRAINING_GUIDE.md`

**Technical Implementation:** See [`001_ARCHITECTURE_OVERVIEW.md`](001_ARCHITECTURE_OVERVIEW.md) Sections 3.1 (gesture_intent_node), 6.2.5 (Gesture Intent Parameters), 6.4 (Gesture Recognition System)

---

## 7. Physical Expression and Movement

### What It Does
Robot expresses awareness and moves through the environment (planned functionality).

### Head Movement ⏳ PLANNED

**Pan (Horizontal Rotation)**
- Head rotates left and right
- Tracks person as they move
- Looks toward speaker during conversation
- Hardware: DeAgostini dome motor (installed, not integrated)

**Camera Orientation**
- Camera mounted in rotating head
- Follows head rotation automatically
- Improves recognition when tracking people
- Hardware ready (OAK-D camera installed)

### Autonomous Navigation ⏳ PLANNED

**Indoor Navigation**
- Build map of rooms and hallways
- Navigate autonomously to requested locations
- Avoid obstacles (furniture, people, pets)
- Safe speed control

**Person Following**
- Follow recognized person through rooms
- Maintain safe following distance
- Configurable enable/disable
- Stop when person stops

**Patrol Behavior**
- Autonomous room-to-room movement
- Configurable patrol routes
- Periodic environment monitoring
- Lightweight security function

**Navigation Commands (Voice)**
- "Come here" - Move to speaker
- "Follow me" - Enable following mode
- "Go to [room]" - Navigate to named location
- "Stop following" - Disable following

### Hardware Status
- ✅ Motors installed (2× wheel motors + 1× dome motor)
- ✅ Motor drivers assembled (Pololu G2 24v21)
- ✅ Encoders integrated
- ✅ Power system ready
- ⏳ Software integration pending
- ⏳ SLAM mapping pending
- ⏳ Path planning pending

**Technical Implementation:** See [`001_ARCHITECTURE_OVERVIEW.md`](001_ARCHITECTURE_OVERVIEW.md) Section 9.3 (Future Navigation Components - Planned)

---

## 8. Advanced Features

### What It Does
Extended capabilities for enhanced interaction and autonomy (planned functionality).

### LED Text Display Panels ⏳ PLANNED

**Capabilities**
- Two LED panels for text output
- Display short messages or status
- Real-time information display
- Visual communication without audio

**Use Cases**
- Display current activity
- Show person name when recognized
- System status messages
- Conversation snippets
- Error notifications

**Hardware Status**
- ✅ LED panels installed
- ⏳ Driver software pending
- ⏳ Content specifications pending

### Advanced Status Indicators 🔨 PARTIAL

**Current Implementation**
- ✅ White LED panel (ON/OFF for recognition)
- Simple binary state indication

**Planned Enhancement**
- ⏳ RGB LED strip (WS2812B)
- Advanced color patterns
- Animated sequences
- Personality expressions
- Multiple simultaneous states
- Hardware reserved (GPIO Pin 12)

**Planned Patterns**
- Status animations (breathing, pulsing)
- Attention-getting patterns
- Error indication modes
- Personality expressions
- Ambient lighting

### Enhanced Memory System 🔨 PARTIAL

**Current Implementation**
- ✅ Conversation history stored
- ✅ Learning progress tracked
- ✅ Person profiles maintained

**Planned Enhancement**
- ⏳ Long-term context awareness
- ⏳ Preference learning
- ⏳ Behavioral adaptation
- ⏳ Proactive assistance
- ⏳ Multi-session memory

### Room Understanding ⏳ PLANNED

**Capabilities**
- Map building (SLAM)
- Room identification
- Location awareness
- Spatial memory
- Named location navigation

**Integration**
- Voice commands: "Go to living room"
- Autonomous exploration
- Map updates over time
- Obstacle detection and mapping

**Technical Implementation:** See [`001_ARCHITECTURE_OVERVIEW.md`](001_ARCHITECTURE_OVERVIEW.md) Section 9 (Integration Points for Future Features)

---

## Status Legend

Throughout this document, the following status markers indicate feature availability:

| Marker | Status | Description |
|--------|--------|-------------|
| ✅ | **OPERATIONAL** | Fully functional, tested, and ready to use |
| 🔨 | **PARTIAL** | Basic functionality works, enhancements in progress |
| ⏳ | **PLANNED** | Hardware ready or design complete, software pending |

---

## Getting Started

### Prerequisites
- R2D2 robot powered on and connected to network
- Face training completed for at least one person
- Tailscale VPN installed on your device (for remote access)

### First Time Setup
1. Train the robot to recognize you (web dashboard or command line)
2. Test recognition by standing in front of camera
3. Verify LED turns on when recognized (RED status)
4. Practice gestures: index finger up, fist
5. Start first conversation with index finger gesture

### Daily Usage
1. Approach robot - LED turns on when recognized
2. Index finger up to start conversation
3. Speak naturally - robot responds
4. Fist to end conversation, or walk away
5. Access web dashboard anytime for monitoring

### Remote Access
1. Connect to Tailscale VPN
2. Open browser to `http://100.x.x.x:8080`
3. Monitor status, adjust settings, train new people
4. View live camera stream
5. Control services as needed

---

## Technical Documentation References

For implementation details and troubleshooting:

**System Architecture:**
- [`001_ARCHITECTURE_OVERVIEW.md`](001_ARCHITECTURE_OVERVIEW.md) - **Complete system architecture (technical implementation of this document)**
- `002_HARDWARE_REFERENCE.md` - Hardware specifications

**Person Recognition:**
- `100_PERCEPTION_STATUS_REFERENCE.md` - Recognition system reference
- `101_PERCEPTION_STATUS_INSTALLATION.md` - Installation guide
- `102_PERCEPTION_STATUS_QUICK_START.md` - Quick start
- `103_PERCEPTION_STATUS_TROUBLESHOOTING.md` - Debug procedures

**Speech System:**
- `200_SPEECH_SYSTEM_REFERENCE.md` - Speech system reference
- `201_SPEECH_SYSTEM_INSTALLATION.md` - Installation guide
- `202_SPEECH_SYSTEM_QUICK_START.md` - Quick start
- `203_SPEECH_SYSTEM_TROUBLESHOOTING.md` - Debug procedures
- `204_SPEECH_SYSTEM_VOICE_CONFIGURATION.md` - Voice customization

**AI Tutor:**
- `300_AI_TUTOR.md` - Complete tutor system reference

**Web Dashboard:**
- `110_WEB_UI_REFERENCE.md` - Web UI reference
- `111_WEB_UI_INSTALLATION.md` - Installation guide
- `112_WEB_UI_QUICK_START.md` - Quick start

**Person Management:**
- `250_PERSON_MANAGEMENT_SYSTEM_REFERENCE.md` - Person registry system

**Gesture Recognition:**
- `300_GESTURE_SYSTEM_OVERVIEW.md` - Gesture system overview
- `303_GESTURE_TRAINING_GUIDE.md` - Training guide

---

**Document Purpose:** This document focuses on user experience and capabilities. For technical implementation details, system administration, or troubleshooting, please refer to the specific technical documentation listed above.

**Maintained By:** R2D2 Project Team  
**Questions or Issues:** See `README.md` for contribution guidelines

