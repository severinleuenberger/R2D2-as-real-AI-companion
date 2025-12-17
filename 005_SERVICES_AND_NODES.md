# R2D2 System - Complete Service and Node Inventory

**Date:** December 17, 2025  
**Purpose:** Comprehensive inventory of all services, nodes, and components in the R2D2 system  
**Status:** Complete inventory for analysis and optimization

---

## Executive Summary

This document provides a complete inventory of all ROS 2 nodes, systemd services, web services, and system daemons in the R2D2 system. It serves as a reference for understanding system architecture, resource allocation, and optimization opportunities.

**Total Components:**
- **9 ROS 2 Nodes** (8 production + 1 demo)
- **7 Systemd Services** (5 R2D2 + 2 supporting)
- **3 Web Services** (on-demand dashboard)
- **1 System Service** (Tailscale VPN)

---

## 1. ROS 2 Nodes Inventory

### Node 1: camera_node

**Name:** `camera_node`  
**Type:** ROS 2 Node (Python)  
**Package:** `r2d2_camera`  
**Purpose:** OAK-D Lite camera driver - captures RGB frames and publishes to ROS 2

**Dependencies:**
- Hardware: OAK-D Lite camera (USB 3.0)
- Software: DepthAI SDK 2.31.0.0, depthai Python bindings
- Environment: `~/depthai_env` virtual environment

**Storage Location:**
- Source: `~/dev/r2d2/ros2_ws/src/r2d2_camera/r2d2_camera/oak_camera_node.py`
- Launch: `~/dev/r2d2/ros2_ws/src/r2d2_camera/launch/camera.launch.py`

**Active Status:** ✅ Always running (auto-start via systemd)  
**Auto-Start:** Yes (via `r2d2-camera-perception.service`)

**Resource Usage:**
- CPU: 2-3% (single core)
- RAM: ~50 MB
- GPU: 0%
- Frequency: 30 Hz (continuous)

**Topics Published:**
- `/oak/rgb/image_raw` (sensor_msgs/Image, 30 Hz) - Raw camera frames

**Documentation References:**
- `001_ARCHITECTURE_OVERVIEW.md` - Section 1.1, 2.1, 3.1
- `102_CAMERA_SETUP_DOCUMENTATION.md` - Complete camera setup
- `100_PERSON_RECOGNITION_AND_STATUS.md` - Integration with perception

---

### Node 2: image_listener

**Name:** `image_listener`  
**Type:** ROS 2 Node (Python)  
**Package:** `r2d2_perception`  
**Purpose:** Image processing pipeline - face detection, recognition, brightness computation

**Dependencies:**
- Topics: `/oak/rgb/image_raw` (from camera_node)
- Software: OpenCV, OpenCV contrib (LBPH), NumPy, cv_bridge
- Model: `~/dev/r2d2/data/face_recognition/models/severin_lbph.xml` (if recognition enabled)
- Environment: `OPENBLAS_CORETYPE=ARMV8` (critical on ARM64)

**Storage Location:**
- Source: `~/dev/r2d2/ros2_ws/src/r2d2_perception/r2d2_perception/image_listener.py` (354 lines)
- Launch: `~/dev/r2d2/ros2_ws/src/r2d2_perception/launch/perception.launch.py`

**Active Status:** ✅ Always running (auto-start via systemd)  
**Auto-Start:** Yes (via `r2d2-camera-perception.service`)

**Resource Usage:**
- CPU: 8-15% (single core) - without recognition: 8-10%, with recognition: 10-15%
- RAM: ~200 MB
- GPU: 0% (not accelerated)
- Frequency: 13 Hz (face detection), 6.5 Hz (recognition when enabled)

**Topics Subscribed:**
- `/oak/rgb/image_raw` (sensor_msgs/Image, 30 Hz)

**Topics Published:**
- `/r2d2/perception/brightness` (std_msgs/Float32, 13 Hz) - Mean brightness
- `/r2d2/perception/face_count` (std_msgs/Int32, 13 Hz) - Number of faces detected
- `/r2d2/perception/person_id` (std_msgs/String, 6.5 Hz*) - Person name ("target_person", "unknown", "no_person")
- `/r2d2/perception/face_confidence` (std_msgs/Float32, 6.5 Hz*) - Recognition confidence (lower is better)
- `/r2d2/perception/is_target_person` (std_msgs/Bool, 6.5 Hz*) - Boolean convenience topic

*Only published if `enable_face_recognition:=true`

**Configuration Parameters:**
- `enable_face_recognition` (bool, default: false)
- `recognition_frame_skip` (int, default: 2) - Process every Nth frame
- `recognition_confidence_threshold` (float, default: 70.0)
- `face_recognition_model_path` (string)

**Documentation References:**
- `001_ARCHITECTURE_OVERVIEW.md` - Section 2.1, 2.2, 3.1, 5.1
- `100_PERSON_RECOGNITION_AND_STATUS.md` - Complete setup guide
- `FACE_RECOGNITION_ANALYSIS_SUMMARY.md` - Performance analysis

---

### Node 3: heartbeat_node

**Name:** `heartbeat_node`  
**Type:** ROS 2 Node (Python)  
**Package:** `r2d2_hello`  
**Purpose:** System health monitoring - publishes CPU, GPU, temperature metrics

**Dependencies:**
- Software: psutil (system metrics), rclpy
- Hardware: Jetson AGX Orin system sensors

**Storage Location:**
- Source: `~/dev/r2d2/ros2_ws/src/r2d2_hello/r2d2_hello/heartbeat_node.py`
- Launch: `~/dev/r2d2/ros2_ws/src/r2d2_hello/launch/heartbeat.launch.py`

**Active Status:** ✅ Always running (auto-start via systemd)  
**Auto-Start:** Yes (via `r2d2-heartbeat.service`)

**Resource Usage:**
- CPU: <0.5% (negligible)
- RAM: ~10 MB
- GPU: 0%
- Frequency: 1 Hz (continuous)

**Topics Published:**
- `/r2d2/heartbeat` (std_msgs/String, JSON, 1 Hz) - System health with metrics

**Message Format (JSON):**
```json
{
  "status": "alive",
  "timestamp": 1734451200,
  "cpu_percent": 23.5,
  "gpu_percent": 0.0,
  "temperature_celsius": 45.2,
  "memory_percent": 15.3
}
```

**Documentation References:**
- `001_ARCHITECTURE_OVERVIEW.md` - Section 2.3, 3.1
- `111_WEB_DASHBOARD_DOCUMENTATION.md` - Web dashboard integration

---

### Node 4: camera_stream_node

**Name:** `camera_stream_node`  
**Type:** ROS 2 Node (Python)  
**Package:** `r2d2_camera`  
**Purpose:** MJPEG HTTP stream server - provides on-demand video stream for web dashboard

**Dependencies:**
- Topics: `/oak/rgb/image_raw` (from camera_node)
- Software: OpenCV, HTTP server (built-in)
- Hardware: OAK-D camera (EXCLUSIVE ACCESS - cannot run with camera_node simultaneously)

**Storage Location:**
- Source: `~/dev/r2d2/ros2_ws/src/r2d2_camera/r2d2_camera/camera_stream_node.py`
- Launch: `~/dev/r2d2/ros2_ws/src/r2d2_camera/launch/camera_stream.launch.py`

**Active Status:** ❌ On-demand only (NOT auto-start)  
**Auto-Start:** No (started manually or via web dashboard)

**Resource Usage:**
- CPU: 2-5% (single core)
- RAM: ~50 MB
- GPU: 0%
- Frequency: 15 FPS (MJPEG stream)
- Network: HTTP server on port 8081

**Topics Subscribed:**
- `/oak/rgb/image_raw` (sensor_msgs/Image, 30 Hz)

**HTTP Endpoints:**
- `http://100.95.133.26:8081/stream` - MJPEG video stream

**Special Notes:**
- **DEVICE EXCLUSIVITY:** Cannot run simultaneously with `camera_node` (both access OAK-D camera)
- Web dashboard enforces mutual exclusivity with 3-second delay when switching
- Use for video preview only; disable for face recognition

**Documentation References:**
- `001_ARCHITECTURE_OVERVIEW.md` - Section 1.3, 3.1
- `111_WEB_DASHBOARD_DOCUMENTATION.md` - Camera stream integration

---

### Node 5: audio_notification_node

**Name:** `audio_notification_node`  
**Type:** ROS 2 Node (Python)  
**Package:** `r2d2_audio`  
**Purpose:** Person recognition state machine - tracks recognition status and plays audio alerts

**Dependencies:**
- Topics: `/r2d2/perception/person_id` (from image_listener)
- Software: ffplay (audio playback), rclpy
- Audio Files: `Voicy_R2-D2 - 2.mp3` (recognition), `Voicy_R2-D2 - 5.mp3` (loss)
- Hardware: PAM8403 amplifier + speaker (ALSA device `hw:1,0`)

**Storage Location:**
- Source: `~/dev/r2d2/ros2_ws/src/r2d2_audio/r2d2_audio/audio_notification_node.py`
- Launch: `~/dev/r2d2/ros2_ws/src/r2d2_audio/launch/audio_notification.launch.py`
- Audio Assets: `~/dev/r2d2/ros2_ws/src/r2d2_audio/r2d2_audio/assets/audio/`

**Active Status:** ✅ Always running (auto-start via systemd)  
**Auto-Start:** Yes (via `r2d2-audio-notification.service`)

**Resource Usage:**
- CPU: 2-4% (single core)
- RAM: ~50 MB
- GPU: 0%
- Frequency: 10 Hz (status publishing)

**Topics Subscribed:**
- `/r2d2/perception/person_id` (std_msgs/String, 6.5 Hz)

**Topics Published:**
- `/r2d2/audio/person_status` (std_msgs/String, JSON, 10 Hz) - Status (RED/BLUE/GREEN)
- `/r2d2/audio/notification_event` (std_msgs/String, Event) - Recognition/loss events
- `/r2d2/audio/status` (std_msgs/String, Event) - Audio system status

**State Machine:**
- 🔴 **RED**: Target person recognized (plays "Hello!")
- 🔵 **BLUE**: No person detected (plays "Oh, I lost you!" after 20s)
- 🟢 **GREEN**: Unknown person detected (silent)

**Configuration Parameters:**
- `target_person` (string, default: "target_person")
- `audio_volume` (float, default: 0.05) - Range 0.0-1.0
- `jitter_tolerance_seconds` (float, default: 5.0)
- `loss_confirmation_seconds` (float, default: 15.0)
- `cooldown_seconds` (float, default: 2.0)
- `recognition_cooldown_after_loss_seconds` (float, default: 5.0)

**Documentation References:**
- `001_ARCHITECTURE_OVERVIEW.md` - Section 4, 6.2
- `100_PERSON_RECOGNITION_AND_STATUS.md` - Complete setup and tuning guide
- `AUDIO_STATUS_SYSTEM_ARCHITECTURE.md` - State machine details

---

### Node 6: status_led_node

**Name:** `status_led_node`  
**Type:** ROS 2 Node (Python)  
**Package:** `r2d2_audio`  
**Purpose:** GPIO LED control - visual feedback for recognition status

**Dependencies:**
- Topics: `/r2d2/audio/person_status` (from audio_notification_node)
- Hardware: RGB LED on GPIO pins 17 (RED), 27 (GREEN), 22 (BLUE)
- Software: RPi.GPIO or Jetson.GPIO, rclpy

**Storage Location:**
- Source: `~/dev/r2d2/ros2_ws/src/r2d2_audio/r2d2_audio/status_led_node.py`
- Launch: Included in `audio_notification.launch.py`

**Active Status:** ✅ Always running (auto-start via systemd)  
**Auto-Start:** Yes (via `r2d2-audio-notification.service`)

**Resource Usage:**
- CPU: <0.1% (negligible)
- RAM: ~20 MB
- GPU: 0%
- Frequency: 10 Hz (LED updates)

**Topics Subscribed:**
- `/r2d2/audio/person_status` (std_msgs/String, JSON, 10 Hz)

**GPIO Pins:**
- Pin 17: RED LED (target person recognized)
- Pin 27: GREEN LED (unknown person)
- Pin 22: BLUE LED (no person / idle)

**Special Notes:**
- Falls back to simulation mode if GPIO not available
- Synchronized with audio_notification_node status

**Documentation References:**
- `001_ARCHITECTURE_OVERVIEW.md` - Section 4.2
- `100_PERSON_RECOGNITION_AND_STATUS.md` - LED setup

---

### Node 7: database_logger_node

**Name:** `database_logger_node`  
**Type:** ROS 2 Node (Python)  
**Package:** `r2d2_audio`  
**Purpose:** Event logging - tracks state transitions and recognition events

**Dependencies:**
- Topics: `/r2d2/audio/person_status` (from audio_notification_node)
- Software: rclpy
- Future: SQLite database integration

**Storage Location:**
- Source: `~/dev/r2d2/ros2_ws/src/r2d2_audio/r2d2_audio/database_logger_node.py`
- Launch: Included in `audio_notification.launch.py`

**Active Status:** ✅ Always running (auto-start via systemd)  
**Auto-Start:** Yes (via `r2d2-audio-notification.service`)

**Resource Usage:**
- CPU: <0.1% (negligible)
- RAM: ~30 MB
- GPU: 0%
- Frequency: 10 Hz (event logging)

**Topics Subscribed:**
- `/r2d2/audio/person_status` (std_msgs/String, JSON, 10 Hz)

**Current Implementation:**
- Logs state transitions to console
- Structure ready for SQLite integration

**Future Enhancements:**
- SQLite database for persistent storage
- Analytics and conversation history
- Integration with Phase 2 speech system

**Documentation References:**
- `001_ARCHITECTURE_OVERVIEW.md` - Section 4.2
- `100_PERSON_RECOGNITION_AND_STATUS.md` - Logger setup

---

### Node 8: audio_beep_node

**Name:** `audio_beep_node`  
**Type:** ROS 2 Node (Python)  
**Package:** `r2d2_audio`  
**Purpose:** Demo node - simple beep generation for testing audio hardware

**Dependencies:**
- Software: PyAudio or similar, rclpy
- Hardware: PAM8403 amplifier + speaker

**Storage Location:**
- Source: `~/dev/r2d2/ros2_ws/src/r2d2_audio/r2d2_audio/audio_beep_node.py`
- Launch: Manual execution only

**Active Status:** ❌ Demo only (NOT auto-start)  
**Auto-Start:** No (manual testing only)

**Resource Usage:**
- CPU: <0.1% (negligible)
- RAM: ~10 MB
- GPU: 0%
- Frequency: Event-driven

**Topics Published:**
- `/r2d2/audio/beep_count` (std_msgs/UInt32, Event)
- `/r2d2/audio/last_frequency` (std_msgs/Float32, Event)

**Purpose:** Testing and demonstration only - NOT used in production

**Documentation References:**
- `001_ARCHITECTURE_OVERVIEW.md` - Section 1.3, 3.1

---

### Node 9: speech_node

**Name:** `speech_node`  
**Type:** ROS 2 Lifecycle Node (Python)  
**Package:** `r2d2_speech`  
**Purpose:** Phase 2 speech system - OpenAI Realtime API integration for STT/TTS/LLM

**Dependencies:**
- Software: OpenAI Realtime API, asyncio, rclpy, lifecycle_msgs
- Hardware: HyperX QuadCast S USB microphone, PAM8403 speaker
- Environment: `OPENAI_API_KEY` in `~/.r2d2/.env`
- Database: SQLite (`~/dev/r2d2/r2d2_speech/chat_history.db`)

**Storage Location:**
- Source: `~/dev/r2d2/ros2_ws/src/r2d2_speech/r2d2_speech/speech_node.py`
- Launch: `~/dev/r2d2/ros2_ws/src/r2d2_speech/launch/speech_node.launch.py`
- Config: `~/dev/r2d2/ros2_ws/src/r2d2_speech/config/speech_params.yaml`

**Active Status:** ❌ Optional (Phase 2, NOT auto-start)  
**Auto-Start:** No (manual launch when needed)

**Resource Usage:**
- CPU: 10-15% (single core) - audio processing and API communication
- RAM: ~150 MB
- GPU: 0%
- Network: WebSocket to OpenAI Realtime API

**Topics Published:**
- `/r2d2/speech/user_transcript` (std_msgs/String) - User speech transcribed
- `/r2d2/speech/assistant_transcript` (std_msgs/String) - Assistant responses
- `/r2d2/speech/session_status` (std_msgs/String, JSON) - Session state

**Topics Subscribed:**
- `/r2d2/speech/commands` (std_msgs/String) - External commands
- `/r2d2/speech/assistant_prompt` (std_msgs/String) - Inject prompts

**Services Provided:**
- `/r2d2/speech/start_session` (std_srvs/Trigger) - Start speech session
- `/r2d2/speech/stop_session` (std_srvs/Trigger) - Stop speech session

**Lifecycle States:**
- Unconfigured → Configured → Active → (Inactive) → Cleanup

**Configuration Parameters:**
- `openai_api_key` (string)
- `model` (string, default: "gpt-4o-realtime-preview-2024-12-17")
- `voice` (string, default: "echo")
- `input_audio_device` (int, default: -1)
- `output_audio_device` (int, default: -1)
- `db_path` (string, default: "~/dev/r2d2/r2d2_speech/chat_history.db")

**Special Notes:**
- Phase 2 system (not yet in production)
- Uses asyncio event loop in background thread
- Integrates with existing ROS 2 perception and audio systems
- Persistent chat history in SQLite database

**Documentation References:**
- `200_SPEECH_SYSTEM_REFERENCE.md` - System architecture and reference
- `201_SPEECH_SYSTEM_INSTALLATION.md` - Installation instructions
- `203_SPEECH_SYSTEM_QUICK_START.md` - Quick reference
- `ROS2_SPEECH_TESTING.md` - Integration testing

---

## 2. Systemd Services Inventory

### Service 1: r2d2-camera-perception.service

**Name:** `r2d2-camera-perception.service`  
**Type:** Systemd service (oneshot + forking)  
**Purpose:** Launch camera + perception pipeline with face recognition

**Dependencies:**
- Nodes: camera_node, image_listener
- Environment: `~/depthai_env` virtual environment
- Hardware: OAK-D Lite camera

**Storage Location:**
- Service file: `/etc/systemd/system/r2d2-camera-perception.service`
- Startup script: `/home/severin/dev/r2d2/start_camera_perception.sh`
- Launch file: `~/dev/r2d2/ros2_ws/src/r2d2_bringup/launch/r2d2_camera_perception.launch.py`

**Active Status:** ✅ Enabled and running (auto-start on boot)  
**When Active:** Always (from boot)

**Resource Usage:**
- Total CPU: 10-18% (combined camera + perception)
- Total RAM: ~250 MB
- Boot time: 5-7 seconds to ready

**Nodes Launched:**
- camera_node (2-3% CPU, ~50 MB RAM)
- image_listener (8-15% CPU, ~200 MB RAM)

**Parameters:**
- `enable_face_recognition:=true` (configured in service)
- `target_person_name:=severin` (configured in service)

**Documentation References:**
- `001_ARCHITECTURE_OVERVIEW.md` - Section 3.2, 6.1
- `100_PERSON_RECOGNITION_AND_STATUS.md` - Service setup

---

### Service 2: r2d2-audio-notification.service

**Name:** `r2d2-audio-notification.service`  
**Type:** Systemd service (oneshot + forking)  
**Purpose:** Launch audio notification system (state machine + LED + logger)

**Dependencies:**
- Nodes: audio_notification_node, status_led_node, database_logger_node
- Topics: `/r2d2/perception/person_id` (from image_listener)
- Hardware: PAM8403 amplifier, speaker, RGB LED

**Storage Location:**
- Service file: `/etc/systemd/system/r2d2-audio-notification.service`
- Startup script: `/home/severin/dev/r2d2/start_audio_service.sh`
- Launch file: `~/dev/r2d2/ros2_ws/src/r2d2_audio/launch/audio_notification.launch.py`

**Active Status:** ✅ Enabled and running (auto-start on boot)  
**When Active:** Always (from boot)

**Resource Usage:**
- Total CPU: 2-4%
- Total RAM: ~100 MB

**Nodes Launched:**
- audio_notification_node (2-4% CPU, ~50 MB RAM)
- status_led_node (<0.1% CPU, ~20 MB RAM)
- database_logger_node (<0.1% CPU, ~30 MB RAM)

**Parameters:**
- `target_person:=severin`
- `audio_volume:=0.05`
- `jitter_tolerance_seconds:=5.0`
- `loss_confirmation_seconds:=15.0`

**Auto-Restart:** Yes (on failure, max 3 restarts, 5s delay)

**Documentation References:**
- `001_ARCHITECTURE_OVERVIEW.md` - Section 4, 7.1
- `100_PERSON_RECOGNITION_AND_STATUS.md` - Audio service setup

---

### Service 3: r2d2-heartbeat.service

**Name:** `r2d2-heartbeat.service`  
**Type:** Systemd service (oneshot + forking)  
**Purpose:** System health monitoring - publishes CPU/GPU/temperature metrics

**Dependencies:**
- Node: heartbeat_node
- Software: psutil

**Storage Location:**
- Service file: `/etc/systemd/system/r2d2-heartbeat.service`
- Launch file: `~/dev/r2d2/ros2_ws/src/r2d2_hello/launch/heartbeat.launch.py`

**Active Status:** ✅ Enabled and running (auto-start on boot)  
**When Active:** Always (from boot)

**Resource Usage:**
- CPU: <0.5%
- RAM: ~10 MB

**Nodes Launched:**
- heartbeat_node (<0.5% CPU, ~10 MB RAM)

**Documentation References:**
- `001_ARCHITECTURE_OVERVIEW.md` - Section 2.3, 3.1
- `111_WEB_DASHBOARD_DOCUMENTATION.md` - Heartbeat integration

---

### Service 4: r2d2-camera-stream.service

**Name:** `r2d2-camera-stream.service`  
**Type:** Systemd service (oneshot + forking)  
**Purpose:** On-demand MJPEG video stream for web dashboard

**Dependencies:**
- Node: camera_stream_node
- Hardware: OAK-D camera (EXCLUSIVE ACCESS)

**Storage Location:**
- Service file: `/etc/systemd/system/r2d2-camera-stream.service`
- Launch file: `~/dev/r2d2/ros2_ws/src/r2d2_camera/launch/camera_stream.launch.py`

**Active Status:** ❌ Disabled (NOT auto-start, on-demand only)  
**When Active:** Manual start or via web dashboard

**Resource Usage:**
- CPU: 2-5%
- RAM: ~50 MB
- Network: HTTP server on port 8081

**Nodes Launched:**
- camera_stream_node (2-5% CPU, ~50 MB RAM)

**Special Notes:**
- **MUTUAL EXCLUSIVITY:** Cannot run with camera-perception service (device conflict)
- Web dashboard enforces 3-second delay when switching services
- Use for video preview only

**Documentation References:**
- `001_ARCHITECTURE_OVERVIEW.md` - Section 1.3, 3.1
- `111_WEB_DASHBOARD_DOCUMENTATION.md` - Camera stream integration

---

### Service 5: r2d2-powerbutton.service

**Name:** `r2d2-powerbutton.service`  
**Type:** Systemd service (simple)  
**Purpose:** Power button control - graceful shutdown via GPIO button

**Dependencies:**
- Hardware: GPIO button on Pin 32 (40-pin header)
- Software: RPi.GPIO or Jetson.GPIO

**Storage Location:**
- Service file: `/etc/systemd/system/r2d2-powerbutton.service`
- Script: `/home/severin/dev/r2d2/r2d2_power_button_simple.py`

**Active Status:** ⚠️ Optional (can be enabled/disabled)  
**When Active:** If enabled (boot-time optional)

**Resource Usage:**
- CPU: <0.1%
- RAM: ~10 MB

**Function:**
- Monitors GPIO Pin 32 for button press
- Triggers `shutdown -h now` on press
- Provides graceful shutdown mechanism

**Documentation References:**
- `001_ARCHITECTURE_OVERVIEW.md` - Section 7.1
- `020_POWER_BUTTON_FINAL_DOCUMENTATION.md` - Complete power button guide

---

### Service 6: r2d2-rosbridge.service

**Name:** `r2d2-rosbridge.service`  
**Type:** Systemd service (oneshot + forking)  
**Purpose:** rosbridge WebSocket server for web dashboard integration

**Dependencies:**
- Software: ros-humble-rosbridge-suite
- ROS 2: Running ROS 2 system with topics

**Storage Location:**
- Service file: `/etc/systemd/system/r2d2-rosbridge.service`
- Launch: Uses rosbridge_suite package

**Active Status:** ❌ Disabled (NOT auto-start, on-demand for web dashboard)  
**When Active:** When web dashboard is in use

**Resource Usage:**
- CPU: 2-3%
- RAM: ~50 MB
- Network: WebSocket server on port 9090

**Purpose:**
- Exposes ROS 2 topics via WebSocket for web dashboard
- Provides real-time topic streaming to browser
- Required for dashboard real-time updates

**Topics Exposed:**
- `/r2d2/perception/person_id`
- `/r2d2/audio/person_status`
- `/r2d2/perception/face_count`
- `/r2d2/heartbeat`

**Documentation References:**
- `111_WEB_DASHBOARD_DOCUMENTATION.md` - rosbridge setup
- `110_WEB_UI_ARCHITECTURE_AND_INTEGRATION.md` - Integration details

---

### Service 7: r2d2-web-dashboard.service

**Name:** `r2d2-web-dashboard.service`  
**Type:** Systemd service (oneshot + forking)  
**Purpose:** FastAPI web server for remote monitoring and control

**Dependencies:**
- Software: FastAPI, uvicorn, Python virtual environment
- Services: rosbridge_server (for real-time updates)
- Network: Tailscale VPN for remote access

**Storage Location:**
- Service file: `/etc/systemd/system/r2d2-web-dashboard.service`
- Application: `~/dev/r2d2/web_dashboard/app/main.py`
- Virtual env: `~/dev/r2d2/web_dashboard/web_dashboard_env/`

**Active Status:** ❌ Disabled (NOT auto-start, on-demand only)  
**When Active:** When remote monitoring/control needed

**Resource Usage:**
- CPU: 3-5%
- RAM: ~100 MB
- Network: HTTP server on port 8080

**Features:**
- REST API for service control (start/stop/restart)
- Volume control
- Face recognition training interface
- Real-time status monitoring
- System metrics display

**API Endpoints:**
- `GET /api/services/status` - Service status
- `POST /api/services/{service}/start` - Start service
- `POST /api/services/{service}/stop` - Stop service
- `GET /api/audio/volume` - Get volume
- `POST /api/audio/volume` - Set volume
- `POST /api/training/*` - Training operations

**Access:**
- URL: `http://100.95.133.26:8080` (via Tailscale VPN)
- VPN: Tailscale (must be connected)

**Documentation References:**
- `111_WEB_DASHBOARD_DOCUMENTATION.md` - Complete documentation
- `110_WEB_UI_ARCHITECTURE_AND_INTEGRATION.md` - Architecture
- `012_VPN_SETUP_AND_REMOTE_ACCESS.md` - VPN setup

---

## 3. Web Services Inventory

### Web Service 1: FastAPI Web Server

**Name:** FastAPI Web Server  
**Type:** Python web application (FastAPI)  
**Purpose:** REST API + dashboard UI for remote monitoring and control

**Dependencies:**
- Service: r2d2-web-dashboard.service (systemd)
- Python: FastAPI, uvicorn, rclpy
- Network: Tailscale VPN

**Storage Location:**
- Application: `~/dev/r2d2/web_dashboard/app/main.py`
- Frontend: `~/dev/r2d2/web_dashboard/app/templates/` and `~/dev/r2d2/web_dashboard/app/static/`

**Active Status:** ❌ On-demand only  
**When Active:** When web dashboard in use

**Resource Usage:**
- CPU: 3-5%
- RAM: ~100 MB
- Port: 8080 (HTTP)

**Features:**
- Service management (systemd control)
- Volume control
- Training interface
- Real-time status display
- System metrics

**Access:** `http://100.95.133.26:8080`

**Documentation References:**
- `111_WEB_DASHBOARD_DOCUMENTATION.md`
- `110_WEB_UI_QUICK_START.md`

---

### Web Service 2: rosbridge_server

**Name:** rosbridge WebSocket Server  
**Type:** ROS 2 package (rosbridge_suite)  
**Purpose:** WebSocket bridge to ROS 2 topics

**Dependencies:**
- Service: r2d2-rosbridge.service (systemd)
- Package: ros-humble-rosbridge-suite
- ROS 2: Active ROS 2 system

**Storage Location:**
- System package: `/opt/ros/humble/share/rosbridge_server/`

**Active Status:** ❌ On-demand only  
**When Active:** When web dashboard in use

**Resource Usage:**
- CPU: 2-3%
- RAM: ~50 MB
- Port: 9090 (WebSocket)

**Purpose:**
- Exposes ROS 2 topics via WebSocket
- Enables browser-based ROS 2 integration
- Required for dashboard real-time updates

**Documentation References:**
- `111_WEB_DASHBOARD_DOCUMENTATION.md`
- `110_WEB_UI_ARCHITECTURE_AND_INTEGRATION.md`

---

### Web Service 3: Camera Stream Service

**Name:** Camera MJPEG Stream  
**Type:** ROS 2 node with HTTP server  
**Purpose:** On-demand MJPEG video stream

**Dependencies:**
- Service: r2d2-camera-stream.service (systemd)
- Node: camera_stream_node
- Hardware: OAK-D camera (exclusive)

**Storage Location:**
- Node: `~/dev/r2d2/ros2_ws/src/r2d2_camera/r2d2_camera/camera_stream_node.py`

**Active Status:** ❌ On-demand only  
**When Active:** When video preview needed

**Resource Usage:**
- CPU: 2-5%
- RAM: ~50 MB
- Port: 8081 (HTTP)

**Access:** `http://100.95.133.26:8081/stream`

**Special Notes:**
- **EXCLUSIVE ACCESS:** Cannot run with camera-perception service
- Managed via web dashboard

**Documentation References:**
- `111_WEB_DASHBOARD_DOCUMENTATION.md`

---

## 4. System Services Inventory

### System Service 1: tailscaled

**Name:** `tailscaled`  
**Type:** System daemon  
**Purpose:** Tailscale VPN - secure remote access to Jetson

**Dependencies:**
- Network: Internet connection
- Software: Tailscale client

**Storage Location:**
- System service: `/etc/systemd/system/tailscaled.service`
- Config: `/var/lib/tailscale/`

**Active Status:** ✅ Enabled and running (auto-start on boot)  
**When Active:** Always (from boot)

**Resource Usage:**
- CPU: <1%
- RAM: ~30 MB
- Network: Minimal (keepalive traffic)

**Purpose:**
- Provides secure VPN access to Jetson from anywhere
- Assigns Jetson IP: `100.95.133.26`
- Zero-configuration mesh VPN
- Enables remote SSH and web dashboard access

**Access:**
- SSH: `ssh severin@100.95.133.26`
- Web Dashboard: `http://100.95.133.26:8080`

**Documentation References:**
- `012_VPN_SETUP_AND_REMOTE_ACCESS.md` - Complete VPN guide
- `vpn_config/TAILSCALE_SUCCESS.md` - Setup verification

---

## 5. Summary Tables

### 5.1 Resource Usage Summary

| Component | CPU | RAM | Auto-Start | Always Running? |
|-----------|-----|-----|-----------|----------------|
| **Core Nodes (Always Running)** | | | | |
| camera_node | 2-3% | 50 MB | ✅ Yes | ✅ Yes |
| image_listener | 8-15% | 200 MB | ✅ Yes | ✅ Yes |
| audio_notification_node | 2-4% | 50 MB | ✅ Yes | ✅ Yes |
| status_led_node | <0.1% | 20 MB | ✅ Yes | ✅ Yes |
| database_logger_node | <0.1% | 30 MB | ✅ Yes | ✅ Yes |
| heartbeat_node | <0.5% | 10 MB | ✅ Yes | ✅ Yes |
| **Core Subtotal** | **15-25%** | **~360 MB** | | |
| | | | | |
| **On-Demand Nodes** | | | | |
| camera_stream_node | 2-5% | 50 MB | ❌ No | ❌ On-demand |
| speech_node (Phase 2) | 10-15% | 150 MB | ❌ No | ❌ Optional |
| audio_beep_node (demo) | <0.1% | 10 MB | ❌ No | ❌ Demo only |
| | | | | |
| **Web Services (On-Demand)** | | | | |
| FastAPI server | 3-5% | 100 MB | ❌ No | ❌ On-demand |
| rosbridge_server | 2-3% | 50 MB | ❌ No | ❌ On-demand |
| | | | | |
| **System Services** | | | | |
| tailscaled (VPN) | <1% | 30 MB | ✅ Yes | ✅ Yes |
| r2d2-powerbutton | <0.1% | 10 MB | ⚠️ Optional | ⚠️ Optional |
| | | | | |
| **Total (Core Only)** | **~16-26%** | **~400 MB** | | |
| **Total (All Active)** | **~26-39%** | **~600 MB** | | |

### 5.2 Service Categorization

#### Always Running (Essential)
- r2d2-camera-perception.service ✅
- r2d2-audio-notification.service ✅
- r2d2-heartbeat.service ✅
- tailscaled.service ✅

**Total:** 16-26% CPU, ~400 MB RAM

#### On-Demand (Start When Needed)
- r2d2-rosbridge.service ❌
- r2d2-web-dashboard.service ❌
- r2d2-camera-stream.service ❌

**Additional:** +5-10% CPU, +200 MB RAM when active

#### Optional (User Choice)
- r2d2-powerbutton.service ⚠️

**Additional:** <0.5% CPU, ~10 MB RAM

---

## 6. Documentation Cross-Reference

### By Document

| Document | Services/Nodes Documented |
|----------|--------------------------|
| `001_ARCHITECTURE_OVERVIEW.md` | All 9 nodes, system architecture |
| `100_PERSON_RECOGNITION_AND_STATUS.md` | Perception, audio notification, LED, logger |
| `111_WEB_DASHBOARD_DOCUMENTATION.md` | Web dashboard, rosbridge, camera stream |
| `200_SPEECH_SYSTEM_REFERENCE.md` | Phase 2 speech system reference (OpenAI Realtime API) |
| `012_VPN_SETUP_AND_REMOTE_ACCESS.md` | Tailscale VPN service |
| `020_POWER_BUTTON_FINAL_DOCUMENTATION.md` | Power button service |
| `102_CAMERA_SETUP_DOCUMENTATION.md` | Camera hardware and camera_node |
| `101_SPEAKER_AUDIO_SETUP_DOCUMENTATION.md` | Audio hardware setup |

### By Service/Node

| Service/Node | Primary Documentation |
|--------------|----------------------|
| camera_node | `102_CAMERA_SETUP_DOCUMENTATION.md` |
| image_listener | `100_PERSON_RECOGNITION_AND_STATUS.md` |
| heartbeat_node | `001_ARCHITECTURE_OVERVIEW.md` |
| camera_stream_node | `111_WEB_DASHBOARD_DOCUMENTATION.md` |
| audio_notification_node | `100_PERSON_RECOGNITION_AND_STATUS.md` |
| status_led_node | `100_PERSON_RECOGNITION_AND_STATUS.md` |
| database_logger_node | `100_PERSON_RECOGNITION_AND_STATUS.md` |
| audio_beep_node | `001_ARCHITECTURE_OVERVIEW.md` |
| speech_node | `200_SPEECH_SYSTEM_REFERENCE.md` |
| Web dashboard | `111_WEB_DASHBOARD_DOCUMENTATION.md` |
| Tailscale VPN | `012_VPN_SETUP_AND_REMOTE_ACCESS.md` |

---

## 7. Next Steps

This inventory document serves as the foundation for:

1. **Optimization Analysis** - Identify which services can be consolidated or disabled
2. **Resource Planning** - Understand resource allocation for Phase 2/3 features
3. **Dependency Mapping** - Clarify which services depend on others
4. **Startup Sequence** - Optimize boot time and service startup order
5. **Cost-Benefit Analysis** - Determine which on-demand services should become always-running

**Recommended Actions:**
- Review with user to identify optimization opportunities
- Analyze which services provide the most value
- Determine which on-demand services should be promoted to always-running
- Identify services that can be eliminated or consolidated

---

**Document Created:** December 17, 2025  
**Status:** Complete inventory ready for analysis  
**Next Review:** After optimization decisions

