# R2D2 Systemd Services Reference
## Complete Service Documentation and Management

**Date:** December 26, 2025  
**Status:** Production System Configuration  
**Platform:** NVIDIA Jetson AGX Orin 64GB  
**Total Services:** 11

---

## Overview

The R2D2 system uses systemd services for automatic startup and management of all core components. This document provides complete reference for all services, their dependencies, startup scripts, and troubleshooting.

**Service Categories:**
- **Core Perception (3 services):** Camera, audio notifications, gesture intent
- **Communication (3 services):** Speech node (Fast Mode), REST speech node (R2-D2 Mode), rosbridge
- **Monitoring (1 service):** Heartbeat monitor
- **Web Interface (2 services):** Web dashboard, wake API
- **Hardware (2 services):** Camera stream, power button

---

## Service Summary Table

| Service | Auto-Start | Script Location | Purpose |
|---------|------------|-----------------|---------|
| r2d2-camera-perception | ✅ Enabled | *inline launch* | Face & gesture recognition |
| r2d2-audio-notification | ✅ Enabled | `scripts/start/start_audio_notification.sh` | Audio alerts & LED control |
| r2d2-gesture-intent | ✅ Enabled | *inline launch* | Gesture-to-speech control |
| r2d2-speech-node | ✅ Enabled | `scripts/start/start_speech_node.sh` | Fast Mode (OpenAI Realtime) |
| r2d2-rest-speech-node | ✅ Enabled | `scripts/start/start_rest_speech_node.sh` | R2-D2 Mode (REST APIs) |
| r2d2-heartbeat | ✅ Enabled | `scripts/start/start_heartbeat.sh` | System health monitoring |
| r2d2-powerbutton | ✅ Enabled | `/usr/local/bin/r2d2_power_button.py` | Physical power button |
| r2d2-wake-api | ✅ Enabled | *inline python* | Minimal service mode API |
| r2d2-rosbridge | ❌ Disabled | `scripts/start/start_rosbridge.sh` | WebSocket bridge (on-demand) |
| r2d2-camera-stream | ❌ Disabled | `scripts/start/start_camera_stream.sh` | MJPEG stream (conflicts!) |
| r2d2-web-dashboard | ❌ Disabled | `web_dashboard/scripts/start_web_dashboard.sh` | Web UI (on-demand) |

---

## Core Perception Services

### 1. r2d2-camera-perception.service

**Purpose:** Primary perception system - face recognition, gesture recognition, and camera control

**Status:** ✅ Auto-start enabled

**Location:** `/etc/systemd/system/r2d2-camera-perception.service`

**ExecStart:**
```bash
/bin/bash -c 'source install/setup.bash && ros2 launch r2d2_bringup r2d2_camera_perception.launch.py enable_face_recognition:=true enable_gesture_recognition:=true gesture_recognition_model_path:=/home/severin/dev/r2d2/data/gesture_recognition/models/severin_gesture_classifier.pkl'
```

**Working Directory:** `/home/severin/dev/r2d2/ros2_ws`

**Dependencies:**
- After: `network.target`
- Wants: `r2d2-audio-notification.service`

**ROS 2 Nodes:**
- `/oak_d_camera` or `/camera_node` - OAK-D Lite camera driver
- `/image_listener` - Face & gesture recognition

**Topics Published:**
- `/oak/rgb/image_raw` (30 Hz) - Raw camera feed
- `/r2d2/perception/person_id` (6.5 Hz) - Recognized person
- `/r2d2/perception/face_count` (13 Hz) - Number of faces
- `/r2d2/perception/gesture_event` (event) - Gesture detection
- `/r2d2/perception/face_confidence` (6.5 Hz) - Recognition confidence

**Management:**
```bash
# Status
sudo systemctl status r2d2-camera-perception

# Restart
sudo systemctl restart r2d2-camera-perception

# Logs
sudo journalctl -u r2d2-camera-perception -f
```

**Troubleshooting:**
- **Service fails to start:** Check camera USB connection (`lsusb | grep Movidius`)
- **No topics published:** Verify ROS 2 environment sourced correctly
- **High CPU usage:** Increase `recognition_frame_skip` or `gesture_frame_skip`

---

### 2. r2d2-audio-notification.service

**Purpose:** Audio feedback system with RED/GREEN/BLUE status machine, LED control, and event logging

**Status:** ✅ Auto-start enabled

**Location:** `/etc/systemd/system/r2d2-audio-notification.service`

**ExecStart:**
```bash
/home/severin/dev/r2d2/start_audio_notification.sh
```

**⚠️ PATH ISSUE:** Script should be at `scripts/start/start_audio_notification.sh` (needs service file update)

**Working Directory:** `/home/severin/dev/r2d2`

**Dependencies:**
- After: `network.target`

**ROS 2 Nodes:**
- `/audio_notification_node` - Status machine & audio playback
- `/status_led_node` - GPIO LED control
- `/database_logger_node` - Event logging

**Topics Subscribed:**
- `/r2d2/perception/person_id` (String) - Person recognition
- `/r2d2/perception/face_count` (Int32) - Face detection

**Topics Published:**
- `/r2d2/audio/person_status` (String JSON, 10 Hz) - RED/GREEN/BLUE status
- `/r2d2/audio/notification_event` (String, event) - Audio event triggers

**Configuration:** `ros2_ws/src/r2d2_audio/config/audio_params.yaml`

**Management:**
```bash
# Status
sudo systemctl status r2d2-audio-notification

# Restart
sudo systemctl restart r2d2-audio-notification

# Logs (with filter)
sudo journalctl -u r2d2-audio-notification -f | grep -E "RED|GREEN|BLUE|recognized"
```

**Troubleshooting:**
- **No audio:** Check volume (`audio_volume: 0.02` in config), verify speaker wiring
- **LED not working:** Check GPIO 17 export (`ls /sys/class/gpio/gpio17/`)
- **Status stuck:** Check person recognition is publishing (`ros2 topic hz /r2d2/perception/person_id`)

---

### 3. r2d2-gesture-intent.service

**Purpose:** Translates gestures into speech system actions (start/stop conversation)

**Status:** ✅ Auto-start enabled

**Location:** `/etc/systemd/system/r2d2-gesture-intent.service`

**ExecStart:**
```bash
/bin/bash -c 'source install/setup.bash && ros2 launch r2d2_gesture gesture_intent.launch.py enabled:=true cooldown_start_seconds:=5.0 cooldown_stop_seconds:=3.0 auto_shutdown_enabled:=true auto_shutdown_timeout_seconds:=35.0 auto_restart_on_return:=false audio_feedback_enabled:=true'
```

**Working Directory:** `/home/severin/dev/r2d2/ros2_ws`

**Dependencies:**
- After: `network.target`, `r2d2-camera-perception.service`
- Requires: `r2d2-camera-perception.service`

**ROS 2 Nodes:**
- `/gesture_intent_node` - Gesture gating & service calls

**Topics Subscribed:**
- `/r2d2/perception/gesture_event` (String) - Gesture detection
- `/r2d2/audio/person_status` (String JSON) - Person recognition status
- `/r2d2/speech/session_status` (String JSON) - Speech session state
- `/r2d2/speech/voice_activity` (String JSON) - VAD updates

**Service Clients:**
- `/r2d2/speech/start_session` (Trigger) - Start conversation
- `/r2d2/speech/stop_session` (Trigger) - Stop conversation

**Configuration:** `ros2_ws/src/r2d2_gesture/config/gesture_params.yaml`

**Management:**
```bash
# Status
sudo systemctl status r2d2-gesture-intent

# Restart
sudo systemctl restart r2d2-gesture-intent

# Logs
sudo journalctl -u r2d2-gesture-intent -f
```

**Troubleshooting:**
- **Gestures ignored:** Check person_status is RED (`ros2 topic echo /r2d2/audio/person_status`)
- **Service not available:** Verify speech node is running (`ros2 service list | grep speech`)
- **Cooldown issues:** Adjust `cooldown_start_seconds` or `cooldown_stop_seconds`

---

## Communication Services

### 4. r2d2-speech-node.service

**Purpose:** OpenAI Realtime API conversation system

**Status:** ✅ Auto-start enabled

**Location:** `/etc/systemd/system/r2d2-speech-node.service`

**ExecStart:**
```bash
/bin/bash /home/severin/dev/r2d2/scripts/start/start_speech_node.sh
```

**✅ PATH FIXED:** Dec 24, 2025 - Updated to correct `scripts/start/` location

**Working Directory:** `/home/severin/dev/r2d2/ros2_ws`

**Dependencies:**
- After: `network.target`

**ROS 2 Nodes:**
- `/speech_node` - Lifecycle managed OpenAI conversation

**Services Provided:**
- `/r2d2/speech/start_session` (Trigger) - Start conversation
- `/r2d2/speech/stop_session` (Trigger) - Stop conversation

**Topics Published:**
- `/r2d2/speech/session_status` (String JSON) - Session state (connected/disconnected)
- `/r2d2/speech/voice_activity` (String JSON) - VAD state (speaking/silent)

**Configuration:** `ros2_ws/src/r2d2_speech/config/speech_params.yaml`

**Management:**
```bash
# Status
sudo systemctl status r2d2-speech-node

# Restart
sudo systemctl restart r2d2-speech-node

# Logs
sudo journalctl -u r2d2-speech-node -f

# Lifecycle commands
ros2 lifecycle get /speech_node
ros2 lifecycle set /speech_node configure
ros2 lifecycle set /speech_node activate
```

**Troubleshooting:**
- **Service fails to start:** Check script path is correct, verify OpenAI API key
- **No session starts:** Check service availability (`ros2 service list | grep speech`)
- **Connection issues:** Verify network connectivity, check API key validity

**Recent Fix (Dec 24, 2025):**
- **Issue:** Speech wouldn't start after index finger gesture
- **Root Cause:** ExecStart pointed to old path `/home/severin/dev/r2d2/start_speech_node.sh`
- **Solution:** Updated to `/home/severin/dev/r2d2/scripts/start/start_speech_node.sh`
- **Impact:** Phase 4 → Phase 5 transition now works correctly

---

### 5. r2d2-rest-speech-node.service

**Purpose:** R2-D2 Mode conversation system using OpenAI REST APIs (STT → LLM → TTS)

**Status:** ✅ Auto-start enabled (added December 25, 2025)

**Location:** `/etc/systemd/system/r2d2-rest-speech-node.service`

**ExecStart:**
```bash
/bin/bash /home/severin/dev/r2d2/scripts/start/start_rest_speech_node.sh
```

**Working Directory:** `/home/severin/dev/r2d2/ros2_ws`

**Dependencies:**
- After: `network.target`

**ROS 2 Nodes:**
- `/rest_speech_node` - Lifecycle managed R2-D2 Mode conversation

**Services Provided:**
- `/r2d2/speech/intelligent/start_session` (Trigger) - Start R2-D2 Mode
- `/r2d2/speech/intelligent/stop_session` (Trigger) - Stop R2-D2 Mode
- `/r2d2/speech/intelligent/process_turn` (Trigger) - Process one conversation turn

**Topics Published:**
- `/r2d2/speech/rest_session_status` (String JSON) - Session state
- `/r2d2/speech/rest_user_transcript` (String) - User's transcribed speech
- `/r2d2/speech/rest_assistant_transcript` (String) - R2-D2's response

**Configuration:** `ros2_ws/src/r2d2_speech/config/speech_params.yaml`

**R2-D2 Mode Personality (December 2025):**
- **LLM Model:** gpt-4o (fast responses, ~2-3s)
- **TTS Voice:** echo (robotic character)
- **Character:** Terse, mission-oriented, uses [beeps], [chirps], [whistles] as flavor
- **Trigger:** Open hand gesture (🖐️)

**Management:**
```bash
# Status
sudo systemctl status r2d2-rest-speech-node

# Restart
sudo systemctl restart r2d2-rest-speech-node

# Logs
sudo journalctl -u r2d2-rest-speech-node -f

# Lifecycle commands
ros2 lifecycle get /rest_speech_node
```

**Troubleshooting:**
- **Service fails to start:** Check script path, verify OpenAI API key in `~/.r2d2/.env`
- **No response after open hand:** Check service availability (`ros2 service list | grep intelligent`)
- **TTS not playing:** Verify audio output device, check pydub/simpleaudio installed

---

### 6. r2d2-rosbridge.service

**Purpose:** WebSocket bridge for web dashboard communication

**Status:** ❌ Auto-start disabled (on-demand only)

**Location:** `/etc/systemd/system/r2d2-rosbridge.service`

**ExecStart:**
```bash
/home/severin/dev/r2d2/start_rosbridge.sh
```

**⚠️ PATH ISSUE:** Script should be at `scripts/start/start_rosbridge.sh` (needs service file update)

**Working Directory:** `/home/severin/dev/r2d2`

**Dependencies:**
- After: `network.target`

**ROS 2 Nodes:**
- `rosbridge_websocket` - WebSocket server (port 9090)

**Purpose:** Enables web browser to subscribe to ROS 2 topics via WebSocket

**Management:**
```bash
# Manual start
sudo systemctl start r2d2-rosbridge

# Status
sudo systemctl status r2d2-rosbridge

# Stop
sudo systemctl stop r2d2-rosbridge
```

**Troubleshooting:**
- **Port conflict:** Check if port 9090 is already in use (`netstat -tulpn | grep 9090`)
- **Connection refused:** Verify rosbridge is actually running (`ros2 node list | grep rosbridge`)

---

## Monitoring Services

### 6. r2d2-heartbeat.service

**Purpose:** System health monitoring with periodic status checks

**Status:** ✅ Auto-start enabled

**Location:** `/etc/systemd/system/r2d2-heartbeat.service`

**ExecStart:**
```bash
/home/severin/dev/r2d2/start_heartbeat.sh
```

**⚠️ PATH ISSUE:** Script should be at `scripts/start/start_heartbeat.sh` (needs service file update)

**Working Directory:** `/home/severin/dev/r2d2`

**Dependencies:**
- After: `network.target`

**Purpose:** Monitors system health, logs status, prevents silent failures

**Management:**
```bash
# Status
sudo systemctl status r2d2-heartbeat

# Restart
sudo systemctl restart r2d2-heartbeat

# Logs
sudo journalctl -u r2d2-heartbeat -f
```

---

## Web Interface Services

### 7. r2d2-web-dashboard.service

**Purpose:** FastAPI REST API and web UI for system control

**Status:** ❌ Auto-start disabled (on-demand only)

**Location:** `/etc/systemd/system/r2d2-web-dashboard.service`

**ExecStart:**
```bash
/home/severin/dev/r2d2/web_dashboard/scripts/start_web_dashboard.sh
```

**Working Directory:** `/home/severin/dev/r2d2/web_dashboard`

**Dependencies:**
- After: `network.target`

**Purpose:** Web interface for system monitoring and control

**Management:**
```bash
# Manual start
~/dev/r2d2/scripts/start_web_dashboard.sh

# Status
sudo systemctl status r2d2-web-dashboard

# Stop
~/dev/r2d2/scripts/stop_web_dashboard.sh
```

---

### 8. r2d2-wake-api.service

**Purpose:** Minimal API for remote system wake/control

**Status:** ✅ Auto-start enabled

**Location:** `/etc/systemd/system/r2d2-wake-api.service`

**ExecStart:**
```bash
/bin/bash -c "source /opt/ros/humble/setup.bash && /home/severin/dev/r2d2/web_dashboard/web_dashboard_env/bin/python3 wake_api.py"
```

**Working Directory:** `/home/severin/dev/r2d2/web_dashboard`

**Dependencies:**
- After: `network.target`

**Purpose:** Lightweight API for remote system control when full dashboard is not needed

**Management:**
```bash
# Status
sudo systemctl status r2d2-wake-api

# Restart
sudo systemctl restart r2d2-wake-api

# Logs
sudo journalctl -u r2d2-wake-api -f
```

---

## Hardware Services

### 9. r2d2-camera-stream.service

**Purpose:** MJPEG video stream for web dashboard

**Status:** ❌ Auto-start disabled (on-demand only)

**Location:** `/etc/systemd/system/r2d2-camera-stream.service`

**ExecStart:**
```bash
/home/severin/dev/r2d2/start_camera_stream.sh
```

**⚠️ PATH ISSUE:** Script should be at `scripts/start/start_camera_stream.sh` (needs service file update)

**Working Directory:** `/home/severin/dev/r2d2`

**Dependencies:**
- After: `network.target`

**⚠️ CRITICAL: MUTUALLY EXCLUSIVE WITH r2d2-camera-perception**

**Why disabled:**
- Both services require exclusive access to OAK-D Lite camera
- Camera-perception is core functionality (auto-start)
- Camera-stream is for web preview only (on-demand)
- **NEVER enable both simultaneously!**

**Management:**
```bash
# Only start if camera-perception is stopped!
sudo systemctl stop r2d2-camera-perception
sleep 2
sudo systemctl start r2d2-camera-stream

# Stop and restore
sudo systemctl stop r2d2-camera-stream
sleep 2
sudo systemctl start r2d2-camera-perception
```

---

### 10. r2d2-powerbutton.service

**Purpose:** Physical power button handler (shutdown, reboot)

**Status:** ✅ Auto-start enabled

**Location:** `/etc/systemd/system/r2d2-powerbutton.service`

**ExecStart:**
```bash
/usr/bin/python3 /usr/local/bin/r2d2_power_button.py
```

**Dependencies:**
- After: `multi-user.target`

**Purpose:** Handles physical button press for graceful shutdown/reboot

**Management:**
```bash
# Status
sudo systemctl status r2d2-powerbutton

# Restart
sudo systemctl restart r2d2-powerbutton

# Logs
sudo journalctl -u r2d2-powerbutton -f
```

---

## Service Dependencies and Boot Order

### Boot Sequence (Enabled Services)

```
System Boot
    ↓
network.target
    ↓
┌───────────────────────┴──────────────────────┐
↓                                              ↓
r2d2-audio-notification (independent)    r2d2-heartbeat (independent)
r2d2-speech-node (independent)           r2d2-powerbutton (independent)
r2d2-wake-api (independent)
    ↓ (2 seconds)
r2d2-camera-perception (wants audio)
    ↓ (3 seconds)
r2d2-gesture-intent (requires camera)
    ↓
System Ready (~5-7 seconds total)
```

### Service Relationships

**Independent (start immediately):**
- r2d2-audio-notification
- r2d2-speech-node
- r2d2-heartbeat
- r2d2-powerbutton
- r2d2-wake-api

**Dependent (wait for others):**
- r2d2-camera-perception → Wants: audio-notification
- r2d2-gesture-intent → Requires: camera-perception

**Disabled (manual only):**
- r2d2-rosbridge (web dashboard support)
- r2d2-web-dashboard (on-demand UI)
- r2d2-camera-stream (conflicts with camera-perception!)

---

## Service Path Issues and Fixes

### Services Needing Path Updates

The following services still reference old root-level script paths and should be updated to `scripts/start/` locations:

| Service | Current Path | Should Be |
|---------|--------------|-----------|
| r2d2-audio-notification | `/home/severin/dev/r2d2/start_audio_notification.sh` | `scripts/start/start_audio_notification.sh` |
| r2d2-heartbeat | `/home/severin/dev/r2d2/start_heartbeat.sh` | `scripts/start/start_heartbeat.sh` |
| r2d2-rosbridge | `/home/severin/dev/r2d2/start_rosbridge.sh` | `scripts/start/start_rosbridge.sh` |
| r2d2-camera-stream | `/home/severin/dev/r2d2/start_camera_stream.sh` | `scripts/start/start_camera_stream.sh` |

**How to fix:**
```bash
# For each service:
sudo nano /etc/systemd/system/<service-name>.service

# Update ExecStart line:
ExecStart=/home/severin/dev/r2d2/scripts/start/<script-name>.sh

# Reload and restart:
sudo systemctl daemon-reload
sudo systemctl restart <service-name>
```

### Recently Fixed (Dec 24, 2025)

✅ **r2d2-speech-node.service** - Updated ExecStart path, now working correctly

---

## Common Management Tasks

### View All R2D2 Services

```bash
# List all services
systemctl list-units "r2d2-*" --all

# Check which are enabled
systemctl list-unit-files "r2d2-*"

# Quick status check
for service in /etc/systemd/system/r2d2-*.service; do 
  name=$(basename $service)
  status=$(systemctl is-active $name)
  enabled=$(systemctl is-enabled $name 2>/dev/null || echo "n/a")
  echo "$name: $status ($enabled)"
done
```

### Restart Core Perception Stack

```bash
# Stop in reverse dependency order
sudo systemctl stop r2d2-gesture-intent
sudo systemctl stop r2d2-camera-perception
sudo systemctl stop r2d2-audio-notification

# Wait for clean shutdown
sleep 3

# Start in correct order
sudo systemctl start r2d2-audio-notification
sleep 2
sudo systemctl start r2d2-camera-perception
sleep 3
sudo systemctl start r2d2-gesture-intent

# Verify all running
systemctl is-active r2d2-audio-notification r2d2-camera-perception r2d2-gesture-intent
```

### View Logs for All Services

```bash
# Recent logs from all R2D2 services
sudo journalctl -u "r2d2-*" --since "10 minutes ago"

# Follow logs in real-time
sudo journalctl -u "r2d2-*" -f

# Specific service with filters
sudo journalctl -u r2d2-audio-notification -f | grep -E "RED|GREEN|BLUE"
```

### Enable/Disable Auto-Start

```bash
# Disable auto-start (keep service installed)
sudo systemctl disable <service-name>

# Enable auto-start
sudo systemctl enable <service-name>

# Check status
systemctl is-enabled <service-name>
```

---

## Troubleshooting Guide

### Service Won't Start

```bash
# 1. Check service status
sudo systemctl status <service-name>

# 2. Check recent logs
sudo journalctl -u <service-name> -n 50

# 3. Verify script exists
ls -la /home/severin/dev/r2d2/scripts/start/<script-name>.sh

# 4. Test script manually
bash /home/severin/dev/r2d2/scripts/start/<script-name>.sh

# 5. Check environment
echo $OPENBLAS_CORETYPE  # Should be ARMV8
```

### Service Crashes Immediately

```bash
# 1. Check dependencies are running
systemctl is-active <dependency-service>

# 2. Check ROS 2 environment
source ~/dev/r2d2/ros2_ws/install/setup.bash
ros2 node list

# 3. Verify no port conflicts
netstat -tulpn | grep <port-number>

# 4. Check permissions
ls -la <script-path>
```

### High CPU Usage

```bash
# 1. Identify culprit
top -bn1 | grep -E "python|ros"

# 2. Check topic rates
ros2 topic hz /r2d2/perception/person_id
ros2 topic hz /r2d2/perception/gesture_event

# 3. Adjust frame skip parameters
ros2 param get /image_listener recognition_frame_skip
ros2 param set /image_listener recognition_frame_skip 3
```

---

## Related Documentation

**Installation Guides:**
- `101_PERCEPTION_STATUS_INSTALLATION.md` - Perception system setup
- `201_SPEECH_SYSTEM_INSTALLATION.md` - Speech system setup

**Reference Documentation:**
- `001_ARCHITECTURE_OVERVIEW.md` - System architecture
- `100_PERCEPTION_STATUS_REFERENCE.md` - Perception technical reference
- `200_SPEECH_SYSTEM_REFERENCE.md` - Speech technical reference

**Monitoring:**
- `006_SYSTEM_STATUS_AND_MONITORING.md` - Monitoring tools and commands

**Internal:**
- `000_INTERNAL_AGENT_NOTES.md` - Agent quick reference

---

**Document Version:** 1.0  
**Last Updated:** December 24, 2025  
**Status:** Production service configuration documented  
**Services:** 10 total (7 enabled, 3 disabled)

