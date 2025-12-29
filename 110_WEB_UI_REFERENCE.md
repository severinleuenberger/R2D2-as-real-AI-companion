# R2D2 Web UI System - Reference Documentation
## Complete Architecture and Technical Reference

**Date:** December 17, 2025  
**Status:** ✅ COMPLETE AND OPERATIONAL  
**Platform:** NVIDIA Jetson AGX Orin 64GB + ROS 2 Humble  
**Access:** Via Tailscale VPN (http://100.x.x.x:8079)

---

## Executive Summary

The R2D2 Web UI system uses a **Service Mode** architecture. By default, only a minimal "Wake API" runs (saving resources). The full Web UI is started on-demand.

**Access Points:**
- **Service Mode (Always On):** http://100.x.x.x:8079 (Check status, Start UI)
- **Full Dashboard (On Demand):** http://100.x.x.x:8080 (Control Robot)

The system includes a comprehensive web dashboard with real-time monitoring, service control, volume adjustment, and face recognition training integration.

**Key Capabilities:**
- Real-time person recognition monitoring (RED/BLUE/GREEN states)
- Service control (start/stop/restart systemd services)
- Audio volume control with live parameter updates
- Complete face recognition training interface
- System health monitoring (CPU, GPU, temperature)
- Live camera stream (MJPEG)
- Accessible from anywhere via Tailscale VPN

**For installation instructions**, see: [`111_WEB_UI_INSTALLATION.md`](111_WEB_UI_INSTALLATION.md)  
**For quick reference**, see: [`112_WEB_UI_QUICK_START.md`](112_WEB_UI_QUICK_START.md)

---

## Architecture Overview

### High-Level Architecture

```
┌─────────────────────────────────────────────────────────────────────┐
│                    WEB UI SYSTEM ARCHITECTURE                      │
└─────────────────────────────────────────────────────────────────────┘

INTERNET (Anywhere)
    ↓
Tailscale VPN (Already Configured)
    ↓ (100.x.x.x)
┌─────────────────────────────────────────────────────────────────────┐
│                    JETSON AGX ORIN                                 │
│                                                                     │
│  ┌─────────────────────────────────────────────────────────────┐  │
│  │  FastAPI Web Server (Port 8080)                             │  │
│  │  ├─ REST API Endpoints                                      │  │
│  │  ├─ WebSocket Server (Real-time updates)                    │  │
│  │  └─ Static File Serving (HTML/CSS/JS)                      │  │
│  └─────────────────────────────────────────────────────────────┘  │
│           │                    │                    │              │
│           ↓                    ↓                    ↓              │
│  ┌─────────────────┐  ┌─────────────────┐  ┌─────────────────┐  │
│  │  rosbridge      │  │  ROS 2          │  │  Training       │  │
│  │  (Port 9090)    │  │  Parameter API  │  │  Scripts        │  │
│  └─────────────────┘  └─────────────────┘  └─────────────────┘  │
│           │                    │                    │              │
│           └────────────────────┴────────────────────┘              │
│                              │                                      │
│                              ↓                                      │
│  ┌─────────────────────────────────────────────────────────────┐  │
│  │  ROS 2 System (Existing)                                    │  │
│  │  ├─ /r2d2/perception/person_id (String)                      │  │
│  │  ├─ /r2d2/audio/person_status (JSON)                        │  │
│  │  ├─ /r2d2/heartbeat (JSON: timestamp + status only)         │  │
│  │  └─ /oak/rgb/image_raw (sensor_msgs/Image)                  │  │
│  └─────────────────────────────────────────────────────────────┘  │
│                                                                     │
└─────────────────────────────────────────────────────────────────────┘
```

### Data Flow

```
Browser (Windows Laptop)
    ↓ HTTPS (Tailscale)
FastAPI Web Server (Port 8080)
    ├─ REST API
    │  ├─ Service Control (systemctl)
    │  ├─ Volume Control (ROS2 param)
    │  └─ Training Control (subprocess)
    │
    ├─ WebSocket (/ws)
    │  └─ Training logs streaming
    │
    └─ Static Files (HTML/CSS/JS)
        └─ roslibjs (WebSocket to rosbridge)
            ↓
rosbridge (Port 9090)
    ↓
ROS 2 Topics
    ├─ /r2d2/audio/person_status
    ├─ /r2d2/perception/person_id
    ├─ /r2d2/heartbeat
    └─ /oak/rgb/image_raw
```

---

## Core Components

### 1. FastAPI Web Server

**Technology:** FastAPI (Python async web framework)

**Port:** 8080 (HTTP)

**Key Responsibilities:**
- Serve web dashboard (HTML/CSS/JS)
- REST API endpoints for control operations
- WebSocket server for real-time training logs
- System command execution (systemctl, ROS2 params)
- Background task management

**Endpoints:**
- `/` - Web dashboard HTML
- `/api/services/*` - Service control
- `/api/audio/*` - Volume control
- `/api/training/*` - Training operations
- `/api/status/*` - System status
- `/api/system/health` - System metrics (CPU, GPU, Disk, Temp) - on-demand
- `/ws` - WebSocket for training logs

### 2. rosbridge Suite

**Technology:** rosbridge_suite (ROS 2 WebSocket bridge)

**Port:** 9090 (WebSocket)

**Purpose:** Bridges ROS 2 topics/services to WebSocket for browser access

**Subscribed Topics:**
- `/r2d2/audio/person_status` - Recognition state (JSON)
- `/r2d2/perception/person_id` - Person identification
- `/r2d2/perception/face_count` - Face count
- `/r2d2/heartbeat` - System alive status (lightweight ping)

**Protocol:** roslibjs (JavaScript library for rosbridge)

### 3. Web Dashboard Frontend

**Technology:** HTML5 + CSS3 + JavaScript (ES6+)

**Libraries:**
- roslibjs (ROS 2 WebSocket client)
- Native Fetch API (REST calls)
- Native WebSocket API (training logs)

**Design:**
- Dark futuristic Star Wars theme
- Optimized for 1920×1200 display (no scrolling)
- Responsive grid layout
- Real-time updates
- Color-coded status indicators

**UI Components:**
- Recognition status panel (RED/BLUE/GREEN)
- Service control panel (start/stop/restart)
- Volume control panel (slider + presets)
- Training panel (7 menu options)
- System health panel (CPU/GPU/temperature)
- Camera stream panel (MJPEG)
- Event stream panel (live logs)

### 4. Camera Stream Service

**Node:** `camera_stream_node.py`

**Port:** 8081 (HTTP MJPEG stream)

**Purpose:** Convert ROS 2 camera topic to MJPEG stream

**Subscribed Topic:** `/oak/rgb/image_raw`

**Features:**
- Configurable FPS (default: 15)
- Configurable quality (default: 85)
- Max resolution: 1280×720 (downscales if needed)
- On-demand start/stop from dashboard

**Stream URL:** `http://100.x.x.x:8081/stream`

### 5. Enhanced Heartbeat Service

**Node:** `heartbeat_node.py`

**Published Topic:** `/r2d2/heartbeat` (std_msgs/String, JSON format)

**Frequency:** 1 Hz

**Metrics Collected:**
- CPU usage (%)
- GPU usage (%)
- Temperature (°C)
- Timestamp

**Data Format:**
```json
{
  "timestamp": "2025-12-17T10:30:00",
  "status": "running",
  "cpu_percent": 15.5,
  "gpu_percent": 8.2,
  "temperature_c": 42.3
}
```

**Source:** Uses `tegrastats` utility (Jetson-specific)

---

## Technology Stack

### Backend

| Component | Technology | Version | Purpose |
|-----------|-----------|---------|---------|
| **Web Framework** | FastAPI | 0.104+ | Async HTTP/WebSocket server |
| **ROS 2 Bridge** | rosbridge_suite | Humble | WebSocket bridge to ROS 2 |
| **ROS 2 Client** | rclpy | Humble | Parameter control |
| **Process Control** | subprocess | stdlib | System command execution |
| **Python Runtime** | Python | 3.10 | Application runtime |

### Frontend

| Component | Technology | Purpose |
|-----------|-----------|---------|
| **Structure** | HTML5 | Page structure |
| **Styling** | CSS3 | Visual design |
| **Logic** | JavaScript (ES6+) | Client-side logic |
| **ROS Client** | roslibjs | WebSocket to rosbridge |
| **HTTP Client** | Fetch API | REST API calls |
| **WebSocket Client** | WebSocket API | Training logs |

### Infrastructure

| Component | Technology | Purpose |
|-----------|-----------|---------|
| **VPN** | Tailscale | Secure remote access |
| **Service Manager** | systemd | Service control |
| **Camera Stream** | MJPEG over HTTP | Live video |
| **Virtual Env** | venv | Python dependency isolation |

---

## API Reference

### Service Control API

**Get All Services Status**
```http
GET /api/services/status

Response: {
    "audio": {
        "service_name": "audio",
        "full_name": "r2d2-audio-notification.service",
        "status": "active",
        "enabled": true,
        "state": "active",
        "substate": "running"
    },
    "camera": {...},
    "powerbutton": {...},
    "heartbeat": {...},
    "camera-stream": {...}
}
```

**Start Service**
```http
POST /api/services/{service_name}/start

Response: {
    "success": true,
    "message": "audio started successfully"
}
```

**Stop Service**
```http
POST /api/services/{service_name}/stop

Response: {
    "success": true,
    "message": "audio stopped successfully"
}
```

**Restart Service**
```http
POST /api/services/{service_name}/restart

Response: {
    "success": true,
    "message": "audio restarted successfully"
}
```

### Audio Control API

**Get Volume**
```http
GET /api/audio/volume

Response: {
    "volume": 0.05
}
```

**Set Volume**
```http
POST /api/audio/volume
Content-Type: application/json

Body: {
    "volume": 0.3
}

Response: {
    "success": true,
    "volume": 0.3
}
```

**Get All Audio Parameters**
```http
GET /api/audio/parameters

Response: {
    "audio_volume": 0.05,
    "target_person": "target_person",
    "alsa_device": "hw:1,0",
    "jitter_tolerance_seconds": 5.0,
    "loss_confirmation_seconds": 15.0,
    "cooldown_seconds": 2.0,
    "recognition_cooldown_after_loss_seconds": 5.0,
    "enabled": true
}
```

### System Health API

**Get System Metrics (On-Demand)**
```http
GET /api/system/health

Response: {
    "cpu_percent": 23.5,
    "gpu_percent": 0.0,
    "disk_percent": 81.9,
    "temperature_c": 45.2
}
```

**Note:** System metrics (CPU, GPU, Disk, Temperature) are collected on-demand only when this endpoint is called. This saves resources compared to the previous approach where metrics were published continuously via the heartbeat topic.

The heartbeat topic (`/r2d2/heartbeat`) now only contains a lightweight "alive" ping with `timestamp` and `status` fields.

### Training API

**Start Capture Training**
```http
POST /api/training/capture
Content-Type: application/json

Body: {
    "person_name": "alice",
    "option": 1
}

Response: {
    "task_id": "uuid-here",
    "status": "running",
    "message": "Capture started"
}
```

**Get Training Status**
```http
GET /api/training/status/{task_id}

Response: {
    "task_id": "uuid-here",
    "type": "capture",
    "person_name": "alice",
    "status": "running",
    "progress": 0,
    "current_step": "Capturing images...",
    "logs": ["Step 1: Starting capture...", "Step 2: Task 1 complete"],
    "started_at": "2025-12-17T14:23:15",
    "completed_at": null
}
```

**Train Model**
```http
POST /api/training/train
Content-Type: application/json

Body: {
    "person_name": "alice"
}

Response: {
    "task_id": "uuid-here",
    "status": "running",
    "message": "Training started"
}
```

**List People/Models**
```http
GET /api/training/list

Response: {
    "trained_people": [
        {
            "name": "target_person",
            "model_size_kb": 33.2,
            "image_count": 80,
            "model_path": "/home/severin/dev/r2d2/data/face_recognition/models/target_person_lbph.xml"
        }
    ],
    "datasets": [
        {
            "name": "target_person",
            "image_count": 80
        }
    ]
}
```

**Delete Person**
```http
DELETE /api/training/{person_name}

Response: {
    "success": true,
    "message": "Deleted target_person"
}
```

### Status API

**Get ROS Nodes**
```http
GET /api/status/nodes

Response: {
    "nodes": [
        "/camera_node",
        "/image_listener",
        "/audio_notification_node",
        "/heartbeat_node",
        ...
    ]
}
```

**Get ROS Topics**
```http
GET /api/status/topics

Response: {
    "topics": [
        "/oak/rgb/image_raw",
        "/r2d2/perception/person_id",
        "/r2d2/audio/person_status",
        "/r2d2/heartbeat",
        ...
    ]
}
```

---

## ROS 2 Integration

### Subscribed Topics (via rosbridge)

| Topic | Type | Rate | Purpose |
|-------|------|------|---------|
| `/r2d2/audio/person_status` | std_msgs/String (JSON) | 10 Hz | Recognition state |
| `/r2d2/perception/person_id` | std_msgs/String | 6.5 Hz | Person identification |
| `/r2d2/perception/face_count` | std_msgs/Int32 | 13 Hz | Face count |
| `/r2d2/heartbeat` | std_msgs/String (JSON) | 1 Hz | Alive status (timestamp + running) |
| `/oak/rgb/image_raw` | sensor_msgs/Image | 30 Hz | Camera feed (for stream) |

### ROS 2 Parameter Control

**Controlled Parameters:**
- `/audio_notification_node/audio_volume` - Volume control (0.0-1.0)
- Other audio parameters (accessible but not controlled via UI yet)

**Method:** Uses `ros2 param set` via subprocess

### Systemd Services Controlled

| Service Name | Systemd Unit | Purpose |
|--------------|-------------|---------|
| `audio` | r2d2-audio-notification.service | Audio alerts |
| `camera` | r2d2-camera-perception.service | Camera + perception |
| `powerbutton` | r2d2-powerbutton.service | Shutdown control |
| `heartbeat` | r2d2-heartbeat.service | Health monitoring |
| `camera-stream` | r2d2-camera-stream.service | MJPEG stream |

---

## Security Considerations

### Access Control

**Current Implementation:**
- ✅ **Tailscale VPN Only:** Services bind strictly to `100.x.x.x`.
- ✅ **Local Access Blocked:** Services are NOT accessible from local WiFi (192.168.x.x).
- ✅ **Service Mode:** Full Web UI is stopped by default to reduce attack surface.

### Input Validation
- ✅ **Regex Validation:** Person names must be alphanumeric + underscore.
- ✅ **Path Traversal Protection:** File operations are strictly confined to data directories.
- ✅ **Service Whitelisting:** Only specific r2d2-* services can be controlled.

**Sudo Configuration:**
```bash
# /etc/sudoers.d/r2d2-services
severin ALL=(ALL) NOPASSWD: /bin/systemctl start r2d2-*, /bin/systemctl stop r2d2-*, /bin/systemctl restart r2d2-*
```

**Security Notes:**
- Limited to specific systemctl commands
- Only for r2d2-* services
- No password required (for convenience)

**Alternative Approaches:**
- Use systemd user services (no sudo needed)
- Create wrapper script with setuid
- Use polkit for fine-grained permissions

### Input Validation

**Implemented:**
- ✅ Volume range validation (0.0-1.0)
- ✅ Person name validation (alphanumeric + underscore)
- ✅ Service name validation (whitelist)

**Additional Recommendations:**
- Sanitize all user inputs
- Validate file paths
- Prevent path traversal attacks
- Rate limit API endpoints

---

## Performance Metrics

### System Resource Usage

**Web Dashboard:**
- CPU: ~2-3% (FastAPI server)
- Memory: ~100 MB (Python process)
- Network: Minimal (event-driven updates)

**rosbridge:**
- CPU: ~2-3% (WebSocket bridge)
- Memory: ~50 MB
- Network: ~10-50 Kbps (depends on topic rates)

**Camera Stream (when active):**
- CPU: ~5-7% (MJPEG encoding)
- Memory: ~150 MB
- Network: ~500-1000 Kbps (depends on quality/FPS)

**Total Overhead:** ~5-10% CPU, ~200-300 MB memory

### Update Frequencies

| Component | Update Rate | Latency |
|-----------|------------|---------|
| Recognition status | 10 Hz | <100 ms |
| Person ID | 6.5 Hz | <100 ms |
| Face count | 13 Hz | <100 ms |
| System health | 1 Hz | <100 ms |
| Service status | 0.2 Hz (5s poll) | N/A |

---

## File Locations

### Web Dashboard

| Component | Location |
|-----------|----------|
| **Main Directory** | `~/dev/r2d2/web_dashboard/` |
| **FastAPI App** | `~/dev/r2d2/web_dashboard/app/` |
| **HTML Dashboard** | `~/dev/r2d2/web_dashboard/app/templates/index.html` |
| **Static Files** | `~/dev/r2d2/web_dashboard/app/static/` |
| **Launch Files** | `~/dev/r2d2/web_dashboard/launch/` |
| **Scripts** | `~/dev/r2d2/web_dashboard/scripts/` |
| **Virtual Environment** | `~/dev/r2d2/web_dashboard/web_dashboard_env/` |

### Configuration

| File | Location |
|------|----------|
| **Requirements** | `~/dev/r2d2/web_dashboard/requirements.txt` |
| **Startup Script** | `~/dev/r2d2/web_dashboard/start_server.sh` |
| **rosbridge Launch** | `~/dev/r2d2/web_dashboard/launch/rosbridge.launch.py` |

### Training Scripts

| Script | Location |
|--------|----------|
| **Training Manager** | `~/dev/r2d2/tests/face_recognition/train_manager.py` |
| **Capture Script** | `~/dev/r2d2/tests/face_recognition/1_capture_training_data.py` |
| **Train Script** | `~/dev/r2d2/tests/face_recognition/2_train_recognizer.py` |
| **Test Script** | `~/dev/r2d2/tests/face_recognition/3_test_recognizer_demo.py` |

---

## Troubleshooting Guide

### Issue: Dashboard Not Loading

**Symptoms:** Browser shows error or blank page

**Solutions:**
1. Check if FastAPI server is running:
   ```bash
   ps aux | grep "app.main"
   ```

2. Check if port 8080 is accessible:
   ```bash
   curl http://localhost:8080
   ```

3. Verify Tailscale VPN connection:
   ```bash
   tailscale status
   ```

4. Check firewall (should allow port 8080):
   ```bash
   sudo ufw allow 8080/tcp
   ```

### Issue: No Real-time Updates

**Symptoms:** Dashboard loads but status doesn't update

**Solutions:**
1. Check if rosbridge is running:
   ```bash
   ros2 node list | grep rosbridge
   ```

2. Check rosbridge port:
   ```bash
   netstat -tuln | grep 9090
   ```

3. Check browser console for WebSocket errors (F12 → Console)

4. Verify ROS 2 topics are publishing:
   ```bash
   ros2 topic hz /r2d2/audio/person_status
   ```

### Issue: Service Control Not Working

**Symptoms:** Buttons don't work or show errors

**Solutions:**
1. Check sudo permissions:
   ```bash
   sudo systemctl status r2d2-audio-notification.service
   # Should work without password
   ```

2. Verify sudoers configuration:
   ```bash
   sudo cat /etc/sudoers.d/r2d2-services
   ```

3. Check service names:
   ```bash
   systemctl list-units | grep r2d2
   ```

### Issue: Volume Control Not Working

**Symptoms:** Volume slider doesn't change audio

**Solutions:**
1. Verify audio node is running:
   ```bash
   ros2 node list | grep audio_notification_node
   ```

2. Check parameter directly:
   ```bash
   ros2 param get /audio_notification_node audio_volume
   ```

3. Test parameter setting:
   ```bash
   ros2 param set /audio_notification_node audio_volume 0.3
   ```

### Issue: Training Not Working

**Symptoms:** Training tasks fail or don't start

**Solutions:**
1. Verify training scripts exist:
   ```bash
   ls ~/dev/r2d2/tests/face_recognition/1_capture_training_data.py
   ```

2. Check environment variables:
   ```bash
   echo $OPENBLAS_CORETYPE  # Should be ARMV8
   ```

3. Test script manually:
   ```bash
   cd ~/dev/r2d2/tests/face_recognition
   source ~/depthai_env/bin/activate
   export OPENBLAS_CORETYPE=ARMV8
   python3 1_capture_training_data.py
   ```

4. Check directory permissions:
   ```bash
   ls -la ~/dev/r2d2/data/face_recognition/
   ```

### Issue: Camera Stream Not Working

**Symptoms:** Camera stream service starts but no video

**Solutions:**
1. Check if camera is publishing:
   ```bash
   ros2 topic hz /oak/rgb/image_raw
   ```

2. Check camera stream service logs:
   ```bash
   sudo journalctl -u r2d2-camera-stream.service -f
   ```

3. Test stream URL directly:
   ```bash
   curl http://localhost:8081/stream
   ```

4. Verify camera service is running:
   ```bash
   sudo systemctl status r2d2-camera-perception.service
   ```

---

## Future Enhancements

### Short Term
- [ ] HTTPS with Let's Encrypt
- [ ] HTTP basic authentication
- [ ] Chat history search
- [ ] Profile picture upload

### Medium Term
- [ ] Voice chat (WebRTC)
- [ ] Mobile app (React Native)
- [ ] Multi-user support
- [ ] Chat export (PDF/JSON)
- [ ] Google OAuth integration (from planning phase)

### Long Term
- [ ] Real-time video streaming (WebRTC instead of MJPEG)
- [ ] Advanced analytics dashboard
- [ ] Integration with speech system (realtime API)
- [ ] Cloud backup of chat history
- [ ] LLM chat integration (OpenAI/Grok)

---

## References

### Documentation
- [110_WEB_UI_REFERENCE.md](110_WEB_UI_REFERENCE.md) - This document
- [111_WEB_UI_INSTALLATION.md](111_WEB_UI_INSTALLATION.md) - Installation guide
- [112_WEB_UI_QUICK_START.md](112_WEB_UI_QUICK_START.md) - Quick reference
- [001_ARCHITECTURE_OVERVIEW.md](001_ARCHITECTURE_OVERVIEW.md) - System architecture
- [012_VPN_SETUP_AND_REMOTE_ACCESS.md](012_VPN_SETUP_AND_REMOTE_ACCESS.md) - Tailscale VPN

### External Resources
- [FastAPI Documentation](https://fastapi.tiangolo.com/)
- [rosbridge_suite](http://wiki.ros.org/rosbridge_suite)
- [roslibjs Documentation](http://robotwebtools.org/jsdoc/roslibjs/current/)
- [Tailscale Documentation](https://tailscale.com/kb/)

---

**Document Version:** 1.0  
**Last Updated:** December 17, 2025  
**Status:** Complete and operational  
**Platform:** NVIDIA Jetson AGX Orin 64GB with ROS 2 Humble  
**Access:** Via Tailscale VPN


