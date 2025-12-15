# R2D2 Web Dashboard - Complete Documentation

**Date:** December 12, 2025  
**Version:** 1.0  
**Status:** ✅ Production Ready  
**Platform:** NVIDIA Jetson AGX Orin 64GB + ROS 2 Humble  
**Access:** Via Tailscale VPN (http://100.95.133.26:8080)

---

## Executive Summary

The R2D2 Web Dashboard is a comprehensive web-based interface for monitoring and controlling the R2D2 system remotely. It provides real-time monitoring, service control, volume adjustment, and complete face recognition training integration, all accessible via Tailscale VPN.

**Key Capabilities:**
- ✅ Real-time person recognition monitoring (RED/BLUE/GREEN states)
- ✅ Service control (start/stop/restart systemd services)
- ✅ Audio volume control with presets
- ✅ Complete face recognition training interface (all 7 menu options)
- ✅ Live event stream
- ✅ System metrics display
- ✅ Accessible from anywhere via Tailscale VPN

---

## Table of Contents

1. [Architecture Overview](#1-architecture-overview)
2. [Installation & Setup](#2-installation--setup)
3. [Usage Guide](#3-usage-guide)
4. [API Reference](#4-api-reference)
5. [Features Detail](#5-features-detail)
6. [Troubleshooting](#6-troubleshooting)
7. [Security Considerations](#7-security-considerations)

---

## 1. Architecture Overview

### 1.1 System Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                    BROWSER (Windows Laptop)                │
│  HTML Dashboard (index.html)                                │
│  ├─ JavaScript (dashboard.js)                              │
│  │  ├─ roslibjs (WebSocket to rosbridge)                   │
│  │  └─ Fetch API (REST calls to FastAPI)                   │
│  └─ CSS (dashboard.css)                                     │
└───────────────────────────┬─────────────────────────────────┘
                            │ HTTPS (Tailscale VPN)
                            │ http://100.95.133.26:8080
                            ↓
┌─────────────────────────────────────────────────────────────┐
│                    JETSON AGX ORIN                          │
│                                                              │
│  ┌──────────────────────────────────────────────────────┐  │
│  │  FastAPI Web Server (Port 8080)                      │  │
│  │  ├─ REST API Endpoints                               │  │
│  │  │  ├─ /api/services/* (service control)            │  │
│  │  │  ├─ /api/audio/* (volume control)                 │  │
│  │  │  ├─ /api/training/* (training control)            │  │
│  │  │  └─ /api/status/* (system status)                  │  │
│  │  ├─ Static File Server                                │  │
│  │  │  └─ Serves HTML/CSS/JS files                      │  │
│  │  └─ WebSocket Endpoint (/ws)                          │  │
│  │     └─ Training logs streaming                       │  │
│  └──────────────────────────────────────────────────────┘  │
│           │                    │                             │
│           ↓                    ↓                             │
│  ┌─────────────────┐  ┌─────────────────┐                  │
│  │  rosbridge      │  │  ROS 2          │                  │
│  │  (Port 9090)    │  │  Parameter API  │                  │
│  │  WebSocket      │  │  (rclpy)        │                  │
│  └─────────────────┘  └─────────────────┘                  │
│           │                    │                             │
│           └──────────┬─────────┘                             │
│                      ↓                                        │
│  ┌──────────────────────────────────────────────────────┐  │
│  │  ROS 2 System                                         │  │
│  │  ├─ /r2d2/perception/person_id                        │  │
│  │  ├─ /r2d2/audio/person_status                         │  │
│  │  └─ /r2d2/perception/face_count                       │  │
│  └──────────────────────────────────────────────────────┘  │
│                                                              │
│  ┌──────────────────────────────────────────────────────┐  │
│  │  Systemd Services                                     │  │
│  │  ├─ r2d2-audio-notification.service                  │  │
│  │  └─ r2d2-camera-perception.service                   │  │
│  └──────────────────────────────────────────────────────┘  │
│                                                              │
│  ┌──────────────────────────────────────────────────────┐  │
│  │  Training Scripts                                     │  │
│  │  ├─ train_manager.py                                 │  │
│  │  ├─ 1_capture_training_data.py                        │  │
│  │  └─ 2_train_recognizer.py                            │  │
│  └──────────────────────────────────────────────────────┘  │
└─────────────────────────────────────────────────────────────┘
```

### 1.2 Technology Stack

**Backend:**
- **FastAPI** - Modern async web framework (Python)
- **rosbridge_suite** - ROS 2 WebSocket bridge
- **rclpy** - ROS 2 Python client (for parameter control)
- **subprocess** - System command execution (systemd control)

**Frontend:**
- **HTML5** - Structure
- **CSS3** - Modern responsive styling
- **JavaScript (ES6+)** - Client-side logic
- **roslibjs** - ROS 2 JavaScript library (CDN)
- **WebSocket API** - Real-time updates
- **Fetch API** - REST API calls

**Infrastructure:**
- **Tailscale VPN** - Remote access (already configured)
- **systemd** - Service management
- **Python virtual environment** - Dependency isolation

---

## 2. Installation & Setup

### 2.1 Prerequisites

- ✅ Tailscale VPN configured (see `012_VPN_SETUP_AND_REMOTE_ACCESS.md`)
- ✅ ROS 2 Humble installed
- ✅ R2D2 system running (camera, perception, audio services)

**Default Boot Behavior:**
- After reboot, the system starts in **Recognition Status mode** by default:
  - `r2d2-camera-perception.service` - Starts automatically (face recognition active)
  - `r2d2-audio-notification.service` - Starts automatically (audio alerts active)
  - `r2d2-heartbeat.service` - Starts automatically (system health monitoring)
- `r2d2-camera-stream.service` - Does NOT start automatically (can be started via dashboard if needed)
- To change default behavior, use: `~/dev/r2d2/change_default_boot_services.sh`

### 2.2 Install Dependencies

**Step 1: Install rosbridge_suite**
```bash
sudo apt install ros-humble-rosbridge-suite
```

**Step 2: Create Python Virtual Environment**
```bash
cd ~/dev/r2d2/web_dashboard
python3 -m venv web_dashboard_env
source web_dashboard_env/bin/activate
pip install -r requirements.txt
```

**Step 3: Verify Installation**
```bash
# Check rosbridge
ros2 pkg list | grep rosbridge

# Check Python packages
pip list | grep fastapi
```

### 2.3 Configure Sudo Permissions (for Service Control)

To allow service control without password prompts, add to sudoers:

```bash
sudo visudo
```

Add this line:
```
severin ALL=(ALL) NOPASSWD: /bin/systemctl start r2d2-*, /bin/systemctl stop r2d2-*, /bin/systemctl restart r2d2-*
```

**Security Note:** This allows passwordless sudo only for specific systemctl commands on r2d2 services.

### 2.3.1 Configure Default Boot Services

**Default Configuration:**
The system is configured to start in **Recognition Status mode** after reboot:
- `r2d2-camera-perception.service` - Enabled (starts automatically)
- `r2d2-audio-notification.service` - Enabled (starts automatically)
- `r2d2-heartbeat.service` - Enabled (starts automatically)
- `r2d2-camera-stream.service` - Disabled (does NOT start automatically)

**To Change Default Boot Behavior:**
```bash
# Use the provided script to change default services
~/dev/r2d2/change_default_boot_services.sh

# Or manually configure:
sudo systemctl disable r2d2-camera-stream.service      # Disable camera stream
sudo systemctl enable r2d2-camera-perception.service  # Enable recognition
sudo systemctl enable r2d2-audio-notification.service # Enable audio alerts
```

**Verify Configuration:**
```bash
systemctl is-enabled r2d2-camera-stream.service      # Should show: disabled
systemctl is-enabled r2d2-camera-perception.service   # Should show: enabled
systemctl is-enabled r2d2-audio-notification.service  # Should show: enabled
```

### 2.4 Start Services

**Terminal 1: Start rosbridge**
```bash
cd ~/dev/r2d2/web_dashboard
source /opt/ros/humble/setup.bash
source ~/dev/r2d2/ros2_ws/install/setup.bash
ros2 launch web_dashboard/launch/rosbridge.launch.py
```

**Terminal 2: Start Web Dashboard**
```bash
cd ~/dev/r2d2/web_dashboard
source web_dashboard_env/bin/activate
./start_server.sh
```

**Or use the combined script:**
```bash
cd ~/dev/r2d2/web_dashboard
./scripts/start_web_dashboard.sh
```

### 2.5 Access Dashboard

From Windows laptop (via Tailscale VPN):
```
http://100.95.133.26:8080
```

Replace `100.95.133.26` with your Jetson's Tailscale IP (check with `tailscale ip -4` on Jetson).

---

## 3. Usage Guide

### 3.1 Dashboard Overview

**Main Sections:**

1. **Recognition Status Panel**
   - Large color-coded status indicator (RED/BLUE/GREEN)
   - Current person ID
   - Confidence score
   - Duration in current state
   - Face count

2. **Service Control Panel**
   - List of all R2D2 services
   - Start/Stop/Restart buttons for each service
   - Service status indicators

3. **Volume Control Panel**
   - Volume slider (0-100%)
   - Current volume display
   - Quick preset buttons (5%, 10%, 20%, 50%, 100%)

4. **Training Panel**
   - All 7 training menu options
   - Person name input
   - Training status and logs
   - List of trained people

5. **Real-time Stream Panel**
   - Live event stream
   - Recognition events
   - Status changes
   - System notifications

6. **System Metrics Panel**
   - Topic publication rates
   - System health indicators

### 3.2 Service Control

**To Start a Service:**
1. Find the service in the Service Control panel
2. Click "Start" button
3. Status updates automatically (green = active)

**To Stop a Service:**
1. Click "Stop" button
2. Confirmation not required (immediate action)

**To Restart a Service:**
1. Click "Restart" button
2. Service stops and starts automatically

**Available Services:**
- `audio` - Audio notification service
- `camera` - Camera and perception pipeline
- `powerbutton` - Power button service (optional)
- `camera-stream` - MJPEG video stream service
- `heartbeat` - System health monitoring service

**Default Boot Behavior:**
After system reboot, the following services start automatically:
- ✅ `r2d2-camera-perception.service` - Recognition Status mode active
- ✅ `r2d2-audio-notification.service` - Audio alerts active
- ✅ `r2d2-heartbeat.service` - System health monitoring active
- ❌ `r2d2-camera-stream.service` - Does NOT start automatically (can be started via dashboard)

To change this default behavior, see [Section 2.3.1: Configure Default Boot Services](#231-configure-default-boot-services).

### 3.3 Volume Control

**Using Slider:**
1. Drag the volume slider
2. Volume updates in real-time
3. Changes are applied immediately (no restart needed)

**Using Presets:**
1. Click a preset button (5%, 10%, 20%, 50%, 100%)
2. Volume changes immediately

**Volume Range:** 0% (silent) to 100% (maximum)

### 3.4 Face Recognition Training

**Option 1: Train New Person**
1. Enter person name in input field
2. Click "[1] Train New Person"
3. Training starts (capture → train → test)
4. Watch progress in training status panel

**Option 2: Add More Pictures**
1. Enter existing person name
2. Click "[2] Add More Pictures"
3. Capture additional training images

**Option 3: Retrain Model**
1. Enter person name
2. Click "[3] Retrain Model"
3. Model retrained from existing images

**Option 4: Test Accuracy**
- Not yet implemented (placeholder)

**Option 5: Real-time Test**
- Not yet implemented (placeholder)

**Option 6: List People/Models**
1. Click "[6] List People/Models"
2. Shows all trained people with model sizes and image counts

**Option 7: Delete Person**
1. Click "[7] Delete Person"
2. Enter person name in dialog
3. Confirm deletion
4. Removes all images and model for that person

### 3.5 Monitoring

**Real-time Updates:**
- Recognition status updates automatically via WebSocket
- Service status polls every 5 seconds
- Event stream shows all recognition events

**Connection Status:**
- Top-right indicator shows ROS connection status
- Green = Connected to rosbridge
- Red = Disconnected (will auto-reconnect)

---

## 4. API Reference

### 4.1 Service Control API

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
    "camera": {...}
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

### 4.2 Audio Control API

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

**Get All Parameters**
```http
GET /api/audio/parameters
Response: {
    "audio_volume": 0.05,
    "target_person": "target_person",
    "alsa_device": "hw:1,0",
    "jitter_tolerance_seconds": 5.0,
    "loss_confirmation_seconds": 15.0,
    ...
}
```

### 4.3 Training API

**Start Capture**
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
    "started_at": "2025-12-12T14:23:15",
    "completed_at": null
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

### 4.4 Status API

**Get ROS Nodes**
```http
GET /api/status/nodes
Response: {
    "nodes": [
        "/camera_node",
        "/image_listener",
        "/audio_notification_node",
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
        ...
    ]
}
```

---

## 5. Features Detail

### 5.1 Real-time Monitoring

**How It Works:**
- Browser connects to rosbridge via WebSocket (port 9090)
- JavaScript subscribes to ROS 2 topics using roslibjs
- UI updates automatically when messages arrive

**Monitored Topics:**
- `/r2d2/audio/person_status` - Status JSON (RED/BLUE/GREEN)
- `/r2d2/perception/person_id` - Person identification
- `/r2d2/perception/face_count` - Number of faces detected

**Update Frequency:**
- Status: ~10 Hz (as published by audio_notification_node)
- Person ID: ~6.5 Hz (as published by perception node)
- Face Count: ~13 Hz (as published by perception node)

### 5.2 Three-State Visualization

**Visual Design:**
- Large color-coded indicator (120×120px)
- Smooth color transitions
- Status text and details
- Real-time updates

**States:**
- 🔴 **RED**: Target person recognized
  - Background: Solid red (#f44336)
  - Text: "RED - Target Person Recognized"
  
- 🔵 **BLUE**: No person (idle)
  - Background: Solid blue (#2196F3)
  - Text: "BLUE - No Person (Idle)"
  
- 🟢 **GREEN**: Unknown person detected
  - Background: Solid green (#4CAF50)
  - Text: "GREEN - Unknown Person"

**Additional Info Displayed:**
- Person identity (target_person, unknown, no_person)
- Confidence score (0-100%)
- Duration in current state (seconds)
- Face count

### 5.3 Service Control

**Implementation:**
- Uses `systemctl` commands via subprocess
- Requires sudo permissions (configured via sudoers)
- Real-time status polling (every 5 seconds)
- Immediate feedback on actions

**Service Status Indicators:**
- Green border = Active
- Red border = Inactive
- Status text shows "● Running" or "○ Stopped"

**Error Handling:**
- Failed operations show error messages
- Service status refreshes after actions
- Logs errors to browser console

### 5.4 Volume Control

**Implementation:**
- Uses ROS 2 parameter API (`ros2 param set`)
- Updates parameter in real-time (no service restart)
- Validates volume range (0.0-1.0)
- Immediate feedback

**UI Features:**
- Slider with visual feedback
- Large volume display
- Quick preset buttons
- Real-time updates

**Volume Levels:**
- 5% - Very quiet (default)
- 10% - Quiet
- 20% - Moderate
- 50% - Loud
- 100% - Maximum

### 5.5 Training Integration

**Training Options:**

1. **Train New Person** - Complete workflow:
   - Captures training images (interactive 4-task system)
   - Trains LBPH model
   - Tests model accuracy
   - Shows progress and logs in real-time

2. **Add More Pictures** - Extends existing dataset:
   - Captures additional images
   - Adds to existing person directory
   - Can retrain after adding

3. **Retrain Model** - Rebuilds from existing images:
   - Uses all images in person directory
   - Trains new model
   - Replaces existing model file

4. **Test Accuracy** - Placeholder (not yet implemented)

5. **Real-time Test** - Placeholder (not yet implemented)

6. **List People/Models** - Shows all trained people:
   - Model file sizes
   - Image counts
   - Directory paths

7. **Delete Person** - Removes all data:
   - Deletes model file
   - Deletes image directory
   - Confirmation dialog for safety

**Training Task Management:**
- Background task execution
- Real-time log streaming
- Progress tracking
- Status updates (running/completed/failed)

### 5.6 Real-time Stream

**Event Types:**
- Recognition events (person recognized)
- Status changes (RED → BLUE, etc.)
- Service actions (started/stopped)
- Training events (started/completed)
- System notifications

**Stream Features:**
- Timestamped messages
- Color-coded by type
- Auto-scroll to latest
- Keeps last 50 messages
- Monospace font for readability

---

## 6. Troubleshooting

### 6.1 Dashboard Not Loading

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
   # On Jetson
   tailscale status
   
   # On Windows
   ping 100.95.133.26
   ```

4. Check firewall:
   ```bash
   sudo ufw allow 8080/tcp
   ```

### 6.2 No Real-time Updates

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

5. Test rosbridge connection:
   ```bash
   # Should show WebSocket connection
   ros2 topic echo /r2d2/audio/person_status
   ```

### 6.3 Service Control Not Working

**Symptoms:** Buttons don't work or show errors

**Solutions:**
1. Check sudo permissions:
   ```bash
   sudo systemctl status r2d2-audio-notification.service
   # Should work without password
   ```

2. Verify sudoers configuration:
   ```bash
   sudo visudo -f /etc/sudoers.d/r2d2-services
   ```

3. Check service names in config:
   ```bash
   # Verify services exist
   systemctl list-units | grep r2d2
   ```

4. Check API response:
   ```bash
   curl -X POST http://localhost:8080/api/services/audio/start
   ```

### 6.4 Volume Control Not Working

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

4. Check API response:
   ```bash
   curl http://localhost:8080/api/audio/volume
   ```

### 6.5 Training Not Working

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
   python3 1_capture_training_data.py test_person
   ```

4. Check training logs in dashboard (training status panel)

5. Verify directory permissions:
   ```bash
   ls -la ~/dev/r2d2/data/face_recognition/
   ```

### 6.6 Connection Issues

**Symptoms:** Can't connect via Tailscale IP

**Solutions:**
1. Verify Tailscale IP:
   ```bash
   # On Jetson
   tailscale ip -4
   ```

2. Update dashboard JavaScript if IP changed:
   - Edit `app/static/js/dashboard.js`
   - Update `ROSBRIDGE_URL` constant

3. Test connection:
   ```bash
   # From Windows
   ping 100.95.133.26
   curl http://100.95.133.26:8080
   ```

4. Check if server binds to correct interface:
   - Verify `HOST=0.0.0.0` in config (allows external connections)

### 6.7 Services Not Starting After Reboot

**Symptoms:** After reboot, Recognition Status is not active or camera stream starts instead

**Solutions:**
1. Check which services are enabled:
   ```bash
   systemctl is-enabled r2d2-camera-stream.service      # Should be: disabled
   systemctl is-enabled r2d2-camera-perception.service  # Should be: enabled
   systemctl is-enabled r2d2-audio-notification.service # Should be: enabled
   ```

2. Fix default boot configuration:
   ```bash
   # Use the provided script
   ~/dev/r2d2/change_default_boot_services.sh
   
   # Or manually:
   sudo systemctl disable r2d2-camera-stream.service
   sudo systemctl enable r2d2-camera-perception.service
   sudo systemctl enable r2d2-audio-notification.service
   ```

3. Verify services are running after reboot:
   ```bash
   systemctl status r2d2-camera-perception.service
   systemctl status r2d2-audio-notification.service
   ```

---

## 7. Security Considerations

### 7.1 Access Control

**Current Implementation:**
- ✅ VPN-only access (Tailscale provides encryption)
- ✅ No public Internet exposure
- ⚠️ No authentication (relies on VPN security)

**Recommendations:**
- Consider adding HTTP basic authentication for additional security
- Or implement session-based authentication
- For production, use HTTPS (Let's Encrypt certificate)

### 7.2 Service Control Security

**Sudo Configuration:**
- Limited to specific systemctl commands
- Only for r2d2-* services
- No password required (for convenience)

**Alternative Approaches:**
- Use systemd user services (no sudo needed)
- Create wrapper script with setuid
- Use polkit for fine-grained permissions

### 7.3 Input Validation

**Implemented:**
- ✅ Volume range validation (0.0-1.0)
- ✅ Person name validation (alphanumeric + underscore)
- ✅ Service name validation (whitelist)

**Additional Recommendations:**
- Sanitize all user inputs
- Validate file paths
- Prevent path traversal attacks
- Rate limit API endpoints

### 7.4 Network Security

**Current Setup:**
- Binds to 0.0.0.0 (all interfaces)
- Accessible only via Tailscale VPN
- No firewall rules needed (Tailscale handles it)

**For Additional Security:**
- Bind to Tailscale interface only (if possible)
- Add firewall rules (allow only Tailscale IPs)
- Use reverse proxy (nginx) with authentication

---

## 8. File Locations

| Component | Location |
|-----------|----------|
| **Web Dashboard** | `~/dev/r2d2/web_dashboard/` |
| **FastAPI App** | `~/dev/r2d2/web_dashboard/app/` |
| **HTML Dashboard** | `~/dev/r2d2/web_dashboard/app/templates/index.html` |
| **Static Files** | `~/dev/r2d2/web_dashboard/app/static/` |
| **Launch Files** | `~/dev/r2d2/web_dashboard/launch/` |
| **Scripts** | `~/dev/r2d2/web_dashboard/scripts/` |
| **Virtual Environment** | `~/dev/r2d2/web_dashboard/web_dashboard_env/` |

---

## 9. Quick Reference

### 9.1 Start Dashboard

```bash
# Terminal 1: rosbridge
cd ~/dev/r2d2/web_dashboard
source /opt/ros/humble/setup.bash
ros2 launch web_dashboard/launch/rosbridge.launch.py

# Terminal 2: Web server
cd ~/dev/r2d2/web_dashboard
source web_dashboard_env/bin/activate
./start_server.sh
```

### 9.2 Access Dashboard

```
http://100.95.133.26:8080
```

### 9.3 Common Tasks

**Check Service Status:**
- View Service Control panel in dashboard
- Or: `curl http://100.95.133.26:8080/api/services/status`

**Set Volume:**
- Use volume slider in dashboard
- Or: `curl -X POST http://100.95.133.26:8080/api/audio/volume -H "Content-Type: application/json" -d '{"volume":0.3}'`

**Start Training:**
- Use Training panel in dashboard
- Or: `curl -X POST http://100.95.133.26:8080/api/training/capture -H "Content-Type: application/json" -d '{"person_name":"alice"}'`

### 9.4 Monitoring Commands

```bash
# Check if rosbridge is running
ros2 node list | grep rosbridge

# Check if web server is running
ps aux | grep "app.main"

# View web server logs
# (if running in terminal, logs appear there)

# Test API
curl http://localhost:8080/api/services/status
```

---

## 10. Related Documentation

- **Architecture Overview:** [`001_ARCHITECTURE_OVERVIEW.md`](001_ARCHITECTURE_OVERVIEW.md)
- **VPN Setup:** [`012_VPN_SETUP_AND_REMOTE_ACCESS.md`](012_VPN_SETUP_AND_REMOTE_ACCESS.md)
- **Person Recognition:** [`100_PERSON_RECOGNITION_AND_STATUS.md`](100_PERSON_RECOGNITION_AND_STATUS.md)
- **Monitoring Tools:** [`100_PERSON_RECOGNITION_AND_STATUS.md`](100_PERSON_RECOGNITION_AND_STATUS.md) (Section 9)

---

**Created:** December 12, 2025  
**Last Updated:** December 12, 2025  
**Status:** ✅ Production Ready  
**Version:** 1.0


