# R2D2 Web UI - Quick Start Guide
## Essential Commands and Daily Use Reference

**Date:** December 17, 2025  
**Access:** http://100.95.133.26:8079 (Service Mode)

---

## Quick Access

### 1. Service Mode (Wake API)
**URL:** `http://100.95.133.26:8079`
- Check if R2D2 is alive (Heartbeat)
- Start the full Web UI
- **Always on** (minimal resource usage)

### 2. Full Web Dashboard
**URL:** `http://100.95.133.26:8080`
- Full control interface
- **Must be started first** from Service Mode

### 3. Camera Stream
**URL:** `http://100.95.133.26:8081/stream`

---

## Daily Usage Pattern

### 1. Check Status
Open `http://100.95.133.26:8079`.
- Look at the "Heartbeat" section.
- If it says "Online", R2D2 core systems are running.

### 2. Start Dashboard
- Click **"Start Web UI"**.
- Wait ~5 seconds.
- You will be redirected to the full dashboard.

### 3. Use System
- Perform your tasks (monitoring, training, etc.).

### 4. Finish & Save Resources
- Click **"Exit Service Mode"** in the top-right corner.
- The heavy Web UI services will stop.
- You'll be returned to the minimal Service Mode page.
- This saves ~300MB RAM and ~10% CPU.

---

## Essential Commands

### Service Control

**Start a service:**
```bash
# From web dashboard UI: Click "Start" button
# Or via command line:
sudo systemctl start r2d2-audio-notification.service
sudo systemctl start r2d2-camera-perception.service
```

**Stop a service:**
```bash
# From web dashboard UI: Click "Stop" button
# Or via command line:
sudo systemctl stop r2d2-audio-notification.service
```

**Restart a service:**
```bash
# From web dashboard UI: Click "Restart" button
# Or via command line:
sudo systemctl restart r2d2-audio-notification.service
```

**Check service status:**
```bash
sudo systemctl status r2d2-audio-notification.service
sudo systemctl status r2d2-camera-perception.service
sudo systemctl status r2d2-camera-stream.service
sudo systemctl status r2d2-heartbeat.service
```

### Volume Control

**Set volume from web dashboard:**
- Use the volume slider (0-100%)
- Or click preset buttons: 5%, 10%, 20%, 50%, 100%

**Set volume from command line:**
```bash
# Set volume (0.0 = silent, 1.0 = max)
ros2 param set /audio_notification_node audio_volume 0.05

# Get current volume
ros2 param get /audio_notification_node audio_volume
```

### Face Recognition Training

**From Web Dashboard:**
1. Enter person name in the input field
2. Click the desired training option:
   - `[1] Train New Person` - Full training workflow
   - `[2] Add More Pictures` - Add to existing dataset
   - `[3] Retrain Model` - Rebuild from existing images
   - `[6] List People/Models` - Show all trained people
   - `[7] Delete Person` - Remove person data

**From Command Line:**
```bash
# Activate training environment
cd ~/dev/r2d2/tests/face_recognition
source ~/depthai_env/bin/activate
export OPENBLAS_CORETYPE=ARMV8

# Train new person (interactive)
python3 train_manager.py

# Or use individual scripts
python3 1_capture_training_data.py <person_name>
python3 2_train_recognizer.py <person_name>
python3 3_test_recognizer_demo.py <person_name>
```

---

## Monitoring Commands

### Check ROS 2 Topics

```bash
# List all topics
ros2 topic list

# Monitor person status (recognition state)
ros2 topic echo /r2d2/audio/person_status

# Monitor person ID
ros2 topic echo /r2d2/perception/person_id

# Check topic publishing rate
ros2 topic hz /r2d2/audio/person_status
ros2 topic hz /r2d2/perception/person_id
```

### Check System Health

```bash
# Check heartbeat (lightweight alive status)
ros2 topic echo /r2d2/heartbeat

# Get full system metrics via REST API
curl http://100.95.133.26:8080/api/system/health

# Check all running ROS 2 nodes
ros2 node list

# Check if rosbridge is running
ros2 node list | grep rosbridge
```

### Check Network Status

```bash
# Check Tailscale VPN
tailscale status
tailscale ip -4

# Check if web dashboard is accessible
curl http://localhost:8080

# Check if rosbridge is running
netstat -tuln | grep 9090

# Check if camera stream is running
curl -I http://localhost:8081/stream
```

---

## Common Tasks

### Task: Enable Face Recognition

```bash
# Make sure camera perception service is running with face recognition enabled
sudo systemctl restart r2d2-camera-perception.service

# Verify face recognition is active
ros2 topic echo /r2d2/perception/person_id
```

### Task: Change Audio Volume Quickly

**From Web Dashboard:**
- Click preset button (e.g., "20%" for normal volume)

**From Command Line:**
```bash
ros2 param set /audio_notification_node audio_volume 0.2
```

### Task: View Live Camera Feed

**Method 1: Via Web Dashboard**
1. Click "Start Stream" in Camera Stream panel
2. Stream opens in new tab/window at port 8081

**Method 2: Direct URL**
```
http://100.95.133.26:8081/stream
```

**Method 3: Command Line (verify)**
```bash
# Check if camera is publishing
ros2 topic hz /oak/rgb/image_raw

# Start camera stream service
sudo systemctl start r2d2-camera-stream.service
```

### Task: Train New Person

**Quick Method (Web Dashboard):**
1. Enter person name (e.g., "alice")
2. Click "[1] Train New Person"
3. Wait for capture tasks to complete
4. Training automatically proceeds
5. Check "Training Status" panel for progress

**Manual Method (Command Line):**
```bash
cd ~/dev/r2d2/tests/face_recognition
source ~/depthai_env/bin/activate
export OPENBLAS_CORETYPE=ARMV8

python3 train_manager.py
# Follow interactive prompts
```

### Task: Retrain Existing Person

**From Web Dashboard:**
1. Enter existing person name
2. Click "[3] Retrain Model"

**From Command Line:**
```bash
cd ~/dev/r2d2/tests/face_recognition
source ~/depthai_env/bin/activate
export OPENBLAS_CORETYPE=ARMV8

python3 2_train_recognizer.py <person_name>
```

### Task: List All Trained People

**From Web Dashboard:**
- Click "[6] List People/Models"
- View in "Training Status" panel

**From Command Line:**
```bash
# List model files
ls -lh ~/dev/r2d2/data/face_recognition/models/

# List datasets
ls -lh ~/dev/r2d2/data/face_recognition/datasets/
```

### Task: Delete Person Data

**From Web Dashboard:**
1. Click "[7] Delete Person"
2. Enter person name in confirmation dialog
3. Confirm deletion

**From Command Line:**
```bash
# Delete model
rm ~/dev/r2d2/data/face_recognition/models/<person_name>_lbph.xml

# Delete dataset
rm -rf ~/dev/r2d2/data/face_recognition/datasets/<person_name>/
```

---

## Troubleshooting Quick Fixes

### Dashboard Not Loading

```bash
# Check if web server is running
ps aux | grep "app.main"

# Check if port is accessible
curl http://localhost:8080

# Restart web server
cd ~/dev/r2d2/web_dashboard
source web_dashboard_env/bin/activate
./start_server.sh
```

### No Real-time Updates

```bash
# Check if rosbridge is running
ros2 node list | grep rosbridge

# Check if topics are publishing
ros2 topic hz /r2d2/audio/person_status

# Restart rosbridge
pkill -f rosbridge_websocket
cd ~/dev/r2d2/web_dashboard
./start_rosbridge.sh
```

### Service Control Not Working

```bash
# Verify sudo permissions
sudo systemctl status r2d2-audio-notification.service

# Check sudoers file
sudo cat /etc/sudoers.d/r2d2-services

# Expected content:
# severin ALL=(ALL) NOPASSWD: /bin/systemctl start r2d2-*, /bin/systemctl stop r2d2-*, /bin/systemctl restart r2d2-*
```

### Volume Control Not Working

```bash
# Check if audio node is running
ros2 node list | grep audio_notification_node

# Check current volume
ros2 param get /audio_notification_node audio_volume

# Set volume manually
ros2 param set /audio_notification_node audio_volume 0.1
```

### Camera Stream Not Working

```bash
# Check if camera is publishing
ros2 topic hz /oak/rgb/image_raw

# Check if camera stream service is running
sudo systemctl status r2d2-camera-stream.service

# Restart camera services
sudo systemctl restart r2d2-camera-perception.service
sudo systemctl restart r2d2-camera-stream.service
```

### Training Fails

```bash
# Check environment variable
echo $OPENBLAS_CORETYPE  # Should output: ARMV8

# Set if not set
export OPENBLAS_CORETYPE=ARMV8

# Check if training scripts exist
ls ~/dev/r2d2/tests/face_recognition/*.py

# Check directory permissions
ls -la ~/dev/r2d2/data/face_recognition/
```

---

## Key ROS 2 Topics Reference

| Topic | Type | Purpose | Rate |
|-------|------|---------|------|
| `/r2d2/audio/person_status` | String (JSON) | Recognition state | 10 Hz |
| `/r2d2/perception/person_id` | String | Person identity | 6.5 Hz |
| `/r2d2/perception/face_count` | Int32 | Face count | 13 Hz |
| `/r2d2/perception/face_confidence` | Float32 | Confidence score | 6.5 Hz |
| `/r2d2/heartbeat` | String (JSON) | Alive status | 1 Hz |
| `/oak/rgb/image_raw` | sensor_msgs/Image | Camera feed | 30 Hz |

---

## Key File Locations

| Component | Location |
|-----------|----------|
| **Web Dashboard** | `~/dev/r2d2/web_dashboard/` |
| **Training Scripts** | `~/dev/r2d2/tests/face_recognition/` |
| **Face Recognition Data** | `~/dev/r2d2/data/face_recognition/` |
| **Models** | `~/dev/r2d2/data/face_recognition/models/` |
| **Training Datasets** | `~/dev/r2d2/data/face_recognition/datasets/` |
| **Virtual Environment** | `~/dev/r2d2/web_dashboard/web_dashboard_env/` |

---

## Service Names Reference

| Short Name | Full systemd Name | Purpose |
|------------|------------------|---------|
| `audio` | r2d2-audio-notification.service | Audio alerts |
| `camera` | r2d2-camera-perception.service | Camera + perception |
| `powerbutton` | r2d2-powerbutton.service | Shutdown control |
| `heartbeat` | r2d2-heartbeat.service | Alive ping |
| `camera-stream` | r2d2-camera-stream.service | MJPEG stream |
| `rosbridge` | r2d2-rosbridge.service | WebSocket bridge |
| `web-dashboard` | r2d2-web-dashboard.service | Web UI server |

---

## Daily Usage Pattern

### Morning Startup

```bash
# 1. SSH into Jetson (via Tailscale)
ssh severin@100.95.133.26

# 2. Start web dashboard
cd ~/dev/r2d2/web_dashboard
./scripts/start_web_dashboard.sh

# 3. Open browser on laptop
# Navigate to: http://100.95.133.26:8080

# 4. Start required services from dashboard
# Click "Start" for: audio, camera
```

### During Operation

- Monitor recognition status in real-time
- Adjust volume as needed using slider
- Check system health metrics via REST API (CPU, GPU, Disk, Temp)
- View event stream for activity logs

### Evening Shutdown

```bash
# 1. Stop services from dashboard (optional)
# Click "Stop" for: audio, camera

# 2. Stop web dashboard
# Press Ctrl+C in terminals running rosbridge and web server

# Or if using systemd:
sudo systemctl stop r2d2-web-dashboard.service
sudo systemctl stop r2d2-rosbridge.service
```

---

## API Quick Reference

### Service Control

```bash
# Start service
curl -X POST http://100.95.133.26:8080/api/services/audio/start

# Stop service
curl -X POST http://100.95.133.26:8080/api/services/audio/stop

# Get all service status
curl http://100.95.133.26:8080/api/services/status
```

### Volume Control

```bash
# Get volume
curl http://100.95.133.26:8080/api/audio/volume

# Set volume (0.0-1.0)
curl -X POST http://100.95.133.26:8080/api/audio/volume \
  -H "Content-Type: application/json" \
  -d '{"volume": 0.3}'
```

### Training

```bash
# List trained people
curl http://100.95.133.26:8080/api/training/list

# Delete person
curl -X DELETE http://100.95.133.26:8080/api/training/alice
```

---

## Documentation References

For more detailed information:

- **Installation Guide:** [`111_WEB_UI_INSTALLATION.md`](111_WEB_UI_INSTALLATION.md)
- **Reference Documentation:** [`110_WEB_UI_REFERENCE.md`](110_WEB_UI_REFERENCE.md)
- **Architecture Overview:** [`001_ARCHITECTURE_OVERVIEW.md`](001_ARCHITECTURE_OVERVIEW.md)
- **VPN Setup:** [`012_VPN_SETUP_AND_REMOTE_ACCESS.md`](012_VPN_SETUP_AND_REMOTE_ACCESS.md)
- **Person Recognition:** [`100_PERSON_RECOGNITION_REFERENCE.md`](100_PERSON_RECOGNITION_REFERENCE.md)

---

**Document Version:** 1.0  
**Last Updated:** December 17, 2025  
**Platform:** NVIDIA Jetson AGX Orin 64GB with ROS 2 Humble


