# R2D2 Audio Notification - Quick Start Guide

## 🚀 Quick Launch Options

### Option A: Manual Launch (Testing/Development)

```bash
# Terminal 1: Audio notifications
cd ~/dev/r2d2/ros2_ws
source install/setup.bash
ros2 launch r2d2_audio audio_notification.launch.py

# Terminal 2: Face recognition pipeline
cd ~/dev/r2d2/ros2_ws
source install/setup.bash
ros2 launch r2d2_bringup r2d2_camera_perception.launch.py enable_face_recognition:=true
```

### Option B: Background Service (Production)

```bash
# Install as SystemD service (one-time setup)
sudo cp /home/severin/dev/r2d2/r2d2-audio-notification.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable r2d2-audio-notification.service
sudo systemctl start r2d2-audio-notification.service

# Check status anytime
sudo systemctl status r2d2-audio-notification.service

# View logs
sudo journalctl -u r2d2-audio-notification.service -f
```

---

## 🔊 What You'll Hear

| Event | Sound | Meaning |
|-------|-------|---------|
| **Face detected** | 🔊 Single beep (1000 Hz) | R2D2 recognizes you! |
| **Face lost** | 🔔🔔 Double beep (500 Hz) | You've been away > 5 seconds |
| **Face re-appears** | 🔊 Single beep (1000 Hz) | Welcome back! |

---

## 🎛️ Customize Beeps

```bash
# Louder beeps
ros2 launch r2d2_audio audio_notification.launch.py beep_volume:=0.9

# Higher pitch recognition
ros2 launch r2d2_audio audio_notification.launch.py beep_frequency:=1500

# Lower pitch loss alert
ros2 launch r2d2_audio audio_notification.launch.py loss_beep_frequency:=300

# Faster loss detection (3 seconds instead of 5)
ros2 launch r2d2_audio audio_notification.launch.py \
  jitter_tolerance_seconds:=3.0 \
  loss_confirmation_seconds:=3.0

# Patient system (tolerates 10 second gaps)
ros2 launch r2d2_audio audio_notification.launch.py \
  jitter_tolerance_seconds:=10.0 \
  loss_confirmation_seconds:=10.0
```

---

## 🧪 Test It

```bash
# Simple test (2 beeps)
python3 ~/dev/r2d2/simple_face_beep_test.py

# Enhanced test (4 beeps with loss detection)
python3 ~/dev/r2d2/enhanced_face_beep_test.py
```

---

## ⚙️ Service Commands

```bash
# Start
sudo systemctl start r2d2-audio-notification.service

# Stop
sudo systemctl stop r2d2-audio-notification.service

# Restart
sudo systemctl restart r2d2-audio-notification.service

# Status
sudo systemctl status r2d2-audio-notification.service

# Live logs
sudo journalctl -u r2d2-audio-notification.service -f

# Disable auto-start
sudo systemctl disable r2d2-audio-notification.service

# Enable auto-start
sudo systemctl enable r2d2-audio-notification.service
```

---

## 🔍 Monitor Events

```bash
# Watch notifications in real-time
ros2 topic echo /r2d2/audio/notification_event
```

---

## ⚡ Key Features

✅ **Jitter Tolerance**: Brief camera gaps (< 5s) don't trigger loss alerts  
✅ **Confirmed Loss**: Must be gone continuously for > 5s to trigger double-beep  
✅ **Smart State**: Status persists through momentary interruptions  
✅ **Background Service**: Auto-start, auto-restart, proper logging  
✅ **Fully Configurable**: All parameters adjustable  
✅ **Lightweight**: ~2-4% CPU, ~50 MB memory  

---

## 📋 Default Configuration

- **Recognition beep**: 1000 Hz, 0.5 sec, 70% volume
- **Loss alert beep**: 500 Hz, 0.3 sec × 2, 70% volume  
- **Jitter tolerance**: 5 seconds
- **Loss confirmation**: 5 seconds
- **Recognition cooldown**: 2 seconds

---

## 📖 Full Documentation

For detailed information, setup troubleshooting, and architecture details, see:
```
AUDIO_NOTIFICATION_BACKGROUND_SERVICE.md
```

Enjoy your R2D2! 🤖 🔊

