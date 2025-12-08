# Audio System - Quick Reference Guide

**Quick Links:** [`050_AUDIO_SETUP_AND_CONFIGURATION.md`](050_AUDIO_SETUP_AND_CONFIGURATION.md) | [`060_AUDIO_NOTIFICATIONS_ROS2_INTEGRATION.md`](060_AUDIO_NOTIFICATIONS_ROS2_INTEGRATION.md)

---

## 🚀 Quick Start

### Test Audio (Is it working?)

```bash
python3 ~/dev/r2d2/audio_beep.py
```

→ You should hear a beep. If not, check [`050_AUDIO_SETUP_AND_CONFIGURATION.md`](050_AUDIO_SETUP_AND_CONFIGURATION.md) section "Troubleshooting".

### Launch Notifications (Development)

```bash
# Terminal 1
cd ~/dev/r2d2/ros2_ws && source install/setup.bash
ros2 launch r2d2_audio audio_notification.launch.py

# Terminal 2
ros2 launch r2d2_bringup r2d2_camera_perception.launch.py enable_face_recognition:=true
```

### Run as Background Service (Production)

```bash
# One-time setup
sudo cp /home/severin/dev/r2d2/r2d2-audio-notification.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable r2d2-audio-notification.service
sudo systemctl start r2d2-audio-notification.service

# Check status anytime
sudo systemctl status r2d2-audio-notification.service
sudo journalctl -u r2d2-audio-notification.service -f
```

---

## 🔊 What You'll Hear

| Event | Sound | Meaning |
|-------|-------|---------|
| **Face detected** | 🔊 Single beep (1000 Hz) | You're recognized! |
| **Face lost** | 🔔🔔 Double beep (500 Hz) | You've been away > 5s |
| **Face returns** | 🔊 Single beep (1000 Hz) | Welcome back! |

---

## ⚙️ Customize Beeps

```bash
# Louder
ros2 launch r2d2_audio audio_notification.launch.py beep_volume:=0.9

# Higher pitch
ros2 launch r2d2_audio audio_notification.launch.py beep_frequency:=1500

# Lower pitch loss alert
ros2 launch r2d2_audio audio_notification.launch.py loss_beep_frequency:=300

# Faster loss detection (3s instead of 5s)
ros2 launch r2d2_audio audio_notification.launch.py \
  jitter_tolerance_seconds:=3.0 \
  loss_confirmation_seconds:=3.0

# Patient system (accepts 10s gaps)
ros2 launch r2d2_audio audio_notification.launch.py \
  jitter_tolerance_seconds:=10.0 \
  loss_confirmation_seconds:=10.0
```

---

## 📊 Service Management

```bash
# Start
sudo systemctl start r2d2-audio-notification.service

# Stop
sudo systemctl stop r2d2-audio-notification.service

# Restart
sudo systemctl restart r2d2-audio-notification.service

# Status
sudo systemctl status r2d2-audio-notification.service

# View logs
sudo journalctl -u r2d2-audio-notification.service -f

# Last 50 lines
sudo journalctl -u r2d2-audio-notification.service -n 50

# Disable auto-start
sudo systemctl disable r2d2-audio-notification.service

# Enable auto-start
sudo systemctl enable r2d2-audio-notification.service
```

---

## 🔍 Monitor & Debug

```bash
# Check events
ros2 topic echo /r2d2/audio/notification_event

# Monitor face recognition
ros2 topic echo /r2d2/perception/person_id

# Test audio directly
python3 ~/dev/r2d2/audio_beep.py --frequency 500 --duration 1.0 --volume 0.8

# View service logs
sudo journalctl -u r2d2-audio-notification.service -n 50
```

---

## 📋 Parameter Reference

| Parameter | Default | Description |
|-----------|---------|-------------|
| `target_person` | `severin` | Who to recognize |
| `beep_frequency` | `1000.0` Hz | Recognition beep pitch |
| `beep_duration` | `0.5` sec | Recognition beep length |
| `beep_volume` | `0.7` | Beep loudness (0.0-1.0) |
| `loss_beep_frequency` | `500.0` Hz | Loss alert pitch |
| `loss_beep_duration` | `0.3` sec | Loss alert length |
| `jitter_tolerance_seconds` | `5.0` sec | Ignore gaps < this |
| `loss_confirmation_seconds` | `5.0` sec | Confirm loss after this |
| `cooldown_seconds` | `2.0` sec | Min time between beeps |
| `enabled` | `true` | Enable/disable notifications |

---

## 🧪 Test Scripts

```bash
# Simple test (2 beeps)
python3 ~/dev/r2d2/simple_face_beep_test.py

# Enhanced test (state machine test)
python3 ~/dev/r2d2/enhanced_face_beep_test.py
```

---

## 🆘 Troubleshooting

**No beep at all?**
→ See: [`050_AUDIO_SETUP_AND_CONFIGURATION.md`](050_AUDIO_SETUP_AND_CONFIGURATION.md) - "Troubleshooting" section

**No notifications from ROS 2?**
→ Check: Face recognition is running: `ros2 topic echo /r2d2/perception/person_id`

**Service won't start?**
→ View logs: `sudo journalctl -u r2d2-audio-notification.service -n 20`

**Beep too quiet/loud?**
→ Adjust: `beep_volume` parameter (0.0 = silent, 1.0 = maximum)

**Too many/few beeps?**
→ Adjust: `cooldown_seconds`, `jitter_tolerance_seconds`, `loss_confirmation_seconds`

---

## 📁 File Locations

- **Hardware Setup**: [`050_AUDIO_SETUP_AND_CONFIGURATION.md`](050_AUDIO_SETUP_AND_CONFIGURATION.md)
- **ROS 2 Integration**: [`060_AUDIO_NOTIFICATIONS_ROS2_INTEGRATION.md`](060_AUDIO_NOTIFICATIONS_ROS2_INTEGRATION.md)
- **Audio Utility**: `~/dev/r2d2/audio_beep.py`
- **Service File**: `/etc/systemd/system/r2d2-audio-notification.service`
- **Startup Script**: `~/dev/r2d2/start_audio_service.sh`
- **ROS 2 Package**: `~/dev/r2d2/ros2_ws/src/r2d2_audio/`

---

## 🎯 Common Workflows

### Setup for First Time

1. Test audio: `python3 ~/dev/r2d2/audio_beep.py`
2. If no beep → Follow [`050_AUDIO_SETUP_AND_CONFIGURATION.md`](050_AUDIO_SETUP_AND_CONFIGURATION.md)
3. If beep works → Launch notifications: `ros2 launch r2d2_audio audio_notification.launch.py`
4. Move in/out of camera frame, listen for beeps

### Install as Background Service

```bash
sudo cp /home/severin/dev/r2d2/r2d2-audio-notification.service /etc/systemd/system/
sudo systemctl daemon-reload && sudo systemctl enable r2d2-audio-notification.service
sudo systemctl start r2d2-audio-notification.service && sudo systemctl status r2d2-audio-notification.service
```

### Monitor Service

```bash
sudo journalctl -u r2d2-audio-notification.service -f
```

### Customize Beeps

```bash
# Edit service file
sudo nano /etc/systemd/system/r2d2-audio-notification.service

# Add to ExecStart line:
# beep_frequency:=1500 beep_volume:=0.9

# Save, reload, restart
sudo systemctl daemon-reload && sudo systemctl restart r2d2-audio-notification.service
```

---

## 📖 Full Documentation

For detailed information:
- **Hardware & ALSA setup**: [`050_AUDIO_SETUP_AND_CONFIGURATION.md`](050_AUDIO_SETUP_AND_CONFIGURATION.md)
- **ROS 2 integration & service setup**: [`060_AUDIO_NOTIFICATIONS_ROS2_INTEGRATION.md`](060_AUDIO_NOTIFICATIONS_ROS2_INTEGRATION.md)

---

**Last Updated:** December 8, 2025  
**Quick Reference Version:** 1.0
