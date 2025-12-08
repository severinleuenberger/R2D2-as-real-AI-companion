# R2D2 Audio Notification System - SETUP COMPLETE ✅

**Date:** December 8, 2025  
**Version:** 2.0 - Production Verified  
**Status:** ✅ **FULLY OPERATIONAL AND TESTED**

---

## 🎉 What's Working

✅ **Audio notifications** - Deep 400Hz beeps at 25% volume  
✅ **Recognition detection** - Beeps when face recognized  
✅ **Loss detection** - Double beep when you leave (5+ sec)  
✅ **Jitter tolerance** - Ignores brief camera gaps  
✅ **SystemD service** - Auto-starts on Jetson boot  
✅ **Background operation** - Service running right now!

---

## 🚀 Current Status

### Service is RUNNING ✅

**Check status anytime:**
```bash
sudo systemctl status r2d2-audio-notification.service
```

**View live logs:**
```bash
sudo journalctl -u r2d2-audio-notification.service -f
```

---

## 🎵 Quick Reference

### You'll Hear

| Event | Sound |
|-------|-------|
| **Face Recognized** | 🔊 Single beep (deep, 400 Hz) |
| **5+ sec without you** | 🔔🔔 Double beep (same tone) |
| **You Return** | 🔊 Single beep |

**Volume:** Subtle (25%), won't startle you

---

## 🛠️ Service Management

### Start the service
```bash
sudo systemctl start r2d2-audio-notification.service
```

### Stop the service
```bash
sudo systemctl stop r2d2-audio-notification.service
```

### Restart the service
```bash
sudo systemctl restart r2d2-audio-notification.service
```

### View service logs (last 50 lines)
```bash
sudo journalctl -u r2d2-audio-notification.service -n 50
```

### View live logs (follow mode)
```bash
sudo journalctl -u r2d2-audio-notification.service -f
```

### Disable auto-start (but keep installed)
```bash
sudo systemctl disable r2d2-audio-notification.service
```

### Check if service will auto-start on boot
```bash
sudo systemctl is-enabled r2d2-audio-notification.service
# Output: enabled ✅
```

---

## 🎚️ Customize Audio Settings

### Make beeps louder
Edit `/etc/systemd/system/r2d2-audio-notification.service` and modify the ExecStart line:

**Before:**
```
ExecStart=/home/severin/dev/r2d2/start_audio_notification.sh
```

**After (for 50% volume):**
```
ExecStart=/home/severin/dev/r2d2/start_audio_notification.sh --ros-args -p beep_volume:=0.5
```

Then reload:
```bash
sudo systemctl daemon-reload
sudo systemctl restart r2d2-audio-notification.service
```

### Change beep frequency
```bash
# Make beeps higher pitched (600 Hz)
sudo systemctl stop r2d2-audio-notification.service
# Edit service file (ExecStart line)
# Add: -p beep_frequency:=600
sudo systemctl daemon-reload
sudo systemctl start r2d2-audio-notification.service
```

### Adjust loss detection timing
By default:
- Tolerates 5 second gaps (jitter tolerance)
- Confirms loss after 5 seconds continuous absence

To make it faster (2 second detection):
```bash
# Edit service file ExecStart line to add:
-p jitter_tolerance_seconds:=2 -p loss_confirmation_seconds:=2
```

---

## 📡 Monitor in Real-Time

### Watch events being published
```bash
ros2 topic echo /r2d2/audio/notification_event
```

Expected output:
```
data: 🎉 Recognized severin!
---
data: ❌ severin lost (confirmed)
---
data: 🎉 Recognized severin!
```

### Check face recognition input
```bash
ros2 topic echo /r2d2/perception/person_id
```

Expected output:
```
data: severin
---
data: unknown
---
data: severin
```

---

## 🧪 Testing

### Test audio directly
```bash
python3 ~/dev/r2d2/audio_beep.py
# You should hear a beep
```

### Test audio at different volumes
```bash
python3 ~/dev/r2d2/audio_beep.py --volume 0.5
# Louder beep
```

### Test complete behavior (manual)
```bash
# Terminal 1: Start service
sudo systemctl start r2d2-audio-notification.service

# Terminal 2: Monitor events
ros2 topic echo /r2d2/audio/notification_event

# Terminal 3: Simulate face recognition
ros2 topic pub -1 /r2d2/perception/person_id std_msgs/msg/String '{data: severin}'
# Should hear beep and see event
```

---

## 🔧 Troubleshooting

### Service won't start

1. **Check status:**
   ```bash
   sudo systemctl status r2d2-audio-notification.service
   ```

2. **Check detailed logs:**
   ```bash
   sudo journalctl -u r2d2-audio-notification.service -n 30
   ```

3. **Verify startup script is executable:**
   ```bash
   ls -la /home/severin/dev/r2d2/start_audio_notification.sh
   # Should show: -rwxr-xr-x
   ```

4. **Test manual launch:**
   ```bash
   cd ~/dev/r2d2/ros2_ws
   source install/setup.bash
   audio_notification_node
   ```

### No beeps heard

1. **Check audio output:**
   ```bash
   python3 ~/dev/r2d2/audio_beep.py
   # Should hear a beep
   ```

2. **Check J511 Pin 5 audio wiring:**
   - Should have wire from J511 Pin 5 (HPO_R RIGHT channel)
   - Wire should go to PAM8403 RIGHT input
   - Speaker should be connected to PAM8403 output

3. **Check face recognition is publishing:**
   ```bash
   ros2 topic echo /r2d2/perception/person_id
   # Should see names like "severin" or "unknown"
   ```

4. **Check audio notification is subscribed:**
   ```bash
   sudo journalctl -u r2d2-audio-notification.service -f
   # Should see "Subscribed to /r2d2/perception/person_id" in startup logs
   ```

### Service auto-restarts frequently

Check logs for the actual error:
```bash
sudo journalctl -u r2d2-audio-notification.service -n 100 | grep -i error
```

Common causes:
- Audio utility not found
- Face recognition topic not available
- ROS 2 environment issue

---

## 📁 Files Reference

| File | Purpose |
|------|---------|
| `/etc/systemd/system/r2d2-audio-notification.service` | Systemd service config |
| `/home/severin/dev/r2d2/start_audio_notification.sh` | Service startup script |
| `/home/severin/dev/r2d2/ros2_ws/src/r2d2_audio/` | ROS 2 package source |
| `/home/severin/dev/r2d2/audio_beep.py` | Audio generation utility |
| `/home/severin/dev/r2d2/QUICK_START.md` | Quick reference guide |
| `/home/severin/dev/r2d2/AUDIO_NOTIFICATION_COMPLETE_DOCUMENTATION.md` | Full documentation |

---

## 🚀 Boot Verification

After system boot, verify the service started:

```bash
# Check service is running
sudo systemctl is-active r2d2-audio-notification.service
# Output: active

# Check it's enabled
sudo systemctl is-enabled r2d2-audio-notification.service
# Output: enabled

# Check recent logs
sudo journalctl -u r2d2-audio-notification.service --since -10m
```

---

## 💡 Tips & Best Practices

1. **Check logs regularly:**
   ```bash
   sudo journalctl -u r2d2-audio-notification.service -f
   ```

2. **Monitor resource usage:**
   ```bash
   ps aux | grep audio_notif
   # Should show ~3-4% CPU, ~20MB RAM
   ```

3. **Verify on reboot:**
   ```bash
   # After rebooting, check:
   sudo systemctl status r2d2-audio-notification.service
   ```

4. **Backup configuration:**
   ```bash
   sudo cp /etc/systemd/system/r2d2-audio-notification.service \
           /home/severin/dev/r2d2/r2d2-audio-notification.service.backup
   ```

---

## 📞 Quick Help

```bash
# Is service running?
sudo systemctl status r2d2-audio-notification.service

# Start now
sudo systemctl start r2d2-audio-notification.service

# See latest errors
sudo journalctl -u r2d2-audio-notification.service -n 20

# Test audio
python3 ~/dev/r2d2/audio_beep.py

# Monitor events
ros2 topic echo /r2d2/audio/notification_event
```

---

## ✅ Setup Complete!

Your R2D2 audio notification system is now:
- ✅ Installed as a systemd service
- ✅ Auto-starting on Jetson boot
- ✅ Running in the background
- ✅ Ready for your face recognition pipeline
- ✅ Configured with optimal settings
- ✅ Tested and verified

**It's working right now!** Check status with:
```bash
sudo systemctl status r2d2-audio-notification.service
```

Enjoy your R2D2 companion! 🤖🔊

