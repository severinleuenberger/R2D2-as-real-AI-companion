# 🎵 R2D2 Audio System - Start Here

**Status:** ✅ Complete & Organized | **Updated:** December 8, 2025

---

## Welcome! 👋

Your R2D2 audio system documentation has been consolidated and organized for easy navigation. This page helps you find exactly what you need.

---

## 🚀 I Want To... (Quick Finder)

### ...Test If Audio Works
→ **[`AUDIO_QUICK_REFERENCE.md`](AUDIO_QUICK_REFERENCE.md)** - "Test Audio (Is it working?)"

One command: `python3 ~/dev/r2d2/audio_beep.py`

If you hear a beep, you're good! If not, see troubleshooting below.

---

### ...Set Up Audio for the First Time
→ **[`050_AUDIO_SETUP_AND_CONFIGURATION.md`](050_AUDIO_SETUP_AND_CONFIGURATION.md)** (30-60 minutes)

Follow these steps:
1. Hardware Wiring section (verify connections)
2. Installation & Configuration section (ALSA setup)
3. Test Script section (verify it works)
4. Then proceed to ROS 2 setup below

---

### ...Set Up Audio Notifications with ROS 2
→ **[`060_AUDIO_NOTIFICATIONS_ROS2_INTEGRATION.md`](060_AUDIO_NOTIFICATIONS_ROS2_INTEGRATION.md)** (20-30 minutes)

Choose one:
- **Development?** → Option A: Manual Launch (2 terminals)
- **Production?** → Option B: Background Service (auto-start)

---

### ...Launch Notifications Right Now (Quick Start)
→ **[`AUDIO_QUICK_REFERENCE.md`](AUDIO_QUICK_REFERENCE.md)** - "Quick Start"

**Terminal 1:**
```bash
cd ~/dev/r2d2/ros2_ws && source install/setup.bash
ros2 launch r2d2_audio audio_notification.launch.py
```

**Terminal 2:**
```bash
ros2 launch r2d2_bringup r2d2_camera_perception.launch.py enable_face_recognition:=true
```

You'll hear beeps when you appear/disappear in the camera frame!

---

### ...Install as Auto-Starting Background Service
→ **[`AUDIO_QUICK_REFERENCE.md`](AUDIO_QUICK_REFERENCE.md)** - "Run as Background Service"

```bash
sudo cp /home/severin/dev/r2d2/r2d2-audio-notification.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable r2d2-audio-notification.service
sudo systemctl start r2d2-audio-notification.service
```

Check status: `sudo systemctl status r2d2-audio-notification.service`

---

### ...Customize Beep Volume, Frequency, or Behavior
→ **[`AUDIO_QUICK_REFERENCE.md`](AUDIO_QUICK_REFERENCE.md)** - "Customize Beeps" or "Parameter Reference"

Quick examples:
```bash
# Louder beeps
ros2 launch r2d2_audio audio_notification.launch.py beep_volume:=0.9

# Higher pitch
ros2 launch r2d2_audio audio_notification.launch.py beep_frequency:=1500

# Faster loss detection (3s instead of 5s)
ros2 launch r2d2_audio audio_notification.launch.py \
  jitter_tolerance_seconds:=3.0 loss_confirmation_seconds:=3.0
```

See the docs for all 9 configurable parameters.

---

### ...Troubleshoot "No Sound from Speaker"
→ **[`050_AUDIO_SETUP_AND_CONFIGURATION.md`](050_AUDIO_SETUP_AND_CONFIGURATION.md)** - "Troubleshooting" section

Walk through:
1. Issue 1: Check if hardware is available
2. Issue 2: Audio plays but no sound (check wiring)
3. Issue 3: Check ALSA errors
4. Issue 4: Test I2S data line

Each has diagnostic commands and solutions.

---

### ...Troubleshoot Service Won't Start
→ **[`060_AUDIO_NOTIFICATIONS_ROS2_INTEGRATION.md`](060_AUDIO_NOTIFICATIONS_ROS2_INTEGRATION.md)** - "Troubleshooting" section

Or quick: `sudo journalctl -u r2d2-audio-notification.service -n 20`

---

### ...Fix Bad Solder Joints
→ **[`AUDIO_SOLDERING_CHECKLIST.md`](AUDIO_SOLDERING_CHECKLIST.md)`**

Contains:
- Visual inspection guide (dull vs shiny solder)
- Step-by-step soldering procedure
- Multimeter testing methods
- Verification checklist

---

### ...Understand How It All Works
→ **[`060_AUDIO_NOTIFICATIONS_ROS2_INTEGRATION.md`](060_AUDIO_NOTIFICATIONS_ROS2_INTEGRATION.md)** - "Architecture" and "Code Structure" sections

Covers:
- ROS 2 data flow (face detection → beep)
- State machine diagram
- Node architecture
- Component interaction

---

### ...Find a Quick Command Reference
→ **[`AUDIO_QUICK_REFERENCE.md`](AUDIO_QUICK_REFERENCE.md)`**

Always bookmark this! Contains:
- Service management commands
- Beep customization snippets
- Parameter reference table
- Monitor & debug commands

---

### ...Get Context on Documentation Changes
→ **[`AUDIO_DOCUMENTATION_CONSOLIDATION.md`](AUDIO_DOCUMENTATION_CONSOLIDATION.md)** (this is a meta-doc explaining the reorganization)

Explains:
- What changed (9 files → 3 primary docs)
- Why it changed (organization, maintainability)
- How to migrate from old docs
- Benefits of new structure

---

## 📚 Documentation Organization

```
Audio Documentation (6 active documents)
│
├─ PRIMARY DOCUMENTS (Use these!)
│  ├─ 050_AUDIO_SETUP_AND_CONFIGURATION.md (hardware & ALSA)
│  ├─ 060_AUDIO_NOTIFICATIONS_ROS2_INTEGRATION.md (ROS 2 & service)
│  └─ AUDIO_QUICK_REFERENCE.md (quick commands)
│
├─ SPECIALIZED (Reference as needed)
│  └─ AUDIO_SOLDERING_CHECKLIST.md (hardware assembly)
│
└─ META (Understanding the docs)
   ├─ AUDIO_DOCUMENTATION_INDEX.md (complete navigation guide)
   └─ AUDIO_DOCUMENTATION_CONSOLIDATION.md (what changed & why)

ARCHIVED (Old versions - for reference only)
└─ _ARCHIVED_AUDIO_DOCS_v1/ (10 deprecated files)
```

---

## 🎯 Common Tasks with Estimated Time

| Task | Time | Location |
|------|------|----------|
| Test audio | 1 min | AUDIO_QUICK_REFERENCE.md |
| Launch notifications | 2 min | AUDIO_QUICK_REFERENCE.md |
| Install background service | 5 min | AUDIO_QUICK_REFERENCE.md |
| Change beep volume | 2 min | AUDIO_QUICK_REFERENCE.md |
| Full hardware setup | 60 min | 050_AUDIO_SETUP_AND_CONFIGURATION.md |
| ROS 2 integration | 30 min | 060_AUDIO_NOTIFICATIONS_ROS2_INTEGRATION.md |
| Troubleshoot hardware | 30 min | 050_AUDIO_SETUP_AND_CONFIGURATION.md |
| Troubleshoot service | 15 min | 060_AUDIO_NOTIFICATIONS_ROS2_INTEGRATION.md |
| Reflow solder joints | 20 min | AUDIO_SOLDERING_CHECKLIST.md |

---

## 📱 For Mobile/Quick Reference

**Save this command:**
```bash
echo "AUDIO_QUICK_REFERENCE.md is your best friend for common tasks"
cat /home/severin/dev/r2d2/AUDIO_QUICK_REFERENCE.md | head -100
```

---

## 🆘 Having Issues?

### Audio won't work at all
1. Test: `python3 ~/dev/r2d2/audio_beep.py`
2. If no beep → Go to [`050_AUDIO_SETUP_AND_CONFIGURATION.md`](050_AUDIO_SETUP_AND_CONFIGURATION.md) Troubleshooting
3. Check: Power, wiring, solder joints

### Notifications not triggering
1. Check: `ros2 topic echo /r2d2/perception/person_id`
2. If showing "unknown"/"severin" → Go to [`060_AUDIO_NOTIFICATIONS_ROS2_INTEGRATION.md`](060_AUDIO_NOTIFICATIONS_ROS2_INTEGRATION.md) Troubleshooting
3. Check: Face recognition running, audio working

### Service won't start
1. Check: `sudo journalctl -u r2d2-audio-notification.service -n 20`
2. Follow error message in logs
3. Go to [`060_AUDIO_NOTIFICATIONS_ROS2_INTEGRATION.md`](060_AUDIO_NOTIFICATIONS_ROS2_INTEGRATION.md) Troubleshooting

### Need to customize something
1. What do you want to change?
2. Go to [`AUDIO_QUICK_REFERENCE.md`](AUDIO_QUICK_REFERENCE.md) → Find parameter/command
3. Or see [`060_AUDIO_NOTIFICATIONS_ROS2_INTEGRATION.md`](060_AUDIO_NOTIFICATIONS_ROS2_INTEGRATION.md) → Configuration section

---

## ✅ Getting Started Checklist

- [ ] I know where quick commands are: **[AUDIO_QUICK_REFERENCE.md](AUDIO_QUICK_REFERENCE.md)**
- [ ] I can test audio: `python3 ~/dev/r2d2/audio_beep.py`
- [ ] I can launch notifications: `ros2 launch r2d2_audio audio_notification.launch.py`
- [ ] I can check status: `sudo systemctl status r2d2-audio-notification.service`
- [ ] I can view logs: `sudo journalctl -u r2d2-audio-notification.service -f`
- [ ] I know where detailed docs are when needed

---

## 📖 Documentation at a Glance

### [`050_AUDIO_SETUP_AND_CONFIGURATION.md`](050_AUDIO_SETUP_AND_CONFIGURATION.md)
**The hardware & ALSA bible** (550 lines, 17 KB)
- Hardware wiring diagrams
- ALSA configuration step-by-step
- Test procedures
- Hardware troubleshooting
- Performance specs

### [`060_AUDIO_NOTIFICATIONS_ROS2_INTEGRATION.md`](060_AUDIO_NOTIFICATIONS_ROS2_INTEGRATION.md)
**The ROS 2 notifications bible** (650 lines, 21 KB)
- ROS 2 node architecture
- State machine details
- Background service setup
- All configuration parameters
- ROS 2 troubleshooting
- Code structure

### [`AUDIO_QUICK_REFERENCE.md`](AUDIO_QUICK_REFERENCE.md)
**The command quick reference** (200 lines, 6.4 KB)
- Quick start (3 commands)
- Service management
- Beep customization
- Parameter lookup
- Common tasks

### [`AUDIO_SOLDERING_CHECKLIST.md`](AUDIO_SOLDERING_CHECKLIST.md)
**The hardware assembly guide** (200 lines)
- Soldering procedures
- Multimeter testing
- Bad joint identification
- Component troubleshooting

---

## 🔗 Everything is Cross-Referenced

Each document links to the others, so you can navigate easily:

```
Need hardware details?
→ Found in: 050_AUDIO_SETUP_AND_CONFIGURATION.md
→ Quick link in: AUDIO_QUICK_REFERENCE.md & 060_AUDIO_NOTIFICATIONS_ROS2_INTEGRATION.md

Need ROS 2 details?
→ Found in: 060_AUDIO_NOTIFICATIONS_ROS2_INTEGRATION.md
→ Quick link in: AUDIO_QUICK_REFERENCE.md & 050_AUDIO_SETUP_AND_CONFIGURATION.md

Need a command?
→ Found in: AUDIO_QUICK_REFERENCE.md
→ Full context in: Relevant detailed doc

Need soldering help?
→ Found in: AUDIO_SOLDERING_CHECKLIST.md
→ Referenced from: 050_AUDIO_SETUP_AND_CONFIGURATION.md
```

---

## 💡 Pro Tips

1. **Bookmark AUDIO_QUICK_REFERENCE.md** - You'll use it all the time
2. **Use Ctrl+F** to search within documents (each is well-structured)
3. **Check topic echo first** - `ros2 topic echo /r2d2/audio/notification_event`
4. **View service logs** - `sudo journalctl -u r2d2-audio-notification.service -f`
5. **Keep test audio working** - If `python3 audio_beep.py` works, most issues are ROS 2 related

---

## 🎉 You're Ready!

Everything you need is organized and cross-referenced. Whether you're setting up for the first time, troubleshooting, or just looking for a quick command, you know exactly where to find it.

**Start with:** **[AUDIO_QUICK_REFERENCE.md](AUDIO_QUICK_REFERENCE.md)** → Navigate from there!

---

**Last Updated:** December 8, 2025  
**Navigation Version:** 1.0  
**Status:** ✅ Complete & Cross-Referenced
