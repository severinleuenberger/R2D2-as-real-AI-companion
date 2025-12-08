# R2D2 Audio Notification - Quick Start (30 seconds)

## Launch Now (Development)

```bash
cd ~/dev/r2d2/ros2_ws
source install/setup.bash
ros2 launch r2d2_audio audio_notification.launch.py
```

Your R2D2 will beep when it recognizes you! 🔊

---

## Install as Background Service (Auto-Start)

```bash
sudo cp /home/severin/dev/r2d2/r2d2-audio-notification.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable r2d2-audio-notification.service
sudo systemctl start r2d2-audio-notification.service
```

Check status:
```bash
sudo systemctl status r2d2-audio-notification.service
```

---

## Test Audio

```bash
python3 ~/dev/r2d2/audio_beep.py
```

You should hear a beep. If not, check J511 Pin 5 audio wiring.

---

## What You'll Hear

| Event | Sound |
|-------|-------|
| Face recognized | 🔊 Single beep (400 Hz, deep) |
| Face lost (5+ sec) | 🔔🔔 Double beep (400 Hz, deep) |
| Face returns | 🔊 Single beep |

---

## Monitor in Real-Time

```bash
# Watch events
ros2 topic echo /r2d2/audio/notification_event

# Watch service logs
sudo journalctl -u r2d2-audio-notification.service -f
```

---

## Customize Beeps

```bash
# Louder
ros2 launch r2d2_audio audio_notification.launch.py beep_volume:=0.5

# Faster loss detection (3 sec instead of 5)
ros2 launch r2d2_audio audio_notification.launch.py loss_confirmation_seconds:=3

# Different frequency (higher pitched)
ros2 launch r2d2_audio audio_notification.launch.py beep_frequency:=600
```

---

## Service Management

```bash
# Status
sudo systemctl status r2d2-audio-notification.service

# Stop
sudo systemctl stop r2d2-audio-notification.service

# Start
sudo systemctl start r2d2-audio-notification.service

# Restart
sudo systemctl restart r2d2-audio-notification.service

# View last 20 lines of logs
sudo journalctl -u r2d2-audio-notification.service -n 20
```

---

## Done! 🎉

Your R2D2 is ready to recognize and notify you with audio alerts!

For complete documentation, see: `AUDIO_NOTIFICATION_COMPLETE_DOCUMENTATION.md`
