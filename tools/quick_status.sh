#!/bin/bash
# Quick R2D2 status check

echo "=== R2D2 System Status Check ==="
echo ""

echo "1. Service Status:"
systemctl is-active r2d2-camera-perception.service && echo "  ✅ Camera/Perception: Running" || echo "  ❌ Camera/Perception: Stopped"
systemctl is-active r2d2-audio-notification.service && echo "  ✅ Audio Notification: Running" || echo "  ❌ Audio Notification: Stopped"
systemctl is-active r2d2-gesture-intent.service && echo "  ✅ Gesture Intent: Running" || echo "  ❌ Gesture Intent: Stopped"
pgrep -f "speech_node" > /dev/null && echo "  ✅ Speech Node: Running" || echo "  ❌ Speech Node: Stopped"

echo ""
echo "2. Recent Recognition Events (last 2 minutes):"
journalctl -u r2d2-audio-notification.service --since "2 minutes ago" --no-pager | grep -E "(RED-FIRST|recognized)" | tail -3

echo ""
echo "3. Recent Gesture Events (last 2 minutes):"
journalctl -u r2d2-gesture-intent.service --since "2 minutes ago" --no-pager | grep "Gesture detected" | tail -3

echo ""
echo "4. Current Person Status:"
timeout 2 ros2 topic echo /r2d2/audio/person_status --once 2>/dev/null | grep -E "(status|person_identity|duration)" || echo "  ⚠️  No status data (timeout)"

echo ""
echo "5. Audio Playback:"
if command -v ffplay &> /dev/null; then
    echo "  ✅ ffplay installed"
else
    echo "  ❌ ffplay NOT installed"
fi

echo ""
echo "=== Quick Diagnostics ==="
echo "To monitor gestures in real-time:"
echo "  python3 /home/severin/dev/r2d2/tools/gesture_monitor.py"
echo ""
echo "To test audio at 2% volume:"
echo "  ffplay -nodisp -autoexit -af volume=0.02 ~/dev/r2d2/ros2_ws/install/r2d2_audio/share/r2d2_audio/assets/audio/\"Voicy_R2-D2 - 2.mp3\""
echo ""
echo "To test audio at 30% volume:"
echo "  ffplay -nodisp -autoexit -af volume=0.3 ~/dev/r2d2/ros2_ws/install/r2d2_audio/share/r2d2_audio/assets/audio/\"Voicy_R2-D2 - 2.mp3\""

