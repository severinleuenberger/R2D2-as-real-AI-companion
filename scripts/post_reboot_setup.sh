#!/bin/bash
# Post-Reboot Setup Script for R2D2
# Run this after every reboot to ensure all services are operational

echo "🤖 R2D2 Post-Reboot Setup"
echo "========================="
echo ""

# Wait for services to start
echo "⏳ Waiting for services to initialize (10 seconds)..."
sleep 10

# Activate speech node (required after reboot)
echo "🎤 Activating speech service..."
ros2 lifecycle set /speech_node activate
sleep 2

# Verify speech service is active
SPEECH_STATE=$(ros2 lifecycle get /speech_node 2>/dev/null)
if [[ "$SPEECH_STATE" == *"active"* ]]; then
    echo "✅ Speech service: active"
else
    echo "❌ Speech service: $SPEECH_STATE (FAILED)"
fi

# Check all critical services
echo ""
echo "📋 Service Status:"
systemctl is-active r2d2-camera-perception.service && echo "✅ Camera perception" || echo "❌ Camera perception"
systemctl is-active r2d2-audio-notification.service && echo "✅ Audio notification" || echo "❌ Audio notification"
systemctl is-active r2d2-gesture-intent.service && echo "✅ Gesture intent" || echo "❌ Gesture intent"
systemctl is-active r2d2-speech-node.service && echo "✅ Speech node" || echo "❌ Speech node"

echo ""
echo "✅ Setup complete! System ready."
echo ""
echo "Test with: cd ~/dev/r2d2 && python3 tools/minimal_monitor.py"

