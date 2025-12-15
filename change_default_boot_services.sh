#!/bin/bash
# Change default boot behavior: Recognition Status instead of Video Stream
# This script disables camera-stream service and ensures camera-perception and audio-notification are enabled

set -e

echo "Changing default boot behavior..."
echo ""

# Disable camera-stream service
echo "1. Disabling r2d2-camera-stream.service from auto-starting on boot..."
sudo systemctl disable r2d2-camera-stream.service
echo "   ✓ Camera stream service disabled"

# Verify camera-perception service is enabled
echo ""
echo "2. Verifying r2d2-camera-perception.service is enabled..."
sudo systemctl enable r2d2-camera-perception.service
echo "   ✓ Camera perception service enabled"

# Verify audio-notification service is enabled
echo ""
echo "3. Verifying r2d2-audio-notification.service is enabled..."
sudo systemctl enable r2d2-audio-notification.service
echo "   ✓ Audio notification service enabled"

# Verify final state
echo ""
echo "4. Verifying service enable states..."
echo ""
CAMERA_STREAM=$(systemctl is-enabled r2d2-camera-stream.service)
CAMERA_PERCEPTION=$(systemctl is-enabled r2d2-camera-perception.service)
AUDIO_NOTIFICATION=$(systemctl is-enabled r2d2-audio-notification.service)

echo "   r2d2-camera-stream.service:      $CAMERA_STREAM"
echo "   r2d2-camera-perception.service: $CAMERA_PERCEPTION"
echo "   r2d2-audio-notification.service: $AUDIO_NOTIFICATION"
echo ""

# Check if configuration is correct
if [ "$CAMERA_STREAM" = "disabled" ] && [ "$CAMERA_PERCEPTION" = "enabled" ] && [ "$AUDIO_NOTIFICATION" = "enabled" ]; then
    echo "✓ Configuration correct!"
    echo ""
    echo "After reboot:"
    echo "  - Video stream will NOT start automatically"
    echo "  - Recognition Status (camera-perception + audio-notification) will start automatically"
    echo ""
    echo "To test, reboot the system: sudo reboot"
else
    echo "⚠ Warning: Configuration may not be as expected"
    exit 1
fi

