#!/bin/bash
# Configure Jetson AGX Orin audio routing for PAM8403 speaker
# This script sets up the ALSA mixer to route audio to J511 headphone output
# Run on boot to ensure audio works correctly

# Wait for audio devices to be ready
sleep 2

# Route ADMAIF1 (hw:APE,0) to I2S6 output (which feeds the RT codec/J511 header)
amixer -c 1 cset numid=1218 1 > /dev/null 2>&1

# Set HP volume to maximum
amixer -c 1 cset numid=310 39 > /dev/null 2>&1

# Enable HPO mixer HPVOL switch
amixer -c 1 cset numid=1331 on > /dev/null 2>&1

echo "Audio routing configured: ADMAIF1 -> I2S6 -> J511 headphone output"

