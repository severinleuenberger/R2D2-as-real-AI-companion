#!/bin/bash
# Auto-connect Bluetooth audio device and set as default sink
# Run on boot via systemd service

# Wait for Bluetooth service to be ready
sleep 5

# Wait for PulseAudio to be ready
until pactl info >/dev/null 2>&1; do
    sleep 1
done

# Connect to Bluetooth device (FreeBuds 4i)
bluetoothctl connect 28:54:71:BB:C6:53

# Wait for connection to establish
sleep 3

# Set Bluetooth as default audio sink
if pactl list sinks short | grep -q "bluez_sink.28_54_71_BB_C6_53"; then
    pactl set-default-sink bluez_sink.28_54_71_BB_C6_53.a2dp_sink
    logger -t bluetooth-auto-connect "✅ Bluetooth sink set as default"
else
    logger -t bluetooth-auto-connect "❌ Bluetooth sink not found"
    exit 1
fi

logger -t bluetooth-auto-connect "✅ Bluetooth auto-connect complete"

