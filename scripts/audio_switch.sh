#!/bin/bash
# R2D2 Audio Output Switcher (Software-based)
# Usage: audio_switch.sh [bluetooth|pam8403|status]

BLUETOOTH_SINK="bluez_sink.28_54_71_BB_C6_53.a2dp_sink"
PAM8403_SINK="alsa_output.platform-sound.analog-stereo"
CONFIG_FILE="$HOME/.r2d2_audio_output"

case "$1" in
    bluetooth)
        pactl set-default-sink "$BLUETOOTH_SINK" && \
        echo "bluetooth" > "$CONFIG_FILE" && \
        echo "✓ Audio output: BLUETOOTH (FreeBuds 4i)"
        ;;
    pam8403)
        pactl set-default-sink "$PAM8403_SINK" && \
        echo "pam8403" > "$CONFIG_FILE" && \
        echo "✓ Audio output: PAM8403 (Onboard Speaker)"
        ;;
    status)
        CURRENT=$(pactl get-default-sink)
        echo "Current sink: $CURRENT"
        if [[ "$CURRENT" == *"bluez"* ]]; then
            echo "Mode: BLUETOOTH"
        else
            echo "Mode: PAM8403"
        fi
        ;;
    *)
        echo "Usage: $0 {bluetooth|pam8403|status}"
        exit 1
        ;;
esac

