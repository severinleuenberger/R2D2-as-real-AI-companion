#!/bin/bash

echo "═══════════════════════════════════════════════════════════════════════"
echo "R2D2 AUDIO HARDWARE DIAGNOSTIC"
echo "═══════════════════════════════════════════════════════════════════════"
echo ""

echo "[STEP 1] Check ALSA recognizes Card 1 (APE)"
echo "───────────────────────────────────────────────────────────────────────"
aplay -l 2>/dev/null | grep -A2 "card 1:" | head -5
echo ""

echo "[STEP 2] List audio device files"
echo "───────────────────────────────────────────────────────────────────────"
ls -la /dev/snd/pcm1d0p 2>/dev/null || echo "⚠ Device node NOT found - system may not recognize audio hardware"
echo ""

echo "[STEP 3] Test SPEAKER-TEST (direct ALSA sine wave - 3 seconds)"
echo "───────────────────────────────────────────────────────────────────────"
echo "Command: speaker-test -t sine -f 1000 -c 2 -D hw:1,0 -d 3"
echo "This plays 1kHz sine wave directly on device hw:1,0"
echo ""
timeout 5 speaker-test -t sine -f 1000 -c 2 -D hw:1,0 -d 3 2>&1 | grep -i "playing\|device\|error\|underrun" || echo "⚠ speaker-test completed (check if you heard anything)"
echo ""

echo "[STEP 4] Check ALSA mixer"
echo "───────────────────────────────────────────────────────────────────────"
amixer -c 1 scontrols 2>/dev/null || echo "⚠ No mixer controls on Card 1 (normal for I2S)"
echo ""

echo "[STEP 5] Verify device permissions"
echo "───────────────────────────────────────────────────────────────────────"
ls -la /dev/snd/pcm* | head -5
echo ""

echo "═══════════════════════════════════════════════════════════════════════"
echo "INTERPRETATION:"
echo "───────────────────────────────────────────────────────────────────────"
echo "✓ If you heard a sine wave tone in STEP 3: Audio hardware IS working"
echo "  → Problem is in our Python code or ALSA config"
echo ""
echo "✗ If you heard NOTHING in STEP 3: Audio hardware NOT working"
echo "  → Problem is in wiring or PAM8403 power supply"
echo "═══════════════════════════════════════════════════════════════════════"
