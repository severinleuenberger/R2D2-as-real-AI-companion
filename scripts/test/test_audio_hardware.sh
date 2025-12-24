#!/bin/bash

echo "═══════════════════════════════════════════════════════════"
echo "R2D2 AUDIO HARDWARE DIAGNOSIS"
echo "═══════════════════════════════════════════════════════════"
echo ""

# Test 1: Verify ALSA can see the device
echo "[TEST 1] ALSA Device Detection"
echo "───────────────────────────────────────────────────────────"
if aplay -l 2>/dev/null | grep -q "card 1.*APE"; then
    echo "✓ Card 1 (APE) detected"
    aplay -l | grep "card 1" | head -2
else
    echo "✗ Card 1 NOT detected"
    exit 1
fi
echo ""

# Test 2: Check mixer
echo "[TEST 2] Mixer Status - CVB-RT Codec"
echo "───────────────────────────────────────────────────────────"
amixer -c 1 get "CVB-RT HP" 2>/dev/null || echo "Checking alternative mixer controls..."
amixer -c 1 scontrols | grep -i "hp\|speaker\|dac" || echo "No mixer controls found"
echo ""

# Test 3: Direct speaker-test to hw:1,0
echo "[TEST 3] Direct ALSA Test - 3 second 1kHz tone on hw:1,0"
echo "───────────────────────────────────────────────────────────"
echo "Running: speaker-test -t sine -f 1000 -c 2 -D hw:1,0 -d 3"
echo "LISTEN CAREFULLY FOR TONE..."
speaker-test -t sine -f 1000 -c 2 -D hw:1,0 -d 3 2>&1 | grep -E "Playback|playing|failed"
echo ""

# Test 4: Check for alternative I2S devices
echo "[TEST 4] List All PCM Devices"
echo "───────────────────────────────────────────────────────────"
cat /proc/asound/cards
echo ""

# Test 5: Check J511 header pinout
echo "[TEST 5] J511 Audio Header Pinout"
echo "───────────────────────────────────────────────────────────"
echo "Expected connections:"
echo "  Jetson J511 Pin 9  (HPO_L) → PAM8403 LIN"
echo "  Jetson J511 Pin 2  (AGND)  → PAM8403 GND"
echo "  Jetson J511 Pin 10 (HPO_R) → (not used)"
echo ""
echo "Check your wiring visually - are these connections solid?"
echo ""

# Test 6: Test with aplay directly to file
echo "[TEST 6] Generate and Test WAV File"
echo "───────────────────────────────────────────────────────────"
python3 << 'PYTHON'
import struct, math
sr = 44100
duration = 2
freq = 1000
amp = 32767

wav = b'RIFF'
file_size = 36 + sr*duration*4
wav += struct.pack('<I', file_size)
wav += b'WAVEfmt '
wav += struct.pack('<I', 16)
wav += struct.pack('<H', 1)
wav += struct.pack('<H', 2)
wav += struct.pack('<I', sr)
wav += struct.pack('<I', sr*4)
wav += struct.pack('<H', 4)
wav += struct.pack('<H', 16)
wav += b'data'
wav += struct.pack('<I', sr*duration*4)

for i in range(sr*duration):
    sample = int(amp * math.sin(2 * math.pi * freq * i / sr))
    wav += struct.pack('<hh', sample, sample)

with open('/tmp/test_1khz_2sec.wav', 'wb') as f:
    f.write(wav)
print("✓ Test WAV file created: /tmp/test_1khz_2sec.wav")
PYTHON

echo ""
echo "Playing WAV file directly to hw:1,0:"
echo "Running: aplay -D hw:1,0 /tmp/test_1khz_2sec.wav"
echo "LISTEN FOR 2-SECOND TONE..."
aplay -D hw:1,0 /tmp/test_1khz_2sec.wav 2>&1 | grep -E "Playing|Error|Cannot"
echo ""

echo "═══════════════════════════════════════════════════════════"
echo "NEXT STEPS:"
echo "───────────────────────────────────────────────────────────"
echo "If you heard NO tones:"
echo "  1. Check J511 Pin 9 voltage with oscilloscope"
echo "  2. Verify PAM8403 +5V input with multimeter"
echo "  3. Check PAM8403 LIN pin voltage during playback"
echo "  4. Verify speaker is not burned out (test with different speaker)"
echo "  5. Check for solder bridges or loose connections"
echo ""
echo "If you heard SOME tones but not all:"
echo "  1. Device may be getting 'locked' by ALSA"
echo "  2. Try: sudo systemctl restart alsa-utils"
echo ""
echo "═══════════════════════════════════════════════════════════"
