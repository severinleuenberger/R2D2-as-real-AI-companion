#!/bin/bash
# Quick Audio Test Script for R2D2

echo "╔════════════════════════════════════════════════════════════╗"
echo "║     R2D2 AUDIO SYSTEM - QUICK TEST WITH CORRECT WIRING     ║"
echo "╚════════════════════════════════════════════════════════════╝"
echo ""
echo "Hardware Setup Verified:"
echo "  ✓ PAM8403 Right channel (R+/R−) → 8Ω speaker"
echo "  ✓ RIN → J511 Pin 9 (HPO_R from Jetson)"
echo "  ✓ GND → J511 Pin 2 (AGND)"
echo "  ✓ Power → 5V + GND"
echo ""

# Test 1: Check ALSA device
echo "Test 1: Check ALSA device..."
aplay -l | grep "card 1"
if [ $? -eq 0 ]; then
    echo "  ✓ Found ALSA device hw:1,0"
else
    echo "  ✗ Could not find ALSA device hw:1,0"
    exit 1
fi
echo ""

# Test 2: Check audio codec
echo "Test 2: Check audio codec..."
if grep -q "CVB-RT" /proc/asound/cards; then
    echo "  ✓ CVB-RT Realtek codec is active"
else
    echo "  ✗ CVB-RT codec not found"
    exit 1
fi
echo ""

# Test 3: Run audio beep
echo "Test 3: Playing 1kHz beep (1 second) at 50% volume..."
echo "  Listening for beep from speaker..."
echo ""

cd /home/severin/dev/r2d2
python3 -c "
from audio_beep import play_beep
import sys
success = play_beep(frequency=1000.0, duration=1.0, volume=0.5, device='hw:1,0')
sys.exit(0 if success else 1)
"

if [ $? -eq 0 ]; then
    echo ""
    echo "  ✓ Beep command executed successfully"
    echo ""
    echo "╔════════════════════════════════════════════════════════════╗"
    echo "║                    RESULTS:                                ║"
    echo "╠════════════════════════════════════════════════════════════╣"
    echo "║                                                            ║"
    echo "║  DID YOU HEAR A BEEP FROM THE SPEAKER?                    ║"
    echo "║                                                            ║"
    echo "║  YES → Hardware is CORRECT! ✓                             ║"
    echo "║        Audio system is working                            ║"
    echo "║        No further soldering needed                        ║"
    echo "║                                                            ║"
    echo "║  NO → Check:                                              ║"
    echo "║     1. Solder joints (use magnifying glass)               ║"
    echo "║     2. Speaker wire connections                          ║"
    echo "║     3. PAM8403 power LED (should be on)                  ║"
    echo "║     4. Multimeter continuity tests                        ║"
    echo "║                                                            ║"
    echo "╚════════════════════════════════════════════════════════════╝"
    echo ""
else
    echo "  ✗ Failed to play beep"
    exit 1
fi
