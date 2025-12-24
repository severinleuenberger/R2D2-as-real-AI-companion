#!/bin/bash
# Test R2D2 beep audio playback

echo "========================================"
echo "R2D2 Beep Audio Test"
echo "========================================"
echo ""

BEEP_FILE="/home/severin/dev/r2d2/assets/audio/r2d2_beep.mp3"

# Check if file exists
if [ ! -f "$BEEP_FILE" ]; then
    echo "❌ Beep file not found: $BEEP_FILE"
    exit 1
fi

echo "✅ Beep file exists: $BEEP_FILE"
echo ""

# Show file info
echo "File info:"
ls -lh "$BEEP_FILE"
echo ""

# Check if it's a symlink
if [ -L "$BEEP_FILE" ]; then
    echo "This is a symlink pointing to:"
    readlink -f "$BEEP_FILE"
    echo ""
fi

# Test 1: Play with ffplay (what the gesture_intent_node uses)
echo "========================================"
echo "Test 1: Playing with ffplay"
echo "========================================"
echo ""
echo "This is the method the gesture_intent_node uses."
echo "You should hear a R2D2 beep sound now..."
echo ""

if command -v ffplay &> /dev/null; then
    ffplay -nodisp -autoexit -loglevel quiet "$BEEP_FILE" 2>&1
    echo ""
    echo "Did you hear the R2D2 beep? (y/n)"
else
    echo "❌ ffplay not found! This is required for audio playback."
    echo "Install with: sudo apt-get install ffmpeg"
fi

echo ""

# Test 2: Play with aplay (alternative test)
echo "========================================"
echo "Test 2: Alternative test with mpg123"
echo "========================================"
echo ""

if command -v mpg123 &> /dev/null; then
    echo "Playing with mpg123..."
    mpg123 -q "$BEEP_FILE" 2>&1
    echo ""
    echo "Did you hear the beep with mpg123? (y/n)"
else
    echo "mpg123 not installed (optional, ffplay is the primary method)"
fi

echo ""
echo "========================================"
echo "Audio Test Complete"
echo "========================================"
echo ""
echo "If you heard the beep(s), audio is working!"
echo "If not, check:"
echo "  1. Volume level (alsamixer)"
echo "  2. Audio output device"
echo "  3. Speaker connections"

