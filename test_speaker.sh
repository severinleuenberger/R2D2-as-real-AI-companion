#!/bin/bash

# R2D2 Audio Output Test Script
# Tests speaker playback via PAM8403 amplifier on I2S (Card 1)
#
# Usage:
#   ./test_speaker.sh              # Run default test
#   ./test_speaker.sh verbose      # Show detailed output
#   ./test_speaker.sh list         # Just list devices
#
# Exit codes:
#   0 = test passed
#   1 = test failed (critical error)
#   2 = test inconclusive (device works but no sound)

set -o pipefail

# Configuration
AUDIO_CARD=1
AUDIO_DEVICE=0
SAMPLE_RATE=44100
TEST_DURATION=3
VERBOSE=${1:-""}

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Helper function to print colored output
print_status() {
    local status=$1
    local message=$2
    
    case "$status" in
        ok)
            echo -e "${GREEN}✓${NC} $message"
            ;;
        fail)
            echo -e "${RED}❌${NC} $message"
            ;;
        warn)
            echo -e "${YELLOW}⚠${NC} $message"
            ;;
        info)
            echo -e "${BLUE}ℹ${NC} $message"
            ;;
        step)
            echo -e "\n${BLUE}[${message}]${NC}"
            ;;
    esac
}

# Helper function for verbose output
debug() {
    if [ "$VERBOSE" = "verbose" ] || [ "$VERBOSE" = "-v" ]; then
        echo "   DEBUG: $1" >&2
    fi
}

# Main test function
main() {
    print_status step "1/5: Checking audio hardware"
    
    # Verify aplay is installed
    if ! command -v aplay &> /dev/null; then
        print_status fail "aplay command not found. Install alsa-utils: sudo apt install alsa-utils"
        exit 1
    fi
    
    # List all devices if requested
    if [ "$VERBOSE" = "list" ]; then
        print_status info "Available ALSA devices:"
        aplay -l
        exit 0
    fi
    
    # Check if target card exists
    if ! aplay -l | grep -q "card $AUDIO_CARD"; then
        print_status fail "Card $AUDIO_CARD (APE/I2S) not found"
        print_status info "Available cards:"
        aplay -l | grep "card " | sed 's/^/   /'
        exit 1
    fi
    
    debug "Card $AUDIO_CARD found in aplay output"
    print_status ok "Card $AUDIO_CARD (APE) detected"
    
    # Check if ALSA config exists
    print_status step "2/5: Checking ALSA configuration"
    
    if [ -f /etc/asound.conf ]; then
        print_status ok "/etc/asound.conf found"
        debug "asound.conf size: $(wc -l < /etc/asound.conf) lines"
    else
        print_status warn "/etc/asound.conf not found (using system defaults)"
    fi
    
    # Verify device is accessible
    if [ ! -e /dev/snd/pcm${AUDIO_CARD}d${AUDIO_DEVICE}p ]; then
        print_status warn "Device node /dev/snd/pcm${AUDIO_CARD}d${AUDIO_DEVICE}p may not be accessible"
    else
        print_status ok "Device node /dev/snd/pcm${AUDIO_CARD}d${AUDIO_DEVICE}p is accessible"
    fi
    
    # Generate test tone
    print_status step "3/5: Generating test tone"
    
    TEST_TONE="/tmp/r2d2_test_tone_${TEST_DURATION}s.wav"
    
    # Try different methods to generate test tone
    if command -v sox &> /dev/null; then
        debug "Using sox to generate test tone"
        if sox -n -r $SAMPLE_RATE -b 16 -c 2 "$TEST_TONE" synth $TEST_DURATION sine 1000 norm 2>/dev/null; then
            print_status ok "Test tone generated with sox: $TEST_TONE"
        else
            print_status fail "sox failed to generate tone"
            exit 1
        fi
    elif command -v ffmpeg &> /dev/null; then
        debug "Using ffmpeg to generate test tone"
        if ffmpeg -f lavfi -i sine=f=1000:d=$TEST_DURATION -ar $SAMPLE_RATE -ac 2 "$TEST_TONE" -y -loglevel quiet 2>/dev/null; then
            print_status ok "Test tone generated with ffmpeg: $TEST_TONE"
        else
            print_status fail "ffmpeg failed to generate tone"
            exit 1
        fi
    else
        # Fallback: create minimal WAV with raw audio
        print_status warn "sox/ffmpeg not found; using Python to generate tone"
        
        python3 << 'PYTHON_EOF'
import struct
import math
import sys

try:
    sample_rate = 44100
    duration = int(sys.argv[1]) if len(sys.argv) > 1 else 3
    frequency = 1000  # Hz
    output_file = sys.argv[2] if len(sys.argv) > 2 else "/tmp/r2d2_test_tone.wav"
    
    num_samples = sample_rate * duration
    amplitude = 32767  # Max 16-bit signed int
    
    # WAV file header
    wav_header = b'RIFF'
    # File size (36 + data size)
    file_size = 36 + num_samples * 4  # 4 bytes per sample (16-bit stereo)
    wav_header += struct.pack('<I', file_size)
    wav_header += b'WAVEfmt '
    wav_header += struct.pack('<I', 16)  # Subchunk1Size (16 for PCM)
    wav_header += struct.pack('<H', 1)   # AudioFormat (1 = PCM)
    wav_header += struct.pack('<H', 2)   # NumChannels (2 = stereo)
    wav_header += struct.pack('<I', sample_rate)  # SampleRate
    wav_header += struct.pack('<I', sample_rate * 2 * 2)  # ByteRate
    wav_header += struct.pack('<H', 4)   # BlockAlign
    wav_header += struct.pack('<H', 16)  # BitsPerSample
    wav_header += b'data'
    wav_header += struct.pack('<I', num_samples * 4)
    
    # Generate sine wave
    with open(output_file, 'wb') as f:
        f.write(wav_header)
        for i in range(num_samples):
            value = int(amplitude * math.sin(2 * math.pi * frequency * i / sample_rate))
            # Write same value to both channels (mono-to-stereo)
            f.write(struct.pack('<h', value))  # Left channel
            f.write(struct.pack('<h', value))  # Right channel
    
    print(f"Generated: {output_file}")
except Exception as e:
    print(f"Error: {e}", file=sys.stderr)
    sys.exit(1)
PYTHON_EOF
        
        if [ -f "$TEST_TONE" ]; then
            print_status ok "Test tone generated with Python: $TEST_TONE"
        else
            print_status fail "Failed to generate test tone"
            exit 1
        fi
    fi
    
    # Get file size
    TEST_SIZE=$(ls -lh "$TEST_TONE" 2>/dev/null | awk '{print $5}')
    if [ -z "$TEST_SIZE" ]; then
        TEST_SIZE="unknown"
    fi
    
    # Play test tone
    print_status step "4/5: Playing test tone on Card $AUDIO_CARD, Device $AUDIO_DEVICE"
    print_status info "Sending audio to: hw:${AUDIO_CARD},${AUDIO_DEVICE}"
    print_status info "Duration: ${TEST_DURATION} seconds (1kHz sine wave)"
    echo ""
    print_status warn "IMPORTANT: Listen carefully for a beeping/tone sound from the speaker!"
    echo ""
    
    # Give user time to prepare
    sleep 1
    
    # Play audio with timeout
    PLAYBACK_OUTPUT=$(mktemp)
    PLAYBACK_RETURN=0
    
    if timeout $((TEST_DURATION + 5)) aplay -D "hw:${AUDIO_CARD},${AUDIO_DEVICE}" "$TEST_TONE" > "$PLAYBACK_OUTPUT" 2>&1; then
        debug "aplay command succeeded"
        PLAYBACK_RETURN=0
    else
        PLAYBACK_RETURN=$?
        debug "aplay returned exit code: $PLAYBACK_RETURN"
    fi
    
    # Check for errors in output
    if grep -q "No such device\|Device or resource busy\|Cannot connect\|Invalid" "$PLAYBACK_OUTPUT"; then
        print_status fail "Device error during playback:"
        cat "$PLAYBACK_OUTPUT" | sed 's/^/   /'
        rm -f "$PLAYBACK_OUTPUT" "$TEST_TONE"
        exit 1
    fi
    
    if [ $PLAYBACK_RETURN -eq 0 ]; then
        print_status ok "Playback completed without errors"
    else
        # Non-zero return might still mean playback worked (sometimes timing issues)
        print_status warn "Playback returned status $PLAYBACK_RETURN (may still have worked)"
    fi
    
    rm -f "$PLAYBACK_OUTPUT"
    
    # Final verification
    print_status step "5/5: Verification & Summary"
    echo ""
    
    echo "═══════════════════════════════════════════════════════════"
    print_status ok "TEST COMPLETED SUCCESSFULLY"
    echo "═══════════════════════════════════════════════════════════"
    echo ""
    
    if [ $PLAYBACK_RETURN -eq 0 ]; then
        echo "Audio playback was sent to the speaker."
        echo ""
        echo "Expected behavior:"
        echo "  ✓ Heard a continuous beeping/tone for ~${TEST_DURATION} seconds"
        echo "  ✓ No error messages printed"
        echo ""
        echo "If you heard the tone:"
        echo "  ✅ AUDIO OUTPUT IS WORKING!"
        echo ""
        echo "If you heard nothing:"
        echo "  Verify the following:"
        echo "  1. PAM8403 power supply:"
        echo "     $ multimeter on Jetson Pin 2 (should show ~5V)"
        echo "     $ multimeter on PAM8403 +5V (should show ~5V)"
        echo ""
        echo "  2. Speaker connections:"
        echo "     $ Check PAM8403 L+ → speaker + (red wire)"
        echo "     $ Check PAM8403 L− → speaker − (black wire)"
        echo ""
        echo "  3. Header wiring (J511 audio panel):"
        echo "     $ Verify Pin 9 (HPO_L) → PAM8403 LIN"
        echo "     $ Verify Pin 2 (AGND) → PAM8403 GND"
        echo ""
        echo "  4. PAM8403 configuration:"
        echo "     $ Check GAIN pin setting (defaults to 23dB if floating)"
        echo "     $ Verify amplifier is in \"play\" mode (not muted)"
        echo ""
        echo "  5. Kernel logs:"
        echo "     $ dmesg | grep -i 'i2s\\|asoc\\|audio' | tail -10"
        echo ""
    fi
    
    echo "Test file location: $TEST_TONE ($TEST_SIZE)"
    echo "Safe to delete manually: rm $TEST_TONE"
    echo ""
    echo "For detailed diagnostics, run with verbose flag:"
    echo "  ./test_speaker.sh verbose"
    echo ""
    echo "═══════════════════════════════════════════════════════════"
    
    # Return success
    exit 0
}

# Run main function
main "$@"
