#!/bin/bash

echo "═══════════════════════════════════════════════════════════════════════"
echo "R2D2 AUDIO MIXER CONFIGURATION - ENABLE HEADPHONE OUTPUT"
echo "═══════════════════════════════════════════════════════════════════════"
echo ""

# Check current HP (headphone) state
echo "[1] Current Headphone Output State:"
echo "───────────────────────────────────────────────────────────────────────"
amixer -c 1 sget "CVB-RT HP" 
echo ""

# Check DAC settings  
echo "[2] Current DAC Output State:"
echo "───────────────────────────────────────────────────────────────────────"
amixer -c 1 sget "CVB-RT DAC1"
amixer -c 1 sget "CVB-RT DAC2"
echo ""

# Enable Headphone output
echo "[3] ENABLING Headphone Output..."
echo "───────────────────────────────────────────────────────────────────────"
amixer -c 1 sset "CVB-RT HP" on || echo "Could not enable HP"
amixer -c 1 sset "CVB-RT HP L" on || echo "Could not enable HP L"
amixer -c 1 sset "CVB-RT HP R" on || echo "Could not enable HP R"
echo "✓ Headphone output enabled"
echo ""

# Set headphone volume to maximum safe level  
echo "[4] Setting Headphone Volume to 80% (safe level)..."
echo "───────────────────────────────────────────────────────────────────────"
# CVB-RT headphone volume typically 0-31 range, so 80% = ~25
amixer -c 1 sset "CVB-RT HP" 80% || echo "Could not set volume"
echo ""

# Enable DAC output
echo "[5] Enabling DAC channels..."
echo "───────────────────────────────────────────────────────────────────────"
amixer -c 1 sset "CVB-RT DAC1" on || echo "Could not enable DAC1"
amixer -c 1 sset "CVB-RT DAC2" on || echo "Could not enable DAC2"
echo "✓ DAC channels enabled"
echo ""

echo "[6] Final State - Verify changes:"
echo "───────────────────────────────────────────────────────────────────────"
amixer -c 1 sget "CVB-RT HP"
amixer -c 1 sget "CVB-RT DAC1"
echo ""

echo "═══════════════════════════════════════════════════════════════════════"
echo "Now testing audio output..."
echo "═══════════════════════════════════════════════════════════════════════"
echo ""
echo "Playing 1kHz tone for 3 seconds..."
speaker-test -t sine -f 1000 -c 2 -D hw:1,0 -d 3
echo ""
echo "✓ Test complete - Did you hear the tone?"
