#!/bin/bash

echo "Configuring mixer for SPEAKER output (RIGHT channel)..."
echo ""

# Make sure Speaker control is unmuted and at full volume
echo "[1] Set CVB-RT Speaker to maximum (39)"
amixer -c 1 set "CVB-RT Speaker" 39

echo ""
echo "[2] Enable CVB-RT Speaker Channel"
amixer -c 1 set "CVB-RT Speaker Channel" on 2>/dev/null || echo "   (control not available)"

echo ""
echo "[3] Enable CVB-RT Speaker L and R"
amixer -c 1 set "CVB-RT Speaker L" on 2>/dev/null || echo "   (L control not available)"
amixer -c 1 set "CVB-RT Speaker R" on 2>/dev/null || echo "   (R control not available)"

echo ""
echo "[4] Current Speaker mixer state:"
amixer -c 1 get "CVB-RT Speaker"

echo ""
echo "Mixer configuration complete!"
