#!/bin/bash
# Quick I2C Detection and Library Installation Script
# Run this AFTER you've connected MCP23017 to Jetson I2C bus
# Date: January 9, 2026

echo "======================================"
echo "MCP23017 I2C Detection & Setup"
echo "======================================"
echo ""

echo "Step 1: Checking I2C Detection"
echo "─────────────────────────────────────"
echo "Scanning I2C bus 1 (40-pin header)..."
echo ""

sudo i2cdetect -y 1

echo ""
echo "VERIFY: Do you see '20' at address 0x20?"
echo "  If YES: MCP23017 is detected ✅"
echo "  If NO: Check wiring (Pins 3, 5, 1, 6)"
echo ""
read -p "Press Enter to continue with library installation..."

echo ""
echo "Step 2: Installing Python Libraries"
echo "─────────────────────────────────────"

./install_mcp23017_libraries.sh

echo ""
echo "======================================"
echo "Setup Complete!"
echo "======================================"
echo ""
echo "Next step: Test the LEDs"
echo "  python3 test_mcp23017_4led_basic.py"
echo ""

