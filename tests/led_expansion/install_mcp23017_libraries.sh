#!/bin/bash
# Install MCP23017 Python Libraries
# Date: January 9, 2026

echo "======================================"
echo "MCP23017 Library Installation"
echo "======================================"
echo ""

echo "Installing Adafruit Blinka (CircuitPython compatibility layer)..."
pip3 install --user adafruit-blinka

echo ""
echo "Installing MCP230xx library..."
pip3 install --user adafruit-circuitpython-mcp230xx

echo ""
echo "Installing additional dependencies..."
pip3 install --user adafruit-circuitpython-busdevice

echo ""
echo "======================================"
echo "Verifying Installation"
echo "======================================"
echo ""

python3 << 'EOF'
try:
    import board
    import busio
    from adafruit_mcp230xx.mcp23017 import MCP23017
    import digitalio
    print("✅ All libraries imported successfully!")
    print("")
    print("Available I2C buses:")
    print(f"  - SDA: {board.SDA}")
    print(f"  - SCL: {board.SCL}")
    print("")
    print("✅ Ready to use MCP23017!")
except ImportError as e:
    print(f"❌ Import failed: {e}")
    print("")
    print("Please check installation and try again.")
    exit(1)
EOF

echo ""
echo "======================================"
echo "Installation Complete!"
echo "======================================"
echo ""
echo "Next step: Connect MCP23017 to Jetson I2C bus and run:"
echo "  sudo i2cdetect -y 1"
echo ""

