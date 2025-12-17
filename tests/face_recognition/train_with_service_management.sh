#!/bin/bash
################################################################################
# Training Helper Script - Safe Service Management
#
# Purpose:
#   Safely manages R2D2 camera perception service during training.
#   Automatically stops the service before training and restarts it after.
#
# Usage:
#   ./train_with_service_management.sh
#
# Features:
#   - Stops camera service before training
#   - Launches train_manager.py in proper environment
#   - Handles Ctrl+C gracefully
#   - Always restarts service (via trap)
#   - Verifies service is running again
#
# Author: R2D2 System
# Date: December 17, 2025
################################################################################

set -e  # Exit on error (except in trap)

# Configuration
SERVICE_NAME="r2d2-camera-perception.service"
VENV_PATH="$HOME/depthai_env"
SCRIPT_DIR="$HOME/dev/r2d2/tests/face_recognition"
TRAIN_SCRIPT="train_manager.py"

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Flag to track if service was running initially
SERVICE_WAS_RUNNING=false

################################################################################
# Cleanup Function - ALWAYS runs on exit
################################################################################
cleanup() {
    local exit_code=$?
    
    echo ""
    echo "========================================================================"
    echo "                      Cleaning Up..."
    echo "========================================================================"
    
    # Only restart if it was running initially
    if [ "$SERVICE_WAS_RUNNING" = true ]; then
        echo -e "${BLUE}Restarting camera perception service...${NC}"
        
        if sudo systemctl start "$SERVICE_NAME"; then
            echo -e "${GREEN}✓ Service start command executed${NC}"
        else
            echo -e "${RED}✗ Failed to start service${NC}"
        fi
        
        # Wait for service to stabilize
        sleep 3
        
        # Verify service is running
        if systemctl is-active --quiet "$SERVICE_NAME"; then
            echo -e "${GREEN}✅ Service restarted successfully!${NC}"
            echo ""
            echo "Camera perception is now active and running."
        else
            echo -e "${RED}❌ Warning: Service did not restart properly!${NC}"
            echo ""
            echo "Please restart manually with:"
            echo "   sudo systemctl start $SERVICE_NAME"
            echo ""
            echo "Check status with:"
            echo "   systemctl status $SERVICE_NAME"
        fi
    else
        echo -e "${YELLOW}Service was not running initially. Not restarting.${NC}"
    fi
    
    echo "========================================================================"
    
    exit $exit_code
}

################################################################################
# Set trap to always run cleanup
################################################################################
trap cleanup EXIT INT TERM

################################################################################
# Header
################################################################################
clear
echo "========================================================================"
echo "               R2D2 Training with Service Management"
echo "========================================================================"
echo ""

################################################################################
# Pre-flight Checks
################################################################################
echo -e "${BLUE}[1/5] Running pre-flight checks...${NC}"

# Check if running as root (we need sudo, but not to be root)
if [ "$EUID" -eq 0 ]; then 
    echo -e "${RED}✗ Please do not run as root. Run as normal user (sudo will be prompted).${NC}"
    exit 1
fi

# Check if virtual environment exists
if [ ! -d "$VENV_PATH" ]; then
    echo -e "${RED}✗ Virtual environment not found at: $VENV_PATH${NC}"
    echo "Please create it first with:"
    echo "   python3 -m venv $VENV_PATH"
    exit 1
fi

# Check if train_manager.py exists
if [ ! -f "$SCRIPT_DIR/$TRAIN_SCRIPT" ]; then
    echo -e "${RED}✗ Training script not found: $SCRIPT_DIR/$TRAIN_SCRIPT${NC}"
    exit 1
fi

echo -e "${GREEN}✓ Pre-flight checks passed${NC}"
echo ""

################################################################################
# Check Service Status
################################################################################
echo -e "${BLUE}[2/5] Checking camera service status...${NC}"

# Check if service exists
if ! systemctl list-unit-files | grep -q "$SERVICE_NAME"; then
    echo -e "${YELLOW}⚠  Service $SERVICE_NAME not found${NC}"
    echo "Training can proceed without stopping service."
    SERVICE_WAS_RUNNING=false
elif systemctl is-active --quiet "$SERVICE_NAME"; then
    echo -e "${GREEN}✓ Service is running${NC}"
    SERVICE_WAS_RUNNING=true
else
    echo -e "${YELLOW}⚠  Service is not running${NC}"
    SERVICE_WAS_RUNNING=false
fi
echo ""

################################################################################
# Stop Service
################################################################################
if [ "$SERVICE_WAS_RUNNING" = true ]; then
    echo -e "${BLUE}[3/5] Stopping camera service...${NC}"
    echo "This requires sudo password."
    echo ""
    
    if sudo systemctl stop "$SERVICE_NAME"; then
        echo -e "${GREEN}✓ Service stopped successfully${NC}"
        
        # Verify it stopped
        sleep 2
        if systemctl is-active --quiet "$SERVICE_NAME"; then
            echo -e "${RED}✗ Service is still running!${NC}"
            echo "Cannot proceed with training."
            exit 1
        fi
    else
        echo -e "${RED}✗ Failed to stop service${NC}"
        exit 1
    fi
else
    echo -e "${BLUE}[3/5] Skipping service stop (not running)${NC}"
fi
echo ""

################################################################################
# Activate Virtual Environment
################################################################################
echo -e "${BLUE}[4/5] Activating virtual environment...${NC}"
echo "Virtual env: $VENV_PATH"

# Source the virtual environment
source "$VENV_PATH/bin/activate"

# Verify Python is from venv
PYTHON_PATH=$(which python3)
if [[ "$PYTHON_PATH" == *"depthai_env"* ]]; then
    echo -e "${GREEN}✓ Virtual environment activated${NC}"
    echo "Python: $PYTHON_PATH"
else
    echo -e "${YELLOW}⚠  Warning: Python may not be from virtual environment${NC}"
    echo "Python: $PYTHON_PATH"
fi
echo ""

################################################################################
# Launch Training
################################################################################
echo -e "${BLUE}[5/5] Launching training manager...${NC}"
echo ""
echo "========================================================================"
echo "                    Training Manager Starting"
echo "========================================================================"
echo ""
echo "You can now train face recognition and gestures."
echo "The camera service will automatically restart when you exit."
echo ""
echo "Press Ctrl+C anytime to exit safely."
echo ""
echo "========================================================================"
echo ""

# Change to script directory
cd "$SCRIPT_DIR"

# Launch train_manager.py
# Note: Set OPENBLAS for ARM optimization
export OPENBLAS_CORETYPE=ARMV8

python3 "$TRAIN_SCRIPT"

################################################################################
# Training completed normally
################################################################################
echo ""
echo -e "${GREEN}✓ Training completed${NC}"

# Cleanup function will run automatically via trap

