#!/bin/bash
# Check for errors in the camera perception service

echo "Checking last 100 lines of camera-perception service logs for errors..."
echo "========================================"
echo ""

sudo journalctl -u r2d2-camera-perception.service -n 100 --no-pager 2>/dev/null | grep -iE "(error|exception|traceback|failed|warning)" | tail -30

echo ""
echo "========================================"
echo "Checking if image_listener is logging anything..."
echo ""

sudo journalctl -u r2d2-camera-perception.service -n 50 --no-pager 2>/dev/null | grep "image_listener" | tail -20

echo ""
echo "========================================"
echo "Raw last 20 lines of log:"
echo ""

sudo journalctl -u r2d2-camera-perception.service -n 20 --no-pager 2>/dev/null

