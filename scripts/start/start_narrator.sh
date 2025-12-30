#!/bin/bash
# Start R2D2 Coding Narrator
# This script is called by r2d2-narrator.service

set -e

echo "Starting R2D2 Coding Narrator..."

# Run narrator directly (no ROS 2 dependencies)
exec /usr/bin/python3 /home/severin/dev/r2d2/scripts/util/coding_narrator.py



