"""Configuration for R2D2 Web Dashboard"""
import os
from pathlib import Path

# Base directory
BASE_DIR = Path(__file__).parent.parent

# Server configuration
HOST = os.getenv("WEB_DASHBOARD_HOST", "0.0.0.0")
PORT = int(os.getenv("WEB_DASHBOARD_PORT", "8080"))

# ROS 2 configuration
ROS_DOMAIN_ID = int(os.getenv("ROS_DOMAIN_ID", "0"))

# rosbridge configuration
ROSBRIDGE_HOST = os.getenv("ROSBRIDGE_HOST", "localhost")
ROSBRIDGE_PORT = int(os.getenv("ROSBRIDGE_PORT", "9090"))

# Service names
SERVICES = {
    "audio": "r2d2-audio-notification.service",
    "camera": "r2d2-camera-perception.service",
    "powerbutton": "r2d2-powerbutton.service"
}

# Training scripts directory
TRAINING_SCRIPT_DIR = Path.home() / "dev" / "r2d2" / "tests" / "face_recognition"
TRAINING_BASE_DIR = Path.home() / "dev" / "r2d2" / "data" / "face_recognition"

# Ensure directories exist (create if missing)
TRAINING_SCRIPT_DIR.mkdir(parents=True, exist_ok=True)
TRAINING_BASE_DIR.mkdir(parents=True, exist_ok=True)

