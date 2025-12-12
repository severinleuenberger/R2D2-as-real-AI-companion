#!/bin/bash
# Simple startup script for web dashboard (just FastAPI, assumes rosbridge is running separately)

cd "$(dirname "$0")"

# Activate virtual environment if it exists
if [ -f "web_dashboard_env/bin/activate" ]; then
    source web_dashboard_env/bin/activate
fi

# Start FastAPI server
python3 -m app.main


