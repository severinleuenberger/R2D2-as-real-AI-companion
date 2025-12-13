#!/bin/bash
# Simple startup script for web dashboard (just FastAPI, assumes rosbridge is running separately)

# #region agent log
echo "{\"sessionId\":\"debug-session\",\"runId\":\"startup\",\"hypothesisId\":\"A\",\"location\":\"start_server.sh:5\",\"message\":\"Script started\",\"data\":{\"pwd\":\"$(pwd)\",\"script\":\"$0\"},\"timestamp\":$(date +%s%3N)}" >> /home/severin/.cursor/debug.log
# #endregion

cd "$(dirname "$0")"

# #region agent log
echo "{\"sessionId\":\"debug-session\",\"runId\":\"startup\",\"hypothesisId\":\"A\",\"location\":\"start_server.sh:10\",\"message\":\"Changed directory\",\"data\":{\"new_dir\":\"$(pwd)\",\"venv_exists\":\"$([ -f web_dashboard_env/bin/activate ] && echo true || echo false)\"},\"timestamp\":$(date +%s%3N)}" >> /home/severin/.cursor/debug.log
# #endregion

# Activate virtual environment if it exists
if [ -f "web_dashboard_env/bin/activate" ]; then
    source web_dashboard_env/bin/activate
    # #region agent log
    echo "{\"sessionId\":\"debug-session\",\"runId\":\"startup\",\"hypothesisId\":\"D\",\"location\":\"start_server.sh:15\",\"message\":\"Venv activated\",\"data\":{\"python\":\"$(which python3)\",\"pip\":\"$(which pip)\"},\"timestamp\":$(date +%s%3N)}" >> /home/severin/.cursor/debug.log
    # #endregion
else
    # #region agent log
    echo "{\"sessionId\":\"debug-session\",\"runId\":\"startup\",\"hypothesisId\":\"D\",\"location\":\"start_server.sh:18\",\"message\":\"Venv not found\",\"data\":{\"path\":\"web_dashboard_env/bin/activate\"},\"timestamp\":$(date +%s%3N)}" >> /home/severin/.cursor/debug.log
    # #endregion
fi

# #region agent log
echo "{\"sessionId\":\"debug-session\",\"runId\":\"startup\",\"hypothesisId\":\"B\",\"location\":\"start_server.sh:22\",\"message\":\"About to start Python server\",\"data\":{\"python_cmd\":\"python3 -m app.main\",\"python_path\":\"$(which python3)\"},\"timestamp\":$(date +%s%3N)}" >> /home/severin/.cursor/debug.log
# #endregion

# Start FastAPI server
python3 -m app.main 2>&1 | while IFS= read -r line; do
    # #region agent log
    echo "{\"sessionId\":\"debug-session\",\"runId\":\"startup\",\"hypothesisId\":\"B\",\"location\":\"start_server.sh:26\",\"message\":\"Python output\",\"data\":{\"line\":\"$line\"},\"timestamp\":$(date +%s%3N)}" >> /home/severin/.cursor/debug.log
    # #endregion
    echo "$line"
done


