#!/bin/bash
# R2D2 Speech System - ROS2 Testing Guide
# This script helps you test the speech system step by step

echo "=========================================="
echo "R2D2 Speech System - Testing Guide"
echo "=========================================="
echo ""

# Colors
GREEN='\033[0;32m'
BLUE='\033[0;34m'
YELLOW='\033[1;33m'
NC='\033[0m'

# Function to wait for user
wait_for_user() {
    echo ""
    echo -e "${YELLOW}Press Enter to continue...${NC}"
    read
}

# Check prerequisites
echo "Checking prerequisites..."
echo ""

if [ ! -f ~/.r2d2/.env ]; then
    echo -e "${YELLOW}⚠ WARNING: ~/.r2d2/.env not found${NC}"
    echo "The speech system needs an OpenAI API key."
    echo "Create ~/.r2d2/.env with:"
    echo "  OPENAI_API_KEY=sk-..."
    wait_for_user
fi

# Step 1: Setup environment
echo ""
echo "=========================================="
echo "STEP 1: Setup Environment"
echo "=========================================="
echo ""
echo "Setting up ROS2 and Python environments..."
echo ""

# This script should be sourced, but we'll show the commands
echo -e "${BLUE}Commands to run:${NC}"
echo "  source /opt/ros/humble/setup.bash"
echo "  source ~/dev/r2d2/ros2_ws/install/setup.bash"
echo "  source ~/dev/r2d2/r2d2_speech_env/bin/activate"
echo ""

# Step 2: Launch node
echo "=========================================="
echo "STEP 2: Launch the Speech Node"
echo "=========================================="
echo ""
echo "This will launch the ROS2 lifecycle node."
echo "The node will:"
echo "  1. Auto-configure (load parameters)"
echo "  2. Auto-activate (start speech system)"
echo "  3. Connect to OpenAI API"
echo "  4. Start audio streaming"
echo ""
echo -e "${BLUE}Command to run in Terminal 1:${NC}"
echo "  ros2 launch r2d2_speech speech_node.launch.py"
echo ""
echo "Expected output:"
echo "  - 'Configuring...'"
echo "  - 'Database: ...'"
echo "  - 'Configuration complete'"
echo "  - 'Activating...'"
echo "  - 'Connected to API'"
echo "  - 'Speech system running'"
echo ""
wait_for_user

# Step 3: Check lifecycle state
echo "=========================================="
echo "STEP 3: Check Node State"
echo "=========================================="
echo ""
echo -e "${BLUE}Command to run in Terminal 2:${NC}"
echo "  ros2 lifecycle get /speech_node"
echo ""
echo "Expected output:"
echo "  active [3]"
echo ""
echo "This confirms the node is running."
echo ""
wait_for_user

# Step 4: Monitor topics
echo "=========================================="
echo "STEP 4: Monitor Transcripts"
echo "=========================================="
echo ""
echo "Now let's monitor the transcript topics."
echo ""
echo -e "${BLUE}Commands to run in separate terminals:${NC}"
echo ""
echo "Terminal 3 (User transcripts):"
echo "  ros2 topic echo /r2d2/speech/user_transcript"
echo ""
echo "Terminal 4 (Assistant transcripts):"
echo "  ros2 topic echo /r2d2/speech/assistant_transcript"
echo ""
echo "Terminal 5 (Session status):"
echo "  ros2 topic echo /r2d2/speech/session_status"
echo ""
wait_for_user

# Step 5: Test speech
echo "=========================================="
echo "STEP 5: Test Speech Recognition"
echo "=========================================="
echo ""
echo "Now speak into your microphone!"
echo ""
echo "Try saying:"
echo "  'Hello, can you hear me?'"
echo ""
echo "You should see:"
echo "  1. In Terminal 1: Log messages about speech detection"
echo "  2. In Terminal 3: Your transcript appears"
echo "  3. In Terminal 4: Assistant's response appears"
echo "  4. Audio plays through your speakers"
echo ""
wait_for_user

# Step 6: Test lifecycle control
echo "=========================================="
echo "STEP 6: Test Lifecycle Control"
echo "=========================================="
echo ""
echo "You can control the node's lifecycle manually."
echo ""
echo -e "${BLUE}Commands to try:${NC}"
echo ""
echo "Deactivate (stop streaming):"
echo "  ros2 lifecycle set /speech_node deactivate"
echo ""
echo "Activate (restart streaming):"
echo "  ros2 lifecycle set /speech_node activate"
echo ""
echo "Check state:"
echo "  ros2 lifecycle get /speech_node"
echo ""
wait_for_user

# Step 7: Test services
echo "=========================================="
echo "STEP 7: Test Services"
echo "=========================================="
echo ""
echo -e "${BLUE}Available services:${NC}"
echo ""
echo "Stop session:"
echo "  ros2 service call /r2d2/speech/stop_session std_srvs/Trigger"
echo ""
echo "Start session:"
echo "  ros2 service call /r2d2/speech/start_session std_srvs/Trigger"
echo ""
wait_for_user

# Step 8: Test prompt update
echo "=========================================="
echo "STEP 8: Update Assistant Prompt"
echo "=========================================="
echo ""
echo "You can change the assistant's personality on the fly."
echo ""
echo -e "${BLUE}Example:${NC}"
echo '  ros2 topic pub --once /r2d2/speech/assistant_prompt std_msgs/String "data: '\''You are R2D2, a helpful astromech droid.'\'' "'
echo ""
echo "After this, the assistant should respond as R2D2!"
echo ""
wait_for_user

# Step 9: View database
echo "=========================================="
echo "STEP 9: Check Database"
echo "=========================================="
echo ""
echo "All conversations are saved to the database."
echo ""
echo -e "${BLUE}Location:${NC}"
echo "  ~/dev/r2d2/r2d2_speech/data/conversations.db"
echo ""
echo -e "${BLUE}View with sqlite3:${NC}"
echo "  sqlite3 ~/dev/r2d2/r2d2_speech/data/conversations.db \"SELECT * FROM messages ORDER BY id DESC LIMIT 10;\""
echo ""
wait_for_user

# Step 10: Troubleshooting
echo "=========================================="
echo "TROUBLESHOOTING"
echo "=========================================="
echo ""
echo "If something doesn't work:"
echo ""
echo "1. Check API key:"
echo "   cat ~/.r2d2/.env | grep OPENAI_API_KEY"
echo ""
echo "2. Check microphone:"
echo "   python3 -c 'import pyaudio; p=pyaudio.PyAudio(); [print(f\"{i}: {p.get_device_info_by_index(i)[\\\"name\\\"]}\") for i in range(p.get_device_count())]'"
echo ""
echo "3. Check node logs:"
echo "   Look at Terminal 1 output for errors"
echo ""
echo "4. Check topics are publishing:"
echo "   ros2 topic list"
echo "   ros2 topic hz /r2d2/speech/user_transcript"
echo ""
echo "5. Run existing diagnostic tests:"
echo "   source ~/dev/r2d2/r2d2_speech_env/bin/activate"
echo "   cd ~/dev/r2d2"
echo "   python -m r2d2_speech.test_mic_level"
echo "   python -m r2d2_speech.test_vad_diagnostic"
echo ""

echo "=========================================="
echo "Quick Reference"
echo "=========================================="
echo ""
echo "Launch node:"
echo "  ros2 launch r2d2_speech speech_node.launch.py"
echo ""
echo "Monitor transcripts:"
echo "  ros2 topic echo /r2d2/speech/user_transcript"
echo "  ros2 topic echo /r2d2/speech/assistant_transcript"
echo ""
echo "Check status:"
echo "  ros2 lifecycle get /speech_node"
echo "  ros2 topic echo /r2d2/speech/session_status"
echo ""
echo "Control:"
echo "  ros2 lifecycle set /speech_node [configure|activate|deactivate]"
echo "  ros2 service call /r2d2/speech/stop_session std_srvs/Trigger"
echo ""
echo "=========================================="
echo -e "${GREEN}Ready to test!${NC}"
echo "=========================================="
echo ""


