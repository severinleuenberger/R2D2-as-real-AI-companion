# R2D2 Speech System - ROS2 Testing Guide

## Quick Start

### 1. Launch the Node (Terminal 1)

```bash
cd ~/dev/r2d2
./launch_ros2_speech.sh
```

**Expected output:**
```
R2D2 Speech Node initializing...
Configuring...
Database: /home/severin/dev/r2d2/r2d2_speech/data/conversations.db
Configuration complete
Activating...
Session: ros2-20251217-HHMMSS
Connected to API
Session created
Audio: HyperX QuadCast S
Playback started
✓ Speech system running
Activation complete
```

### 2. Monitor User Transcripts (Terminal 2)

```bash
# Source ROS2 first
source /opt/ros/humble/setup.bash
source ~/dev/r2d2/ros2_ws/install/setup.bash

# Monitor user speech
ros2 topic echo /r2d2/speech/user_transcript
```

### 3. Monitor Assistant Transcripts (Terminal 3)

```bash
# Source ROS2
source /opt/ros/humble/setup.bash
source ~/dev/r2d2/ros2_ws/install/setup.bash

# Monitor assistant responses
ros2 topic echo /r2d2/speech/assistant_transcript
```

### 4. Speak Into Microphone

Say something like:
- "Hello, can you hear me?"
- "What's the weather like today?"
- "Tell me a joke"

**You should see:**
1. Terminal 1: Log messages about speech detection
2. Terminal 2: Your transcript appears
3. Terminal 3: Assistant's response appears
4. Audio plays through your speakers

---

## Lifecycle Control

### Check Current State

```bash
ros2 lifecycle get /speech_node
```

Expected: `active [3]`

### Manual Control

```bash
# Deactivate (stop streaming)
ros2 lifecycle set /speech_node deactivate

# Activate (restart streaming)
ros2 lifecycle set /speech_node activate

# Full reset
ros2 lifecycle set /speech_node deactivate
ros2 lifecycle set /speech_node cleanup
ros2 lifecycle set /speech_node configure
ros2 lifecycle set /speech_node activate
```

---

## Services

### Stop Session

```bash
ros2 service call /r2d2/speech/stop_session std_srvs/Trigger
```

### Start Session

```bash
ros2 service call /r2d2/speech/start_session std_srvs/Trigger
```

---

## Update Assistant Prompt

Change the assistant's personality:

```bash
ros2 topic pub --once /r2d2/speech/assistant_prompt std_msgs/String \
  "data: 'You are R2D2, a helpful astromech droid from Star Wars.'"
```

Now speak again - the assistant should respond as R2D2!

---

## Check Session Status

```bash
ros2 topic echo /r2d2/speech/session_status
```

Output (JSON):
```json
{
  "status": "connected",
  "session_id": "ros2-20251217-HHMMSS",
  "timestamp": 1702842000
}
```

---

## View Database

All conversations are saved:

```bash
# View recent messages
sqlite3 ~/dev/r2d2/r2d2_speech/data/conversations.db \
  "SELECT role, text, created_at FROM messages ORDER BY id DESC LIMIT 10;"
```

---

## Troubleshooting

### Node Won't Start

1. **Check API key:**
   ```bash
   cat ~/.r2d2/.env | grep OPENAI_API_KEY
   ```
   Should show: `OPENAI_API_KEY=sk-...`

2. **Check Python environment:**
   ```bash
   source ~/dev/r2d2/r2d2_speech_env/bin/activate
   python -c "from r2d2_speech.config import get_config; print('OK')"
   ```

3. **Rebuild package:**
   ```bash
   cd ~/dev/r2d2/ros2_ws
   colcon build --packages-select r2d2_speech --symlink-install
   source install/setup.bash
   ```

### No Transcripts Appearing

1. **Check microphone:**
   ```bash
   source ~/dev/r2d2/r2d2_speech_env/bin/activate
   cd ~/dev/r2d2
   python -m r2d2_speech.test_mic_level
   ```

2. **Check topics are active:**
   ```bash
   ros2 topic list
   ros2 topic hz /r2d2/speech/user_transcript
   ```

3. **Run diagnostic:**
   ```bash
   source ~/dev/r2d2/r2d2_speech_env/bin/activate
   cd ~/dev/r2d2
   python -m r2d2_speech.test_vad_diagnostic
   ```

### Audio Not Playing

1. **Check playback device:**
   ```bash
   aplay -l  # List devices
   ```

2. **Test with existing system:**
   ```bash
   source ~/dev/r2d2/r2d2_speech_env/bin/activate
   cd ~/dev/r2d2
   python -m r2d2_speech.test_realtime_with_mic
   ```

---

## Advanced Testing

### Custom Parameters

```bash
# Disable auto-start
ros2 launch r2d2_speech speech_node.launch.py auto_start:=false

# Custom instructions
ros2 launch r2d2_speech speech_node.launch.py \
  instructions:="You are a pirate assistant. Speak like a pirate!"

# Specific microphone
ros2 launch r2d2_speech speech_node.launch.py mic_device:=24
```

### Monitor All Topics

```bash
# List all speech topics
ros2 topic list | grep speech

# Info about topic
ros2 topic info /r2d2/speech/user_transcript

# Watch message rate
ros2 topic hz /r2d2/speech/user_transcript
```

### Service Discovery

```bash
# List services
ros2 service list | grep speech

# Service type
ros2 service type /r2d2/speech/start_session
```

---

## Test Scenarios

### Scenario 1: Basic Conversation

1. Launch node: `./launch_ros2_speech.sh`
2. Monitor transcripts: `ros2 topic echo /r2d2/speech/user_transcript`
3. Say: "Hello, how are you?"
4. Verify: Transcript appears, assistant responds, audio plays

### Scenario 2: Personality Change

1. Update prompt: `ros2 topic pub --once /r2d2/speech/assistant_prompt ...`
2. Say something
3. Verify: Assistant responds with new personality

### Scenario 3: Session Restart

1. Stop: `ros2 service call /r2d2/speech/stop_session ...`
2. Verify: Status changes to "disconnected"
3. Start: `ros2 service call /r2d2/speech/start_session ...`
4. Verify: New session created, can speak again

### Scenario 4: Lifecycle Control

1. Deactivate: `ros2 lifecycle set /speech_node deactivate`
2. Verify: Streaming stops
3. Activate: `ros2 lifecycle set /speech_node activate`
4. Verify: Streaming resumes

---

## Expected Performance

- **Latency:** ~700-1200ms from speech to response
- **CPU:** ~10-15% (single core)
- **Memory:** ~150MB
- **Audio quality:** Clear, no distortion
- **Transcript accuracy:** High (Whisper-1 model)

---

## Success Criteria

✅ Node launches without errors  
✅ Auto-configures and auto-activates  
✅ Connects to OpenAI API  
✅ Captures audio from microphone  
✅ Publishes user transcripts to topic  
✅ Publishes assistant transcripts to topic  
✅ Plays assistant audio through speakers  
✅ Saves messages to database  
✅ Responds to lifecycle commands  
✅ Responds to service calls  
✅ Updates on prompt changes  

---

## Next Steps After Testing

1. **System Integration:** Add to r2d2_bringup launch file
2. **Systemd Service:** Enable auto-start on boot
3. **Higher-Level Behaviors:** Integrate with other ROS2 nodes
4. **Monitoring:** Add ROS2 diagnostics

---

## Documentation

- Package README: `~/dev/r2d2/ros2_ws/src/r2d2_speech/README.md`
- Subtasks 1 & 2: `~/dev/r2d2/207_SUBTASKS_1_2_COMPLETE.md`
- Full test guide: `~/dev/r2d2/test_ros2_speech.sh` (interactive)

---

## Quick Commands Reference

```bash
# Launch
./launch_ros2_speech.sh

# Monitor
ros2 topic echo /r2d2/speech/user_transcript
ros2 topic echo /r2d2/speech/assistant_transcript

# Status
ros2 lifecycle get /speech_node
ros2 topic echo /r2d2/speech/session_status

# Control
ros2 lifecycle set /speech_node [configure|activate|deactivate]
ros2 service call /r2d2/speech/stop_session std_srvs/Trigger
ros2 service call /r2d2/speech/start_session std_srvs/Trigger

# Update prompt
ros2 topic pub --once /r2d2/speech/assistant_prompt std_msgs/String "data: 'New instructions'"

# Database
sqlite3 ~/dev/r2d2/r2d2_speech/data/conversations.db "SELECT * FROM messages LIMIT 10;"
```

---

🎉 **Ready to test!** Start with Terminal 1 (launch), Terminal 2 (monitor), then speak!

