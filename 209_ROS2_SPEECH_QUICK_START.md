# R2D2 Speech System - ROS2 Quick Start

**Last Updated:** December 17, 2025  
**Status:** ✅ WORKING

---

## Quick Launch

```bash
cd ~/dev/r2d2
./launch_ros2_speech.sh
```

**That's it!** The node will auto-configure and auto-activate.

---

## Monitor Transcripts

**Terminal 2 - User Speech:**
```bash
source /opt/ros/humble/setup.bash
source ~/dev/r2d2/ros2_ws/install/setup.bash
ros2 topic echo /r2d2/speech/user_transcript
```

**Terminal 3 - Assistant Responses:**
```bash
source /opt/ros/humble/setup.bash
source ~/dev/r2d2/ros2_ws/install/setup.bash
ros2 topic echo /r2d2/speech/assistant_transcript
```

---

## Test It

1. Launch node (Terminal 1)
2. Monitor transcripts (Terminal 2 & 3)
3. **Speak into microphone:** "Hello, can you hear me?"
4. **Watch:** Transcripts appear, audio plays

---

## Check Status

```bash
ros2 lifecycle get /speech_node
# Should show: active [3]
```

---

## Control

```bash
# Stop streaming
ros2 lifecycle set /speech_node deactivate

# Restart streaming
ros2 lifecycle set /speech_node activate

# Stop session
ros2 service call /r2d2/speech/stop_session std_srvs/Trigger

# Start session
ros2 service call /r2d2/speech/start_session std_srvs/Trigger
```

---

## Change Assistant Personality

```bash
ros2 topic pub --once /r2d2/speech/assistant_prompt std_msgs/String \
  "data: 'You are R2D2, a helpful astromech droid from Star Wars.'"
```

Then speak - assistant will respond as R2D2!

---

## View Conversation History

All conversations are automatically saved to a SQLite database.

**Quick View:**
```bash
# Show recent messages
~/dev/r2d2/view_conversations.sh recent

# Show today's messages
~/dev/r2d2/view_conversations.sh today

# Show statistics
~/dev/r2d2/view_conversations.sh stats

# List all sessions
~/dev/r2d2/view_conversations.sh sessions

# Open interactive SQL shell
~/dev/r2d2/view_conversations.sh sql
```

**Database Location:**
```
/home/severin/dev/r2d2/r2d2_speech/data/conversations.db
```

**Schema:**
- `sessions` table: Session metadata (session_id, started_at, ended_at)
- `messages` table: All conversations (role, text, created_at, session_id)

**Manual SQL:**
```bash
sqlite3 /home/severin/dev/r2d2/r2d2_speech/data/conversations.db

# Recent messages
SELECT role, text FROM messages ORDER BY id DESC LIMIT 10;

# Today's conversations
SELECT role, text, created_at FROM messages 
WHERE date(created_at) = date('now') ORDER BY created_at ASC;
```

---

## Troubleshooting

### Node won't start
```bash
# Check API key
cat ~/.r2d2/.env | grep OPENAI_API_KEY

# Test microphone
source ~/dev/r2d2/r2d2_speech_env/bin/activate
python -m r2d2_speech.test_mic_level
```

### No transcripts
```bash
# Check topics exist
ros2 topic list | grep speech

# Check topic is publishing
ros2 topic hz /r2d2/speech/user_transcript
```

### Rebuild if needed
```bash
cd ~/dev/r2d2/ros2_ws
rm -rf build/r2d2_speech install/r2d2_speech
source /opt/ros/humble/setup.bash
colcon build --packages-select r2d2_speech --symlink-install
```

---

## Full Documentation

- **Implementation:** [208_SUBTASK_3_ROS2_INTEGRATION_COMPLETE.md](208_SUBTASK_3_ROS2_INTEGRATION_COMPLETE.md)
- **Testing Guide:** [ROS2_SPEECH_TESTING.md](ROS2_SPEECH_TESTING.md)
- **Existing System:** Subtask 1 & 2 (see git history)

---

## Architecture

```
Your Speech
    ↓
Microphone (HyperX QuadCast S)
    ↓
ROS2 Speech Node (Lifecycle)
    ↓
OpenAI Realtime API
    ↓
ROS2 Topics (/r2d2/speech/*)
    ↓
Database + Speakers
```

---

## Performance

- **Latency:** ~700-1200ms (speech → response)
- **CPU:** ~10-15% (single core)
- **Memory:** ~150MB
- **Quality:** High (Whisper-1 transcription, GPT-4o responses)

---

🎉 **Enjoy your ROS2-integrated speech system!**

