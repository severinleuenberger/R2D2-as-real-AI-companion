# R2D2 Speech System - Quick Start
## Fast Reference for Daily Use

**Last Updated:** December 17, 2025  
**Hardware:** HyperX QuadCast S USB + PAM8403 Speaker  
**API:** OpenAI Realtime (GPT-4o + Whisper-1)

---

## Quick Launch

```bash
cd ~/dev/r2d2
./launch_ros2_speech.sh
```

**That's it!** Node auto-configures and activates.

---

## Monitor Conversations

**Terminal 2 - User Speech:**
```bash
source /opt/ros/humble/setup.bash
source ~/dev/r2d2/ros2_ws/install/setup.bash
ros2 topic echo /r2d2/speech/user_transcript
```

**Terminal 3 - AI Responses:**
```bash
source /opt/ros/humble/setup.bash
source ~/dev/r2d2/ros2_ws/install/setup.bash
ros2 topic echo /r2d2/speech/assistant_transcript
```

---

## Test It

1. Launch node (Terminal 1)
2. Monitor topics (Terminal 2 & 3)
3. **Speak:** "Hello, can you hear me?"
4. **Watch:** Transcripts appear, hear AI response

---

## Common Commands

### Check Node Status
```bash
# Check lifecycle state
ros2 lifecycle get /speech_node
# Should show: active [3]

# List all ROS2 nodes
ros2 node list | grep speech

# List speech topics
ros2 topic list | grep speech
```

### Control Node
```bash
# Stop streaming (pause)
ros2 lifecycle set /speech_node deactivate

# Resume streaming
ros2 lifecycle set /speech_node activate

# Stop session
ros2 service call /r2d2/speech/stop_session std_srvs/Trigger

# Start new session
ros2 service call /r2d2/speech/start_session std_srvs/Trigger
```

### Change AI Personality
```bash
# Update system instructions
ros2 topic pub --once /r2d2/speech/assistant_prompt std_msgs/String \
  "data: 'You are R2D2, a helpful astromech droid from Star Wars. Be playful and use beep-boop sounds.'"

# Then speak - AI will respond as R2D2!
```

---

## Gesture Control (Auto-Start)

The speech system can be controlled via hand gestures when gesture recognition is enabled.

**Service Status:**
```bash
# Check gesture intent service
sudo systemctl status r2d2-gesture-intent.service

# View logs
sudo journalctl -u r2d2-gesture-intent.service -f
```

**Gesture Triggers:**
- 👆 **Index finger up** → Start speech session
- ✊ **Fist** → Stop speech session
- 🚶 **Walk away >35s** → Auto-shutdown (watchdog)

**Audio Feedback:**
- 🔊 Beep on speech start
- 🔊 Beep on speech stop
- 🔊 Beep on auto-shutdown

**Requirements:**
- Target person must be recognized (RED status)
- Gestures trained for the person
- Gesture intent service running (auto-starts on boot)

**For training gestures, see:** [303_GESTURE_TRAINING_GUIDE.md](303_GESTURE_TRAINING_GUIDE.md)

---

## View Conversation History

All conversations saved to SQLite database automatically.

**Database Location:**
```
/home/severin/dev/r2d2/r2d2_speech/data/conversations.db
```

**Quick View Recent Messages:**
```bash
sqlite3 /home/severin/dev/r2d2/r2d2_speech/data/conversations.db << 'EOF'
SELECT role, text, datetime(created_at, 'localtime') as time 
FROM messages 
ORDER BY id DESC 
LIMIT 10;
EOF
```

**View Today's Conversations:**
```bash
sqlite3 /home/severin/dev/r2d2/r2d2_speech/data/conversations.db << 'EOF'
SELECT role, text, datetime(created_at, 'localtime') as time 
FROM messages 
WHERE date(created_at) = date('now', 'localtime')
ORDER BY created_at ASC;
EOF
```

**Count Total Messages:**
```bash
sqlite3 /home/severin/dev/r2d2/r2d2_speech/data/conversations.db << 'EOF'
SELECT 
    COUNT(*) as total_messages,
    SUM(CASE WHEN role='user' THEN 1 ELSE 0 END) as user_messages,
    SUM(CASE WHEN role='assistant' THEN 1 ELSE 0 END) as assistant_messages
FROM messages;
EOF
```

**Interactive SQL Shell:**
```bash
sqlite3 /home/severin/dev/r2d2/r2d2_speech/data/conversations.db
# Type SQL queries interactively
# Exit with: .quit
```

---

## Troubleshooting

### Node Won't Start

```bash
# Check API key
cat ~/.r2d2/.env | grep OPENAI_API_KEY
# Should show: OPENAI_API_KEY=sk-...

# Test microphone
lsusb | grep -i hyperx
# Should show HyperX device

# Check virtualenv
source ~/dev/r2d2/r2d2_speech_env/bin/activate
python3 -c "import openai; print('✓ openai installed')"
```

### No Transcripts

```bash
# Check topics exist
ros2 topic list | grep speech
# Should show:
#   /r2d2/speech/user_transcript
#   /r2d2/speech/assistant_transcript
#   /r2d2/speech/session_status

# Check topic is publishing
ros2 topic hz /r2d2/speech/user_transcript
# Speak into mic - should show ~1-2 Hz

# Check node is active
ros2 lifecycle get /speech_node
# Should show: active [3]
```

### No Audio Output

```bash
# Test speaker directly
speaker-test -t wav -c 2
# Press Ctrl+C to stop

# If no sound, check PAM8403 setup (Phase 1)
```

### Rebuild if Needed

```bash
cd ~/dev/r2d2/ros2_ws
rm -rf build/r2d2_speech install/r2d2_speech
source /opt/ros/humble/setup.bash
colcon build --packages-select r2d2_speech --symlink-install
source install/setup.bash
```

---

## Performance Metrics

| Metric | Value |
|--------|-------|
| **Latency** | 700-1200ms (speech → response) |
| **CPU** | 10-15% (single core) |
| **Memory** | ~150MB |
| **GPU** | 0% (all processing on OpenAI) |
| **Network** | ~400-1000 Kbps (during conversation) |

---

## Hardware Info

### HyperX QuadCast S USB Microphone

**Connection:** USB (plug-and-play)  
**Sample Rate:** 48000 Hz native → resampled to 24000 Hz for API  
**Channels:** 2 (stereo) → converted to mono for API  
**Auto-Detection:** Searches for "hyperx" or "quadcast" in device name

**Manual Configuration (if auto-detection fails):**
```bash
# Edit ~/.r2d2/.env
nano ~/.r2d2/.env

# Add device index (find with: python3 -c "import pyaudio...")
MIC_DEVICE=2
```

### PAM8403 Speaker

**Connection:** J511 Audio Header (from Phase 1)  
**Sample Rate:** 24000 Hz (matches API output)  
**Configuration:** Handled by Phase 1 audio setup

---

## Configuration Files

### API Key & Settings
```bash
# Location
~/.r2d2/.env

# Required settings
OPENAI_API_KEY=sk-...           # Your API key

# Optional settings (have defaults)
REALTIME_MODEL=gpt-4o-realtime-preview-2024-12-17
REALTIME_VOICE=sage             # Options: alloy, echo, fable, onyx, nova, shimmer, sage
MIC_DEVICE=                     # Empty = auto-detect HyperX
DB_PATH=/home/severin/dev/r2d2/r2d2_speech/data/conversations.db
```

### ROS2 Parameters
```bash
# Location
~/dev/r2d2/ros2_ws/src/r2d2_speech/config/speech_params.yaml

# Can also set via command line:
ros2 param set /speech_node realtime_voice "echo"
```

---

## Launch Options

### Standard Launch
```bash
./launch_ros2_speech.sh
```

### Launch with Custom Voice
```bash
# Edit ~/.r2d2/.env and change:
REALTIME_VOICE=echo  # More energetic voice
# Or: alloy, fable, onyx, nova, shimmer, sage
```

### Launch with Custom Instructions
```bash
# Edit ~/.r2d2/.env and add:
INSTRUCTIONS="You are R2D2, speak like a droid with beeps and boops"
```

---

## Integration with R2D2 System

### Architecture
```
User Speech
    ↓
HyperX USB Mic
    ↓
Speech Node (ROS2 Lifecycle)
    ↓
OpenAI Realtime API
    ↓
ROS2 Topics:
  - /r2d2/speech/user_transcript
  - /r2d2/speech/assistant_transcript
    ↓
Database + Speaker
```

### Topics for Other Nodes
Other R2D2 nodes can subscribe to:
- `/r2d2/speech/user_transcript`: Get user commands
- `/r2d2/speech/assistant_transcript`: Get AI responses
- `/r2d2/speech/session_status`: Monitor session state

Example subscriber:
```python
import rclpy
from std_msgs.msg import String

def callback(msg):
    print(f"User said: {msg.data}")

node = rclpy.create_node('my_node')
sub = node.create_subscription(String, '/r2d2/speech/user_transcript', callback, 10)
rclpy.spin(node)
```

---

## Tips & Best Practices

### For Best Audio Quality
- Position HyperX microphone 6-12 inches from mouth
- Speak clearly and at normal volume
- Minimize background noise
- Use cardioid polar pattern (front-facing)

### For Natural Conversations
- Wait for silence before speaking (500ms pause detection)
- Speak complete sentences
- Avoid interrupting AI mid-response

### For System Performance
- Close other resource-heavy applications
- Monitor with: `htop` or `tegrastats`
- Keep network latency low (good WiFi/Ethernet)

### For Debugging
- Check logs: `ros2 topic echo /rosout`
- Monitor topics: `ros2 topic hz <topic>`
- View lifecycle: `ros2 lifecycle get /speech_node`

---

## Cost Estimates

**OpenAI Realtime API Pricing (as of Dec 2024):**
- ~$0.01-0.05 per conversation turn
- Free tier: Limited requests per day
- Paid tier: Pay-per-use (add credits to account)

**Typical Usage:**
- 10 conversations/day: ~$0.30-1.50/day
- 100 conversations/day: ~$3.00-15.00/day

---

## Aliases for Convenience

Add to `~/.bashrc`:
```bash
# R2D2 Speech aliases
alias r2d2-speech='~/dev/r2d2/launch_ros2_speech.sh'
alias r2d2-speech-logs='ros2 topic echo /rosout | grep speech'
alias r2d2-speech-status='ros2 lifecycle get /speech_node'
alias r2d2-speech-history='sqlite3 ~/dev/r2d2/r2d2_speech/data/conversations.db "SELECT role, text, datetime(created_at, \"localtime\") FROM messages ORDER BY id DESC LIMIT 20;"'
```

Then use:
```bash
r2d2-speech              # Launch
r2d2-speech-status       # Check status
r2d2-speech-history      # View recent conversations
```

---

## Full Documentation

- **Reference:** [200_SPEECH_SYSTEM_REFERENCE.md](200_SPEECH_SYSTEM_REFERENCE.md) - Complete architecture
- **Installation:** [201_SPEECH_SYSTEM_INSTALLATION.md](201_SPEECH_SYSTEM_INSTALLATION.md) - Setup guide
- **Testing:** [ROS2_SPEECH_TESTING.md](ROS2_SPEECH_TESTING.md) - Test procedures
- **Architecture:** [001_ARCHITECTURE_OVERVIEW.md](001_ARCHITECTURE_OVERVIEW.md) - System overview

---

## Support Resources

- **OpenAI API Docs:** https://platform.openai.com/docs/guides/realtime
- **OpenAI Status:** https://status.openai.com/
- **ROS2 Lifecycle:** https://design.ros2.org/articles/node_lifecycle.html
- **HyperX Support:** https://www.hyperxgaming.com/support

---

🎉 **Happy Conversing!**

---

**Quick Start Version:** 1.0  
**Last Updated:** December 17, 2025  
**Hardware:** HyperX QuadCast S USB Microphone  
**Status:** Production-ready

