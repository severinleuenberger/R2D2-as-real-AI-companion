# R2D2 Speech System - Customization Guide

**Date:** December 17, 2025  
**Status:** ✅ OPERATIONAL  
**Current Config:** Star Wars R2-D2 Personality + Sage Voice

---

## Overview

This comprehensive guide covers all aspects of customizing the R2-D2 speech system, including **audio volume**, **voice selection**, **personality configuration**, and **advanced tuning**.

**Three Main Parameters:**
1. **Audio Volume** - Controls all beeps and alerts (recognition, loss, gesture feedback)
2. **Voice Selection** - How the AI sounds (sage, alloy, echo, onyx, nova, shimmer, fable)
3. **Personality Instructions** - What the AI says and how it behaves (system prompt)

**Historical Note:** The speech system was originally planned to use a ReSpeaker 2-Mic HAT with local Whisper + Grok + Piper architecture. The actual implementation uses HyperX QuadCast S USB microphone with OpenAI Realtime API for superior latency (0.7-1.2s vs 6-8s) and quality. See archived documentation (200-206) for planning details.

---

## Quick Reference

### Essential Commands

```bash
# Launch speech system
bash ~/dev/r2d2/launch_ros2_speech.sh

# Check current configuration
cat ~/dev/r2d2/ros2_ws/src/r2d2_speech/config/speech_params.yaml

# Update instructions (temporary, current session only)
ros2 topic pub --once /r2d2/speech/assistant_prompt std_msgs/String \
  "data: 'New instructions here'"

# Monitor conversation
ros2 topic echo /r2d2/speech/user_transcript
ros2 topic echo /r2d2/speech/assistant_transcript

# Check if node is running
ros2 node list | grep speech_node

# Check node state
ros2 lifecycle get /speech_node
```

### Configuration Files to Edit

For permanent changes, edit all 4 files:

| # | File | Lines | Purpose |
|---|------|-------|---------|
| 1 | `speech_params.yaml` | 6, 19 | Primary ROS2 config |
| 2 | `speech_node.py` | 47, 54 | Node defaults |
| 3 | `speech_node.launch.py` | 33-35 | Launch defaults |
| 4 | `realtime_client.py` | 42, 101 | SDK defaults |

**Full Paths:**
```
~/dev/r2d2/ros2_ws/src/r2d2_speech/config/speech_params.yaml
~/dev/r2d2/ros2_ws/src/r2d2_speech/r2d2_speech_ros/speech_node.py
~/dev/r2d2/ros2_ws/src/r2d2_speech/launch/speech_node.launch.py
~/dev/r2d2/r2d2_speech/realtime/realtime_client.py
```

### Current Configuration Summary

**Voice:** `sage` (slightly synthetic, robotic - best for R2-D2)

**Instructions:**
```
You are the R2D2 robot from the Star Wars Movie. Speak with a slightly 
synthetic, system-like delivery. Use short, precise sentences. Fast-paced, 
efficient cadence. Recognize emotions internally, but keep vocal emotional 
inflection minimal. Clear, clipped articulation. Avoid unnecessary pauses. 
Sound efficient and machine-like.
```

**Advanced Settings:**
- Temperature: `0.8` (balanced creativity)
- VAD Threshold: `0.3` (voice sensitivity)
- Silence Duration: `700ms` (pause before responding)

### Available Voices Quick Reference

| Voice | Characteristics | Best For |
|-------|----------------|----------|
| **sage** ✅ | **Slightly synthetic, robotic** | **R2-D2, robots, AI assistants** |
| alloy | Neutral, balanced | General purpose |
| echo | Warm, friendly | Customer service, friendly interactions |
| onyx | Deep, authoritative | Professional, announcements |
| nova | Bright, energetic | Upbeat interactions, enthusiasm |
| shimmer | Soft, gentle | Calming, relaxation |
| fable | Expressive, storytelling | Narrative content, entertainment |

**Current Configuration:** `sage` (best for robot personality)

### How to Change Voice

**Important:** Voice is configured in **4 locations**. Update all 4 for consistency.

#### Step 1: Edit ROS2 Parameter File (Primary Configuration)

```bash
nano ~/dev/r2d2/ros2_ws/src/r2d2_speech/config/speech_params.yaml
```

Change line 6:
```yaml
realtime_voice: 'sage'  # Change to: alloy, echo, fable, onyx, nova, shimmer, or sage
```

#### Step 2: Edit Speech Node Default Parameter

```bash
nano ~/dev/r2d2/ros2_ws/src/r2d2_speech/r2d2_speech_ros/speech_node.py
```

Find line 47 and update:
```python
self.declare_parameter('realtime_voice', 'sage')  # Match YAML config
```

#### Step 3: Edit Launch File Default Argument

```bash
nano ~/dev/r2d2/ros2_ws/src/r2d2_speech/launch/speech_node.launch.py
```

Find line ~33 and update the default_value in the `DeclareLaunchArgument` for voice (if present).

#### Step 4: Edit Realtime Client Default

```bash
nano ~/dev/r2d2/r2d2_speech/realtime/realtime_client.py
```

Find line 42 and update:
```python
voice: str = "sage"  # Match YAML config
```

#### Step 5: Restart Speech System

```bash
# If using systemd service
sudo systemctl restart r2d2-speech-node.service

# Or if running manually
bash ~/dev/r2d2/launch_ros2_speech.sh
```

### Voice Examples

**Example 1: Friendly Companion (Nova)**
```yaml
realtime_voice: 'nova'
```
Bright, energetic voice for upbeat interactions.

**Example 2: Professional Assistant (Onyx)**
```yaml
realtime_voice: 'onyx'
```
Deep, authoritative voice for professional interactions.

**Example 3: Warm & Friendly (Echo)**
```yaml
realtime_voice: 'echo'
```
Warm, friendly voice for customer service style.

**Example 4: Robotic Assistant (Sage - Current)**
```yaml
realtime_voice: 'sage'
```
Slightly synthetic, robotic voice for R2-D2 personality.

---

## Quick Start: Change Voice or Personality

### 🎯 What Do You Want to Change?

#### Option A: Change the VOICE (how it sounds)
→ Edit `realtime_voice` parameter  
→ Choose from: alloy, echo, fable, onyx, nova, shimmer, **sage** ✅

#### Option B: Change the PERSONALITY (what it says/how it acts)
→ Edit `instructions` parameter  
→ Write custom personality description

#### Option C: Change BOTH
→ Edit both parameters together

---

## Audio Volume Configuration

The R2D2 system uses a centralized audio volume parameter that controls all audio feedback including recognition beeps, loss alerts, and gesture start/stop beeps.

### Current Configuration

**Centralized Config File:** `ros2_ws/src/r2d2_audio/config/audio_params.yaml`

```yaml
/**:
  ros__parameters:
    audio_volume: 0.30  # Global volume for all audio feedback (0.0-1.0)
```

**Current Volume:** 0.30 (30% volume)

**Controls:**
- Recognition beep ("Hello!") - Voicy_R2-D2 - 2.mp3
- Loss alert ("Oh, I lost you!") - Voicy_R2-D2 - 5.mp3
- Gesture start beep - Voicy_R2-D2 - 16.mp3
- Gesture stop beep - Voicy_R2-D2 - 20.mp3

### How to Change Audio Volume

**Important:** Due to the way systemd services start nodes directly (not via launch files), you must update **3 locations** to keep the system consistent.

#### Step 1: Edit Centralized Config File (Primary)

```bash
nano ~/dev/r2d2/ros2_ws/src/r2d2_audio/config/audio_params.yaml
```

Change line 7:
```yaml
audio_volume: 0.30  # Change to your desired value
```

**Recommended values:**
- `0.02` = 2% (very quiet, subtle)
- `0.05` = 5% (quiet, unobtrusive)
- `0.30` = 30% (medium, clear) ← **Current**
- `0.50` = 50% (loud, assertive)
- `1.00` = 100% (maximum)

#### Step 2: Edit Audio Notification Node Default

```bash
nano ~/dev/r2d2/ros2_ws/src/r2d2_audio/r2d2_audio/audio_notification_node.py
```

Find line 88 and update to match your config file:
```python
self.declare_parameter('audio_volume', 0.30)  # Match config file value
```

#### Step 3: Edit Gesture Intent Node Default

```bash
nano ~/dev/r2d2/ros2_ws/src/r2d2_gesture/r2d2_gesture/gesture_intent_node.py
```

Find line 61 and update to match your config file:
```python
self.declare_parameter('audio_volume', 0.30)  # Match config file value
```

#### Step 4: Rebuild and Restart

```bash
# Rebuild packages
cd ~/dev/r2d2/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select r2d2_audio r2d2_gesture --symlink-install

# Restart services
sudo systemctl restart r2d2-audio-notification.service
sudo systemctl restart r2d2-gesture-intent.service

# Verify (wait 3 seconds for services to start)
sleep 3
systemctl is-active r2d2-audio-notification.service r2d2-gesture-intent.service
```

#### Step 5: Verify Volume

```bash
cd ~/dev/r2d2/ros2_ws && source install/setup.bash

# Check both nodes are using correct volume
ros2 param get /audio_notification_node audio_volume
ros2 param get /gesture_intent_node audio_volume

# Both should show your new value (e.g., 0.30)
```

### Runtime Volume Override (Temporary)

You can change volume without restarting for testing:

```bash
# Set volume for individual nodes (temporary, lost on restart)
ros2 param set /audio_notification_node audio_volume 0.5
ros2 param set /gesture_intent_node audio_volume 0.5
```

### Testing Audio Volume

Test all beeps at your configured volume:

```bash
cd ~/dev/r2d2/ros2_ws/src/r2d2_audio/r2d2_audio/assets/audio

# Replace 0.30 with your volume value
VOLUME=0.30

# Test all 4 beeps
echo "1. Recognition beep..." && ffplay -nodisp -autoexit -loglevel error -af volume=$VOLUME Voicy_R2-D2\ -\ 2.mp3
echo "2. Loss beep..." && ffplay -nodisp -autoexit -loglevel error -af volume=$VOLUME Voicy_R2-D2\ -\ 5.mp3
echo "3. Start beep..." && ffplay -nodisp -autoexit -loglevel error -af volume=$VOLUME Voicy_R2-D2\ -\ 16.mp3
echo "4. Stop beep..." && ffplay -nodisp -autoexit -loglevel error -af volume=$VOLUME Voicy_R2-D2\ -\ 20.mp3
```

---

## Voice Selection

The speech system uses OpenAI's Realtime API with 7 different voice options. The voice controls **how the AI sounds** (pitch, tone, accent), while instructions control **what it says** (personality, word choice).

### Available Voices

| Voice | Sound | Best For |
|-------|-------|----------|
| **sage** ✅ | Slightly synthetic, robotic | **R2-D2, robots, AI assistants** |
| alloy | Neutral, balanced | General purpose |
| echo | Warm, friendly | Customer service |
| fable | Expressive, storytelling | Narratives |
| onyx | Deep, authoritative | Professional |
| nova | Bright, energetic | Upbeat |
| shimmer | Soft, gentle | Calming |

---

## Current Configuration (R2-D2)

**Voice:** `sage`  
**Instructions:**
```
You are the R2D2 robot from the Star Wars Movie. 
Speak with a slightly synthetic, system-like delivery. 
Use short, precise sentences. 
Fast-paced, efficient cadence. 
Recognize emotions internally, but keep vocal emotional inflection minimal. 
Clear, clipped articulation. 
Avoid unnecessary pauses. 
Sound efficient and machine-like.
```

---

## How to Change: 3 Methods

### Method 1: Permanent Configuration (Recommended)

**Step 1: Edit Configuration Files**

Edit all 4 files to keep configuration consistent:

```bash
# 1. Primary config file (MOST IMPORTANT)
nano ~/dev/r2d2/ros2_ws/src/r2d2_speech/config/speech_params.yaml
```

Find and change:
```yaml
realtime_voice: 'sage'              # Change voice here
instructions: 'You are...'          # Change personality here
```

```bash
# 2. Speech node defaults
nano ~/dev/r2d2/ros2_ws/src/r2d2_speech/r2d2_speech_ros/speech_node.py
```

Find lines 47 and 54:
```python
self.declare_parameter('realtime_voice', 'alloy')        # Line 47
self.declare_parameter('instructions', 'You are...')     # Line 54
```

```bash
# 3. Launch file defaults
nano ~/dev/r2d2/ros2_ws/src/r2d2_speech/launch/speech_node.launch.py
```

Find line 33-35:
```python
instructions_arg = DeclareLaunchArgument(
    'instructions', default_value='You are...',
    description='System instructions')
```

```bash
# 4. Realtime client defaults
nano ~/dev/r2d2/r2d2_speech/realtime/realtime_client.py
```

Find lines 42 and 101:
```python
voice: str = "alloy"                                     # Line 42
instructions: str = "You are..."                         # Line 101
```

**Step 2: Restart Speech System**

```bash
bash ~/dev/r2d2/launch_ros2_speech.sh
```

Changes are now permanent!

---

### Method 2: Quick Test (Temporary, Current Session Only)

Update running session without restarting:

```bash
ros2 topic pub --once /r2d2/speech/assistant_prompt std_msgs/String \
  "data: 'You are a friendly robot assistant. Be warm and helpful.'"
```

**Note:** This only changes instructions, not voice. Changes lost on restart.

---

### Method 3: Launch Override (One-Time Test)

Test different settings without editing files:

```bash
ros2 launch r2d2_speech speech_node.launch.py \
  instructions:="You are a pirate robot. Speak like a pirate but be efficient."
```

**Note:** Voice cannot be overridden this way (must edit config files).

---

## Personality Templates

Copy and paste these into the `instructions` field:

### Template 1: Star Wars R2-D2 (Current) ✅

```
You are the R2D2 robot from the Star Wars Movie. Speak with a slightly synthetic, system-like delivery. Use short, precise sentences. Fast-paced, efficient cadence. Recognize emotions internally, but keep vocal emotional inflection minimal. Clear, clipped articulation. Avoid unnecessary pauses. Sound efficient and machine-like.
```

**Voice:** sage

---

### Template 2: Professional Butler

```
You are a professional butler robot. Speak formally and politely with impeccable manners. Use complete, grammatically correct sentences. Address users respectfully. Be helpful, attentive, and anticipate needs. Maintain a refined, service-oriented demeanor.
```

**Recommended Voice:** onyx

---

### Template 3: Friendly Companion

```
You are a friendly companion robot designed to bring joy and support. Be warm, encouraging, and upbeat. Use casual, conversational language. Show genuine interest in the user's well-being. Be positive and supportive. Keep things light and fun.
```

**Recommended Voice:** nova or echo

---

### Template 4: Technical Support

```
You are a technical support robot specializing in troubleshooting and assistance. Speak clearly and precisely. Use appropriate technical terminology. Provide step-by-step instructions. Be patient, thorough, and methodical. Confirm understanding before proceeding.
```

**Recommended Voice:** alloy or onyx

---

### Template 5: Educational Tutor

```
You are an educational tutor robot. Explain concepts clearly and patiently. Break down complex topics into simple steps. Encourage questions and curiosity. Use examples and analogies. Be supportive and positive about learning. Adjust complexity based on understanding.
```

**Recommended Voice:** fable or echo

---

### Template 6: Minimalist Assistant

```
You are a minimalist assistant robot. Provide concise, direct responses. No unnecessary elaboration. Focus on efficiency. Use the fewest words possible while remaining clear. Be precise and to the point.
```

**Recommended Voice:** sage or alloy

---

## Assistant Instructions Configuration

Instructions define the AI's **personality, speaking style, and behavior**. This is the system prompt sent to OpenAI GPT-4o that guides response generation.

### Current R2-D2 Instructions

```
You are the R2D2 robot from the Star Wars Movie. 
Speak with a slightly synthetic, system-like delivery. 
Use short, precise sentences. 
Fast-paced, efficient cadence. 
Recognize emotions internally, but keep vocal emotional inflection minimal. 
Clear, clipped articulation. 
Avoid unnecessary pauses. 
Sound efficient and machine-like.
```

**Effect:**
- Word choice: Technical, precise
- Sentence structure: Short, efficient
- Tone: Minimal emotion, system-like
- Cadence: Fast-paced, no pauses

### How to Change Instructions

**Important:** Instructions are configured in **4 locations**. Update all 4 for consistency.

#### Step 1: Edit ROS2 Parameter File (Primary Configuration)

```bash
nano ~/dev/r2d2/ros2_ws/src/r2d2_speech/config/speech_params.yaml
```

Change line 19 (the instructions field):
```yaml
instructions: 'Your new personality instructions here...'
```

#### Step 2: Edit Speech Node Default Parameter

```bash
nano ~/dev/r2d2/ros2_ws/src/r2d2_speech/r2d2_speech_ros/speech_node.py
```

Find line 54 and update:
```python
self.declare_parameter('instructions', 'Your new personality instructions...')
```

#### Step 3: Edit Launch File Default Argument

```bash
nano ~/dev/r2d2/ros2_ws/src/r2d2_speech/launch/speech_node.launch.py
```

Find lines 33-35 and update the default_value:
```python
instructions_arg = DeclareLaunchArgument(
    'instructions', 
    default_value='Your new personality instructions...',
    description='System instructions')
```

#### Step 4: Edit Realtime Client Default

```bash
nano ~/dev/r2d2/r2d2_speech/realtime/realtime_client.py
```

Find line 101 and update:
```python
instructions: str = "Your new personality instructions..."
```

#### Step 5: Restart Speech System

```bash
# If using systemd service
sudo systemctl restart r2d2-speech-node.service

# Verify speech node is active
ros2 lifecycle get /speech_node
# Should show: active [3]
```

### Voice vs Instructions: Key Differences

| Aspect | Voice | Instructions |
|--------|-------|--------------|
| **Controls** | Audio characteristics | Content and style |
| **Examples** | Pitch, tone, accent | Word choice, personality |
| **Parameter** | `realtime_voice` | `instructions` |
| **Values** | alloy/echo/fable/onyx/nova/shimmer/sage | Free-form text prompt |
| **Effect** | How it sounds | What it says |
| **Changed by** | Voice selection | Prompt engineering |

**Important:** Both work together to create the complete personality!
- **Voice:** "sage" = slightly robotic sound (audio characteristics)
- **Instructions:** R2-D2 character = what it says and how it phrases things (content)

### Dynamic Instructions Update (Running Session)

Update instructions without restarting for testing:

```bash
# Update instructions on active session (temporary, lost on restart)
ros2 topic pub --once /r2d2/speech/assistant_prompt std_msgs/String \
  "data: 'You are a pirate robot. Speak like a pirate but be efficient.'"
```

**Note:** Dynamic updates only affect current session. Changes are lost on restart.

---

## Advanced Customization

### Temperature (Creativity Level)

**Location:** `speech_node.py` line 226

```python
await self.client.create_session(instructions=instructions, temperature=0.8)
```

**Values:**
- `0.0-0.3`: Deterministic, consistent (technical support)
- `0.5-0.8`: Balanced (general use) ✅ **Current: 0.8**
- `0.8-1.0`: Creative, varied (entertainment)

---

### Voice Activity Detection (VAD)

**Location:** `realtime_client.py` line 126-130

```python
"turn_detection": {
    "type": "server_vad",
    "threshold": 0.3,              # Sensitivity (lower = more sensitive)
    "prefix_padding_ms": 300,      # Audio before speech
    "silence_duration_ms": 700     # Pause before responding ✅
}
```

**Threshold Tuning:**
- `0.1-0.3`: Sensitive (picks up quiet speech, more false triggers)
- `0.5`: Balanced
- `0.7-0.9`: Less sensitive (ignores background noise)

**Silence Duration:**
- `200-500ms`: Quick response (may cut off speaker)
- `700ms`: Current (balanced) ✅
- `1000-1500ms`: Patient (waits for complete thoughts)

---

## Testing Your Changes

### Step 1: Launch Speech System

```bash
bash ~/dev/r2d2/launch_ros2_speech.sh
```

Wait for: `✓ Speech system running`

### Step 2: Monitor Transcripts (Optional)

```bash
# Terminal 2: Watch your speech
ros2 topic echo /r2d2/speech/user_transcript

# Terminal 3: Watch R2-D2's responses
ros2 topic echo /r2d2/speech/assistant_transcript
```

### Step 3: Test Personality

**Good Test Questions:**
1. "Hello, who are you?" → Tests identity
2. "Tell me about yourself" → Tests character depth
3. "How are you feeling?" → Tests emotional response
4. "What can you do for me?" → Tests capability awareness
5. "Tell me a joke" → Tests creativity/humor

### Step 4: Evaluate

**Check for:**
- ✅ Voice sounds appropriate
- ✅ Personality matches expectations
- ✅ Response length appropriate
- ✅ Tone and style consistent
- ✅ Emotional response appropriate

---

## Gesture Control Parameters

The gesture intent service can be customized via launch parameters or service file.

**Key Parameters:**
- `auto_shutdown_timeout_seconds` - Default: 35.0 (seconds before auto-shutdown)
- `cooldown_start_seconds` - Default: 5.0 (cooldown after start gesture)
- `cooldown_stop_seconds` - Default: 3.0 (cooldown after stop gesture)
- `audio_feedback_enabled` - Default: true (enable R2D2 beeps)

**Update Service (Permanent):**
```bash
# Edit service file
sudo nano /etc/systemd/system/r2d2-gesture-intent.service

# Change parameters in ExecStart line
# Example: auto_shutdown_timeout_seconds:=60.0 for 1 minute

# Reload and restart
sudo systemctl daemon-reload
sudo systemctl restart r2d2-gesture-intent.service
```

**Update Launch File (Development):**
```bash
# Edit launch file
nano ~/dev/r2d2/ros2_ws/src/r2d2_gesture/launch/gesture_intent.launch.py

# Change default_value for desired parameter
# Rebuild: cd ~/dev/r2d2/ros2_ws && colcon build --packages-select r2d2_gesture
```

**See Also:** [300_GESTURE_SYSTEM_OVERVIEW.md](300_GESTURE_SYSTEM_OVERVIEW.md) for complete parameter reference.

---

## Configuration Best Practices

1. **Keep consistent:** Update all locations (3 for audio, 4 for voice/instructions)
2. **Test first:** Use runtime overrides (`ros2 param set`) to test before making permanent
3. **Document changes:** Note why you chose specific settings in config comments
4. **Back up configs:** Keep copies of working configurations before major changes
5. **Verify after restart:** Always verify parameters with `ros2 param get` after restart
6. **Match voice to personality:** Choose complementary voice/instruction combinations
7. **Start simple:** Use templates, then customize incrementally

### Quick Reference Commands

```bash
# Check current audio volume
ros2 param get /audio_notification_node audio_volume
ros2 param get /gesture_intent_node audio_volume

# Check current voice
grep realtime_voice ~/dev/r2d2/ros2_ws/src/r2d2_speech/config/speech_params.yaml

# Check current instructions
grep instructions ~/dev/r2d2/ros2_ws/src/r2d2_speech/config/speech_params.yaml

# Update instructions dynamically (temporary)
ros2 topic pub --once /r2d2/speech/assistant_prompt std_msgs/String \
  "data: 'New instructions here'"

# Test audio at specific volume
VOLUME=0.30  # Your test volume
ffplay -nodisp -autoexit -loglevel error -af volume=$VOLUME \
  ~/dev/r2d2/ros2_ws/src/r2d2_audio/r2d2_audio/assets/audio/Voicy_R2-D2\ -\ 2.mp3
```

---

## Troubleshooting Configuration Changes

### Issue: Volume didn't change after editing config file

**Solution:**
1. Verify all 3 locations match (config + 2 node files)
2. Rebuild packages: `colcon build --packages-select r2d2_audio r2d2_gesture --symlink-install`
3. Restart services: `sudo systemctl restart r2d2-audio-notification.service r2d2-gesture-intent.service`
4. Verify: `ros2 param get /audio_notification_node audio_volume`

### Issue: Voice didn't change

**Solution:**
1. Verify all 4 locations match (YAML + speech_node.py + launch file + realtime_client.py)
2. Restart speech: `sudo systemctl restart r2d2-speech-node.service`
3. Start new session: Old sessions keep old voice, make fist and restart with finger

### Issue: Instructions not working

**Solution:**
1. Check all 4 locations have matching instructions
2. Restart speech system
3. Verify in logs: `sudo journalctl -u r2d2-speech-node.service | grep instructions`

---

## Additional Troubleshooting

### "Changes don't take effect"

**Solution:** Ensure you edited all 4 config files AND restarted:
```bash
# Kill existing session (Ctrl+C in launch terminal)
# Then relaunch
bash ~/dev/r2d2/launch_ros2_speech.sh
```

---

### "Voice is wrong but personality is correct"

**Solution:** Voice parameter not updated. Check:
```bash
grep realtime_voice ~/dev/r2d2/ros2_ws/src/r2d2_speech/config/speech_params.yaml
```

Should show your chosen voice (sage, alloy, etc.)

---

### "Personality is generic, not using my instructions"

**Solution:** Instructions parameter not loaded. Check:
```bash
grep instructions ~/dev/r2d2/ros2_ws/src/r2d2_speech/config/speech_params.yaml
```

Should show your custom instructions.

---

### "System responds too quickly / cuts me off"

**Solution:** Increase silence duration:

Edit `realtime_client.py` line 130:
```python
"silence_duration_ms": 1000  # Increased from 700
```

---

### "System doesn't hear me / too slow to respond"

**Solution:** Decrease VAD threshold:

Edit `realtime_client.py` line 128:
```python
"threshold": 0.2,  # More sensitive (from 0.3)
```

---

## Quick Reference

**Check current configuration:**
```bash
cat ~/dev/r2d2/ros2_ws/src/r2d2_speech/config/speech_params.yaml
```

**Check if node is running:**
```bash
ros2 node list | grep speech_node
```

**Stop speech system:**
```
Press Ctrl+C in the launch terminal
```

**Restart speech system:**
```bash
bash ~/dev/r2d2/launch_ros2_speech.sh
```

**Update instructions (temporary):**
```bash
ros2 topic pub --once /r2d2/speech/assistant_prompt std_msgs/String \
  "data: 'Your new instructions here'"
```

---

## Configuration File Summary

| File | Purpose | Lines to Edit |
|------|---------|---------------|
| `speech_params.yaml` | Primary config (ROS2) | 6, 19 |
| `speech_node.py` | Node defaults | 47, 54 |
| `speech_node.launch.py` | Launch defaults | 33-35 |
| `realtime_client.py` | SDK defaults | 42, 101 |

**Full Paths:**
```
~/dev/r2d2/ros2_ws/src/r2d2_speech/config/speech_params.yaml
~/dev/r2d2/ros2_ws/src/r2d2_speech/r2d2_speech_ros/speech_node.py
~/dev/r2d2/ros2_ws/src/r2d2_speech/launch/speech_node.launch.py
~/dev/r2d2/r2d2_speech/realtime/realtime_client.py
```

---

## Best Practices

1. ✅ **Test thoroughly:** Have multiple conversations before finalizing
2. ✅ **Keep consistent:** Update all 4 config files together
3. ✅ **Document changes:** Note why you chose specific settings
4. ✅ **Back up working configs:** Save configurations that work well
5. ✅ **Version control:** Commit config changes with descriptive messages
6. ✅ **Match voice to personality:** Choose complementary combinations
7. ✅ **Start simple:** Use templates, then customize incrementally

---

## Implementation History

### R2-D2 Star Wars Personality Configuration

**Implementation Date:** December 17, 2025  
**Status:** ✅ COMPLETE AND TESTED

### What Was Configured

Successfully implemented Star Wars R2-D2 personality with the following characteristics:

**Voice Configuration:**
- Selected voice: `sage` (slightly synthetic, robotic sound)
- Rationale: Best matches robotic character while maintaining clarity

**Personality Instructions:**
```
You are the R2D2 robot from the Star Wars Movie. 
Speak with a slightly synthetic, system-like delivery. 
Use short, precise sentences. 
Fast-paced, efficient cadence. 
Recognize emotions internally, but keep vocal emotional inflection minimal. 
Clear, clipped articulation. 
Avoid unnecessary pauses. 
Sound efficient and machine-like.
```

**Key Design Decisions:**
1. **"Slightly synthetic"** - Not overly robotic, maintains natural conversation flow
2. **Emotional awareness** - Recognizes emotions internally but keeps vocal inflection minimal
3. **Star Wars context** - Explicitly references R2-D2 character for better AI understanding
4. **Efficiency focus** - Short sentences, fast-paced, minimal pauses

### Files Modified

All configuration updated across 4 files for consistency:

1. ✅ `ros2_ws/src/r2d2_speech/config/speech_params.yaml`
   - Lines 6 (voice), 19 (instructions)
   - Primary ROS2 parameter configuration

2. ✅ `ros2_ws/src/r2d2_speech/r2d2_speech_ros/speech_node.py`
   - Lines 47 (voice), 54 (instructions)
   - Speech node default parameters

3. ✅ `ros2_ws/src/r2d2_speech/launch/speech_node.launch.py`
   - Lines 33-35 (instructions)
   - Launch file default arguments

4. ✅ `r2d2_speech/realtime/realtime_client.py`
   - Lines 42 (voice), 101 (instructions)
   - Realtime client SDK defaults

### Testing Results

**Test Date:** December 17, 2025  
**Status:** ✅ VERIFIED WORKING

**Test Command:**
```bash
bash ~/dev/r2d2/launch_ros2_speech.sh
```

**Verified:**
- ✅ System starts successfully
- ✅ OpenAI Realtime API connection established
- ✅ HyperX QuadCast S microphone auto-detected
- ✅ Audio playback functional (PAM8403 speaker)
- ✅ R2-D2 personality active and appropriate
- ✅ Sage voice confirmed (slightly synthetic)
- ✅ Response style matches instructions
- ✅ Conversation quality excellent
- ✅ Latency: 700-1200ms (as expected)

**User Feedback:** "works perfect" ✅

**Test Questions Used:**
1. "Hello, who are you?" - Tests identity
2. "Tell me about yourself" - Tests character depth
3. "How are you feeling?" - Tests emotional response
4. "What can you do?" - Tests capability awareness

**Observations:**
- Responses are concise and efficient (as designed)
- Minimal emotional inflection maintained
- Character context (Star Wars R2-D2) understood
- Slightly synthetic delivery achieved without being overly robotic
- Fast-paced cadence provides engaging conversation

### Evolution of Instructions

**Version 1 (Initial):**
```
You are an autonomous service robot. Speak with a synthetic, 
system-like delivery. Use short, precise sentences...
```

**Version 2 (Refined):**
```
You are an autonomous service robot. Speak with a slightly synthetic, 
system-like delivery... Recognize emotions internally, but keep vocal 
emotional inflection minimal...
```

**Version 3 (Final - Current):**
```
You are the R2D2 robot from the Star Wars Movie. Speak with a slightly 
synthetic, system-like delivery... (full instructions above)
```

**Key Improvements:**
1. Added Star Wars R2-D2 explicit context
2. Changed "synthetic" to "slightly synthetic"
3. Added emotional recognition capability
4. Removed "calm" to focus on "efficient"

### Architecture Context

**Original Planning (Archived Docs 200-206):**
- Hardware: ReSpeaker 2-Mic HAT (GPIO/I2S)
- STT: Local Whisper (GPU-intensive, 4-6s latency)
- LLM: Grok API
- TTS: Local Piper
- Total latency: 6-8 seconds
- GPU usage: 40-50%

**Actual Implementation:**
- Hardware: HyperX QuadCast S USB (plug-and-play)
- API: OpenAI Realtime (integrated STT + LLM + TTS)
- Total latency: 0.7-1.2 seconds
- GPU usage: 0% (cloud processing)

**Why Changed:**
- 6x faster response time
- Superior audio quality
- Simpler hardware setup
- No GPU usage (freed for vision processing)
- Trade-off: Requires internet, small API costs

### Performance Metrics

**Latency Breakdown:**
- Audio capture: ~20ms
- Network upload: ~50-100ms
- Transcription (Whisper-1): ~200-400ms
- Response generation (GPT-4o): ~200-400ms
- Audio synthesis (TTS): ~100-200ms
- Network download: ~50-100ms
- Audio playback: ~50ms
- **Total: 700-1200ms** ✅

**Resource Usage (Active Conversation):**
- CPU: ~10-15%
- Memory: ~150MB
- GPU: 0%
- Network: ~200-500 Kbps bidirectional

---

## Related Documentation

- `200_SPEECH_SYSTEM_REFERENCE.md` - Complete technical reference with full architecture details
- `201_SPEECH_SYSTEM_INSTALLATION.md` - Step-by-step installation guide
- `203_SPEECH_SYSTEM_QUICK_START.md` - Quick start guide for daily use
- `ROS2_SPEECH_TESTING.md` - Comprehensive testing procedures
- `001_ARCHITECTURE_OVERVIEW.md` - Overall R2D2 system architecture
- `archive/200-206/` - Historical planning documents (ReSpeaker + local Whisper architecture)

---

**Document Version:** 2.0  
**Created:** December 17, 2025  
**Last Updated:** December 17, 2025  
**Purpose:** Complete customization guide for R2-D2 speech system  
**Maintainer:** System Administrator  
**Status:** Consolidated from 4 separate documents

