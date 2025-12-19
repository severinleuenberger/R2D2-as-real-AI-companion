# 🔍 Speaker Issue - Root Cause Found!

## Summary
The speech system **generates audio perfectly** but **never plays it**. The caller must explicitly play the audio manually. Meanwhile, the notification system that produces beeps works perfectly because it calls the audio player explicitly.

---

## What Was Working (Beeps)
**File:** `ros2_ws/src/r2d2_audio/r2d2_audio/audio_player.py`

✅ **Working flow:**
```
Face Recognition Event → Audio Notification Node → Calls _play_audio_file() → Calls audio_player.py → subprocess.Popen(ffplay/mpv/aplay)
```

The notification system **explicitly calls playback** using subprocess:
```python
def _play_audio_file(self, audio_file: Path, alert_type: str = "GENERIC"):
    """Play an audio file using the audio player."""
    subprocess.Popen(
        cmd,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        start_new_session=True
    )
```

---

## What's Broken (TTS Speaker)
**File:** `r2d2_speech/speech_pipeline.py` - `process_speech()` method (line 223)

❌ **Broken flow:**
```
process_speech() → Generates audio array → Returns it → NOTHING PLAYS
```

**The problem code (lines 223-290):**
```python
def process_speech(
    self,
    audio_input: str,
    input_language: str = "de",
    output_language: str = "de",
) -> Dict[str, any]:
    """Complete speech pipeline: transcribe → process → synthesize."""
    
    # ... STT, LLM, TTS all work great ...
    
    return {
        "input_text": input_text,
        "response_text": response_text,
        "response_audio": response_audio,  # ← Audio generated but NOT PLAYED
        "timings": timings,
        "total_time": total_time,
        "llm_provider": self.llm.get_provider(),
    }
    # ← No playback call!
```

---

## Why the Speaker Seems Broken
1. **TTS module has a `.play()` method** (lines 218-260 in `r2d2_speech/tts/tts_wrapper.py`)
   - It takes numpy array and plays via `aplay` subprocess ✓
   - Code is correct and should work

2. **But it's NEVER CALLED** from the speech pipeline
   - `process_speech()` generates audio and returns it
   - The caller must manually call `tts.play()` on the returned audio

3. **So the audio sits silent:**
   - Generated ✓
   - Returned ✓
   - Discarded ✗

---

## The Fix (Two Options)

### Option A: Add Playback to Pipeline (Recommended)
**File:** `r2d2_speech/speech_pipeline.py`, method `process_speech()` (after line 281)

```python
            # Step 4: Play audio
            logger.info("Step 4: Playing response audio...")
            try:
                self.tts.play(response_audio, sample_rate=22050)
                logger.info("✓ Audio playback started")
            except Exception as e:
                logger.error(f"Failed to play audio: {e}")
                # Continue - don't crash if speaker is broken
```

**Why this is best:**
- ✅ Matches notification system pattern (explicit playback)
- ✅ Self-contained pipeline
- ✅ No changes needed to callers
- ✅ Consistent with "process_speech" semantics

### Option B: Use audio_player.py Like Notifications (Fallback Compatible)
Instead of TTS's `.play()`, use the same `audio_player.py` that beeps use:

```python
            # Save to temporary WAV file
            temp_audio_path = Path("/tmp/r2d2_response.wav")
            response_audio_path = self.tts.save(response_text, language=output_language)
            
            # Use notification system's audio player (works with ffplay/mpv/aplay)
            from ros2_ws.src.r2d2_audio.r2d2_audio.audio_player import play_audio
            play_audio(str(response_audio_path), volume=0.8)
```

**Advantages:**
- ✅ Reuses proven working code
- ✅ Multiple player fallback (ffplay → mpv → aplay)
- ✅ Better volume control

---

## Additional Issue Found

**File:** `r2d2_speech/tts/tts_wrapper.py`, line 256

The `.play()` method uses `aplay -` (stdin pipe). This might fail if:
1. Device permissions issue
2. ALSA not configured for Jetson audio routing
3. PAM8403 speaker not initialized

**Better approach:** Use the proven `audio_player.py` that has fallback players.

---

## Quick Diagnosis Steps

### 1. Verify TTS audio generation works:
```bash
cd /home/severin/dev/r2d2
python3 -c "
from r2d2_speech.tts import PiperTTS
tts = PiperTTS()
audio = tts.synthesize('Hoi, test!', language='de')
print(f'Audio generated: {audio.shape} samples')
"
```

### 2. Verify aplay works with test file:
```bash
# Create test audio
ffmpeg -f lavfi -i sine=f=1000:d=1 /tmp/test.wav -y

# Try to play it
aplay /tmp/test.wav
```

### 3. Check if audio notification service is blocking device:
```bash
ps aux | grep audio
fuser /dev/snd/*
```

---

## Recommended Action Plan

**Step 1 (5 min):** Add playback to `process_speech()` using TTS's `.play()` method  
**Step 2 (5 min):** Test with small German phrase  
**Step 3 (10 min):** If aplay fails, implement fallback using `audio_player.py`  
**Step 4 (Optional):** Check if audio notification service conflicts  

---

## Files Involved

| File | Purpose | Status |
|------|---------|--------|
| `r2d2_speech/speech_pipeline.py` | **Main culprit** - generates audio but doesn't play | ❌ Broken |
| `r2d2_speech/tts/tts_wrapper.py` | Has `.play()` method, works | ✓ Working |
| `ros2_ws/src/r2d2_audio/r2d2_audio/audio_player.py` | Fallback audio player | ✓ Working |
| `ros2_ws/src/r2d2_audio/r2d2_audio/audio_notification_node.py` | Notification audio (beeps) | ✓ Working |

---

## Expected Outcome After Fix

```
Before: User speaks → TTS generates → Nothing heard ❌
After:  User speaks → TTS generates → Plays audio immediately ✅
```

**Impact:** Full end-to-end speech conversation will work!

---

**Status:** Root cause identified, solution ready to implement  
**Difficulty:** 5 minutes (add 5 lines of code)  
**Risk:** Very low (adding explicit playback call, no side effects)
