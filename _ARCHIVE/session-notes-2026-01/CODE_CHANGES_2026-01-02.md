# Code Changes - January 2, 2026

## Summary
Fixed critical bugs in VAD timeout and fist gesture detection. Core functionality works but audio beeps remain unsolved.

---

## Files Modified

### 1. `ros2_ws/src/r2d2_speech/r2d2_speech_ros/speech_node.py`

**Changes:**
- **Line 28:** Added `ROS2VADPublisher` to imports
  ```python
  from .ros2_bridge import ROS2TranscriptHandler, ROS2StatusPublisher, ROS2VADPublisher, ros2_params_to_config
  ```

- **Line 65:** Declared vad_publisher variable
  ```python
  self.vad_publisher: Optional[ROS2VADPublisher] = None
  ```

- **Line 114:** Created VAD publisher instance
  ```python
  self.status_publisher = ROS2StatusPublisher(self)
  self.vad_publisher = ROS2VADPublisher(self)
  ```

- **Line 270:** Passed VAD publisher to EventRouter
  ```python
  self.event_router = EventRouter(
      self.client, self.ros2_transcript_handler,
      audio_playback=self.audio_manager.playback,
      vad_publisher=self.vad_publisher)
  ```

- **Line 207:** Cleanup VAD publisher
  ```python
  self.vad_publisher = None
  ```

**Impact:** VAD timeout now works - sessions auto-stop after 30s of silence

---

### 2. `ros2_ws/src/r2d2_gesture/r2d2_gesture/gesture_intent_node.py`

**Changes:**

- **Line 64:** Reduced fist window from 1.5s to 1.0s
  ```python
  self.declare_parameter('fist_window_seconds', 1.0)
  ```

- **Line 65:** Reduced fist threshold from 10 to 2 detections
  ```python
  self.declare_parameter('fist_threshold', 2)
  ```

- **Line 111:** Added flag for fist release requirement
  ```python
  self.waiting_for_fist_release = False  # After Stage 1, wait for release before Stage 2
  ```

- **Line 433-435:** After Stage 1, require release
  ```python
  self.fist_stage = "warning_played"
  self.fist_detection_buffer = []
  self.waiting_for_fist_release = True  # Wait for user to release fist
  self.last_trigger_time = current_time
  self.get_logger().info('⏸️  Waiting for fist release (you can cancel by not making fist again)')
  ```

- **Line 445-449:** Block Stage 2 until fist released
  ```python
  # Skip if waiting for release (user must release fist first)
  if self.waiting_for_fist_release:
      self.get_logger().debug('Fist detected but waiting for release first')
      return
  ```

- **Line 472:** Reset release flag after Stage 2
  ```python
  self.waiting_for_fist_release = False
  ```

- **Line 513-516:** Detect fist release (any non-fist gesture)
  ```python
  # Detect fist release after Stage 1 (any non-fist gesture)
  if gesture_name != "fist" and self.waiting_for_fist_release:
      self.get_logger().info('✅ Fist released → Ready for Stage 2 (make fist again to stop)')
      self.waiting_for_fist_release = False
      self.fist_stage = "idle"  # Reset to idle, ready for new fist detection
  ```

- **Line 689:** Added debug logging for beep playback
  ```python
  self.get_logger().info(f'🔊 Playing: {audio_file.name}, vol={effective_volume:.3f}')
  ```

- **Line 691-697:** Simplified ffplay call (removed pan filter)
  ```python
  proc = subprocess.Popen(
      ['ffplay', '-nodisp', '-autoexit', '-loglevel', 'info', 
       '-af', f'volume={effective_volume}', str(audio_file)],
      stdout=subprocess.PIPE,
      stderr=subprocess.PIPE
  )
  self.get_logger().info(f'✓ ffplay PID: {proc.pid}')
  # Log any errors
  if proc.poll() is not None:
      stderr = proc.stderr.read().decode() if proc.stderr else ""
      self.get_logger().error(f'ffplay died immediately! Error: {stderr}')
  ```

**Impact:** Fist gesture now successfully stops sessions with two-stage confirmation

---

### 3. `ros2_ws/src/r2d2_gesture/launch/gesture_intent.launch.py`

**Changes:**

- **Line 88:** Changed VAD timeout from 60s to 30s
  ```python
  default_value='30.0',
  description='VAD-based silence timeout (Option 2: VAD-only approach, default: 30 seconds)'
  ```

- **Line 82-83:** Increased audio volume from 0.02 to 0.3
  ```python
  default_value='0.3',
  description='Audio volume for gesture beeps (0.0-1.0) - 30% volume'
  ```

**Impact:** Better UX timeout, louder beeps (though still inaudible - see Known Issues)

---

### 4. `ros2_ws/src/r2d2_perception/r2d2_perception/image_listener.py`

**Changes:**

- **Line 443-453:** Changed from "publish on change" to "publish continuously"
  ```python
  if gesture_name and confidence > self.gesture_confidence_threshold:
      # Publish gesture event continuously (for rolling window detection in gesture_intent_node)
      gesture_msg = String()
      gesture_msg.data = gesture_name
      self.gesture_event_publisher.publish(gesture_msg)
      
      # Log only on state change to avoid spam
      if gesture_name != self.last_gesture:
          self.get_logger().debug(f"Gesture detected: {gesture_name} (confidence: {confidence:.2f})")
          self.last_gesture = gesture_name
  else:
      # Clear last gesture if no valid gesture detected
      self.last_gesture = None
  ```

**Impact:** Rolling window detection now works - gestures accumulate properly

---

### 5. `/etc/systemd/system/r2d2-camera-perception.service`

**Changes:**

- **ExecStart line:** Added `gesture_frame_skip:=1` parameter
  ```bash
  ExecStart=/bin/bash -c 'source install/setup.bash && ros2 launch r2d2_bringup r2d2_camera_perception.launch.py enable_face_recognition:=true enable_gesture_recognition:=true gesture_recognition_model_path:=/home/severin/dev/r2d2/data/gesture_recognition/models/severin_gesture_classifier.pkl gesture_frame_skip:=1'
  ```

**Impact:** Gestures detected every frame instead of every 2nd frame

---

## Working Features

✅ **VAD timeout** - Sessions auto-stop after 30s of silence  
✅ **Fist gesture detection** - Continuous, steady detection  
✅ **Two-stage fist stop** - Release required between Stage 1 and Stage 2  
✅ **Session stopping** - Fist gesture successfully terminates sessions  
✅ **Speech service** - Conversations work, transcripts saved

---

## Known Issues

❌ **Audio beeps not audible**
- Beeps ARE triggered (logs show "🔊 Playing")
- ffplay processes start but become zombie (defunct)
- Likely environment/PulseAudio/permissions issue with subprocess
- Status beeps (red/blue) work fine, gesture beeps don't
- Needs further investigation

**Workaround:** Gestures still work without beeps, just no audio feedback

---

## Parameters Summary

| Parameter | Old Value | New Value | Reason |
|-----------|-----------|-----------|--------|
| vad_silence_timeout | 60.0s | 30.0s | Better UX |
| fist_window | 1.5s | 1.0s | Easier triggering |
| fist_threshold | 10 | 2 | Works with detection flicker |
| audio_volume | 0.02 (2%) | 0.3 (30%) | Louder beeps (not working yet) |
| gesture_frame_skip | 2 | 1 | Continuous detection |
| gesture publish | On change | Continuous | Rolling window works |

---

**Status:** Working but incomplete  
**Next:** Debug ffplay zombie process issue  
**Commit:** In-progress solution - core functionality works, beeps pending

