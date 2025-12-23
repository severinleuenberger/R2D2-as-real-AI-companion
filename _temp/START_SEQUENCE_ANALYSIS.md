# R2D2 Speech Start Sequence Analysis
**Date:** December 23, 2025
**Source Status:** RED (Target Person Authorized)
**Target Action:** index_finger_up -> Speech Service "connected"

## 1. Detailed Sequence Diagram

```mermaid
sequenceDiagram
    participant Cam as OAK-D Lite (30 FPS)
    participant Perc as r2d2_perception (ImageListener)
    participant Gest as r2d2_gesture (GestureIntentNode)
    participant Speech as r2d2_speech (SpeechNode)
    participant OpenAI as OpenAI Realtime API

    Note over Cam, OpenAI: PHASE 1: Perception & Detection
    Cam->>Perc: [33ms] Raw RGB Frame (1920x1080)
    Note right of Perc: Filter: Sampling every 5th frame (166ms max delay)
    Note right of Perc: Process: MediaPipe Hand Landmarks (~50-70ms)
    Note right of Perc: Process: SVM Classifier (~5ms)
    Perc->>Gest: [2ms] /r2d2/perception/gesture_event ("index_finger_up")

    Note over Cam, OpenAI: PHASE 2: Gating & Service Call
    Note right of Gest: Gate: person_status == "red" (Checked)
    Note right of Gest: Gate: start_cooldown > 2.0s (Checked)
    Note right of Gest: Trigger: Voicy_R2-D2 - 12.mp3 (IMMEDIATE)
    Note right of Gest: ffplay: Acknowledgment Beep (~200ms)
    Gest->>Speech: [2ms] /r2d2/speech/start_session (Service Call)

    Note over Cam, OpenAI: PHASE 3: Session Initialization (THE BOTTLENECK)
    Note right of Speech: SQLite: Create Session Entry (~10ms)
    Speech->>OpenAI: [1000-2000ms] WebSocket Handshake (TLS/Auth)
    OpenAI-->>Speech: 101 Switching Protocols
    Speech->>OpenAI: [50ms] session.update (Instructions/VAD/Voice)
    
    Note over Cam, OpenAI: PHASE 4: Hardware & Feedback
    Note right of Speech: Audio: Init PyAudio Mic/Speaker (~100-300ms)
    Speech->>Gest: [2ms] /r2d2/speech/session_status ("connected")
    Note right of Gest: Trigger: Voicy_R2-D2 - 16.mp3 (READY)
    Note right of Gest: ffplay: System Ready Beep (~200ms process lag)
    Note over Cam, OpenAI: SYSTEM READY FOR SPEECH
```

## 2. Dependency & Delay Breakdown

| Component | Dependency / Clock | Logic / Filter | Delay (Typical) | Delay (Worst) |
| :--- | :--- | :--- | :--- | :--- |
| **Camera** | 30 Hz Hardware Clock | Exposure & Sensor Readout | 33ms | 33ms |
| **Perception** | `gesture_frame_skip` | Processes every 2nd frame (Proposed) | 33ms | 66ms |
| **MediaPipe** | CPU (Jetson Orin) | Hand Landmark Extraction (21 points) | 50ms | 80ms |
| **Gating** | ROS 2 Subscription | `person_status == "red"` | <1ms | <1ms |
| **Feedback 1** | **Gesture Detect** | **Voicy_R2-D2 - 12.mp3** (ffplay) | **~200ms** | **~400ms** |
| **Network** | TCP/TLS Handshake | **OpenAI WebSocket Connect** | **1200ms** | **3000ms** |
| **API Init** | `session.update` | Remote JSON config acknowledgment | 50ms | 200ms |
| **Hardware** | HyperX / PAM8403 | PyAudio stream initialization | 150ms | 400ms |
| **Feedback 2** | `session_status` | **Voicy_R2-D2 - 16.mp3** (ffplay) | **~200ms** | **~400ms** |

### Total Path Latency (with optimizations)
*   **Minimum (Perfect conditions):** ~0.5 seconds (warm start)
*   **Maximum (Typical load):** ~1.2 seconds (warm start)
*   **Cold Start (first gesture after boot):** ~2.5 seconds

## 3. Critical Observations and Optimizations

### ✅ Implemented Optimizations

1.  **Warm Start Connection (Major):** OpenAI WebSocket is now established during node activation, removing ~1.5s from the gesture-to-start path.
2.  **Faster Sampling (100ms saved):** Reduced `gesture_frame_skip` from 5 to 2, cutting sampling lag from 166ms to 66ms.
3.  **Dual-Beep Feedback (UX):** Two-stage acknowledgment system provides immediate gesture confirmation:
    *   `Voicy_R2-D2 - 12.mp3`: Plays immediately when gesture detected (~200ms)
    *   `Voicy_R2-D2 - 16.mp3`: Plays when system fully ready (~400ms later)

### Expected User Experience
*   **Gesture Detection:** ~150ms (user sees their hand, makes gesture)
*   **Immediate Beep:** ~350ms total (you hear "I saw it!")
*   **System Ready:** ~750ms total (you hear "Ready to talk!")
*   **First Response:** ~1.2s from gesture (much faster than previous ~3-4s)

