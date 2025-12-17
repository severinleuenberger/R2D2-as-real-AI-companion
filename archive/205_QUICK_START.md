# Phase 2 Premium Speech: Quick Start & Execution Summary

**Date:** December 9, 2025  
**Status:** READY TO BUILD  
**Total Time:** 3-4 hours  
**Difficulty:** Moderate (follow steps exactly)

---

## 📚 Documentation Files Created

You now have **4 comprehensive guides**:

| Document | Purpose | Time | Read First |
|----------|---------|------|-----------|
| **201_SPEECH_SWISS_GERMAN_PREMIUM.md** | Architecture & rationale | 20 min | YES |
| **202_INSTALLATION_GUIDE.md** | Step-by-step install | 90 min | FOLLOW |
| **203_TEST_SCRIPTS.md** | Test each component | 90 min | FOLLOW |
| **204_BUILD_GUIDE.md** | Full build walkthrough | 180 min | FOLLOW |

---

## 🚀 Quick Start: 4 Hours to Working System

### Hour 1: Installation (60 minutes)

**Follow:** `202_INSTALLATION_GUIDE.md` → Parts 1-8

```bash
# Summary of what you'll do:
1. Verify Jetson environment (5 min)
2. Create Python venv (5 min)
3. Install core dependencies (15 min)
4. Install PyTorch + CUDA (15 min)
5. Install speech libraries (5 min)
6. Download models: Whisper, Piper, Silero (45 min total)
7. Configure API keys: Groq + Porcupine (5 min)
8. Create project structure (5 min)

Total: ~90 minutes
```

**Key Commands:**
```bash
# Setup
python3 -m venv ~/dev/r2d2/r2d2_speech_env
source ~/dev/r2d2/r2d2_speech_env/bin/activate
pip install --upgrade pip

# Install everything
pip install faster-whisper piper-tts silero-vad groq porcupine \
  pyaudio librosa soundfile torch torchaudio

# Download models (LONGEST STEP - 45 min)
python3 << 'EOF'
from faster_whisper import WhisperModel
model = WhisperModel("large-v2", device="cuda", compute_type="float32")
EOF

piper --download-model de_DE-kerstin-high
piper --download-model en_US-libritts-high

# Configure API keys
nano ~/.r2d2/.env
# Add: GROQ_API_KEY="your_key"
# Add: PORCUPINE_ACCESS_KEY="your_key"
```

---

### Hour 2: Testing (60 minutes)

**Follow:** `203_TEST_SCRIPTS.md` → Tests 1-6

```bash
# Run tests in order (copy test code from document first)
cd ~/dev/r2d2/r2d2_speech

python3 tests/test_environment.py      # 5 min - verify setup
python3 tests/test_microphone.py       # 10 min - test ReSpeaker
python3 tests/test_stt.py --record     # 20 min - test Whisper
python3 tests/test_tts.py              # 10 min - test Piper
python3 tests/test_llm.py              # 10 min - test Grok API
python3 tests/test_end_to_end.py       # 20 min - test full pipeline

Total: ~90 minutes
```

**Expected Results:**
- ✅ Environment: All checks pass
- ✅ Microphone: Records and plays back
- ✅ STT: Transcribes speech correctly
- ✅ TTS: Synthesizes German/English speech
- ✅ LLM: Responds to queries in Swiss German
- ✅ End-to-End: Full conversational loop works!

---

### Hour 3: Build Components (60 minutes)

**Follow:** `204_BUILD_GUIDE.md` → Phase 2.3

Create 4 Python classes (copy from document):

1. `src/stt/whisper_wrapper.py` - STT wrapper (5 min copy)
2. `src/tts/piper_wrapper.py` - TTS wrapper (5 min copy)
3. `src/llm/grok_client.py` - LLM wrapper (5 min copy)
4. `src/speech_pipeline.py` - Main orchestrator (5 min copy)

```bash
# After creating files, test imports
python3 << 'EOF'
from r2d2_speech.src.speech_pipeline import SpeechPipeline
pipeline = SpeechPipeline()
print("✓ All components loaded successfully")
EOF
```

Total: ~20 minutes copying, ~40 minutes learning/tweaking

---

### Hour 4: ROS 2 Integration (60 minutes)

**Follow:** `204_BUILD_GUIDE.md` → Phases 2.4-2.6

```bash
# Create ROS 2 package
mkdir -p ~/dev/r2d2/ros2_ws/src/r2d2_speech_node

# Copy package.xml and r2d2_speech_node.py from document
# (takes 10 min to copy and understand)

# Build
cd ~/dev/r2d2/ros2_ws
colcon build --packages-select r2d2_speech_node
source install/setup.bash

# Test
ros2 run r2d2_speech_node r2d2_speech_node
# In another terminal:
ros2 topic pub /r2d2/speech/trigger std_msgs/String "data: 'trigger'" --once
```

Total: ~60 minutes (build + test)

---

## ✅ Verification Checklist

After all 4 hours, verify:

```bash
# 1. All tests pass
cd ~/dev/r2d2/r2d2_speech
python3 tests/test_end_to_end.py
# Expected: ✅ Pipeline complete

# 2. ROS 2 node runs
ros2 run r2d2_speech_node r2d2_speech_node
# Expected: "Speech Processing Node ready"

# 3. Command trigger works
ros2 topic pub /r2d2/speech/trigger std_msgs/String "data: trigger" --once
# Expected: Speech processes and publishes response

# 4. GPU performance OK
tegrastats
# Expected during STT: ~50% GPU, ~12% CPU

echo "✅ PHASE 2 COMPLETE!"
```

---

## 🎯 What You'll Have

After 4 hours:

```
R2D2 Speech System (PREMIUM)
├─ STT: Faster-Whisper large (97% accuracy, 4-6s)
├─ LLM: Grok API (Swiss German native)
├─ TTS: Piper kerstin-high (professional, 0.8-1.0s)
├─ ROS 2 Integration: Command publishing
└─ Full end-to-end pipeline: Record→STT→LLM→TTS→Play
```

**Quality:** Professional grade ✨  
**Speed:** 6-8 seconds per utterance (acceptable)  
**GPU Usage:** 40-50% peak (within 55% budget)  
**Language:** Swiss German + English  

---

## 📋 Document Reading Order

**For Initial Understanding:**
1. Read: `201_SPEECH_SWISS_GERMAN_PREMIUM.md` (20 min)
   - Understand the "why" behind each component
   - See performance metrics
   - Review cost analysis

**For Implementation:**
2. Follow: `202_INSTALLATION_GUIDE.md` (90 min)
   - Step-by-step installation
   - Troubleshooting if issues
   
3. Follow: `203_TEST_SCRIPTS.md` (90 min)
   - Copy each test script
   - Run tests in order
   - Verify each component works

4. Follow: `204_BUILD_GUIDE.md` (180 min)
   - Build Python classes
   - Create ROS 2 node
   - Final integration and testing

---

## 🔑 Key Points to Remember

### Installation
- **Do NOT skip:** PyTorch CUDA installation (use the exact URL provided)
- **Do NOT interrupt:** Model downloads (use screen/tmux)
- **Do configure:** API keys BEFORE testing

### Testing
- **Run tests sequentially** (test 1 → 2 → 3 etc.)
- **Use ReSpeaker audio** from test 2 for test 3 (saves time)
- **End-to-end test** (test 6) should record you speaking Swiss German

### Building
- **Copy exactly:** Use code from documents (formatting matters for imports)
- **Test imports:** After creating files, verify imports work
- **Build ROS 2:** Follow exact colcon build steps

### Integration
- **Use terminals carefully:** Open separate terminal windows
- **ROS 2 daemon:** Restart if issues: `ros2 daemon stop && ros2 daemon start`
- **Source setup.bash:** In each terminal: `source install/setup.bash`

---

## 🐛 Common Issues & Fixes

### "ModuleNotFoundError: No module named 'faster_whisper'"
```bash
# Solution: Activate venv or use full pip path
source ~/dev/r2d2/r2d2_speech_env/bin/activate
pip install faster-whisper
```

### "CUDA not available"
```bash
# Solution: Reinstall torch with CUDA
pip install torch torchaudio --force-reinstall --index-url https://download.pytorch.org/whl/cu121
```

### Model download times out
```bash
# Solution: Download manually in background
cd ~/.cache/huggingface
wget -P . https://huggingface.co/[model-path]/resolve/main/model.safetensors
```

### "piper: command not found"
```bash
# Solution: Reinstall piper
pip install piper-tts --force-reinstall
# Then: piper --download-model de_DE-kerstin-high
```

### No audio from speaker
```bash
# Solution: Test PAM8403 directly
ffmpeg -f lavfi -i sine=f=400:d=2 -f alsa hw:1,0
# Should hear tone. If not, check 050_AUDIO_SETUP.md
```

### ROS 2 node won't start
```bash
# Solution: Check sourcing
source /opt/ros/humble/setup.bash
source ~/dev/r2d2/ros2_ws/install/setup.bash
# Then: ros2 run r2d2_speech_node r2d2_speech_node
```

---

## 📞 When to Ask for Help

If you get stuck on:

1. **Installation issues** → Check `202_INSTALLATION_GUIDE.md` troubleshooting
2. **Test failures** → Show me which test fails (1-6) + error message
3. **ROS 2 integration** → Check sourcing steps in `204_BUILD_GUIDE.md`
4. **Audio not working** → Review `050_AUDIO_SETUP_AND_CONFIGURATION.md`

---

## 🎓 Learning Path (Optional)

To understand the system better:

1. **Watch:** Whisper demo video (5 min) - OpenAI docs
2. **Read:** Piper TTS docs (5 min) - Mozilla rhasspy
3. **Try:** Groq API playground (10 min) - console.groq.com
4. **Explore:** ROS 2 Python tutorial (30 min) - ROS 2 docs

---

## 🚀 Ready to Start?

**Next action:** Open `202_INSTALLATION_GUIDE.md` and start with Part 1.

Expected timeline:
- **Today:** Installation + Testing (3 hours)
- **Tomorrow:** Build + Integration (2 hours)
- **Total:** ~5 hours for complete, working system

**Questions during build?** Reference the relevant guide document and check the troubleshooting section first.

---

## 📊 Success Metrics

You'll know it's working when:

✅ Test 1 passes: All environment checks green  
✅ Test 2 passes: Record and playback audio from ReSpeaker  
✅ Test 3 passes: Whisper correctly transcribes your speech  
✅ Test 4 passes: Piper speaks German text naturally  
✅ Test 5 passes: Grok API responds to your questions  
✅ Test 6 passes: **Full loop works** (hear R2D2 respond!)  
✅ ROS 2 node runs and publishes speech responses  

---

**Quick Start Version:** 1.0  
**Status:** READY TO EXECUTE  
**Time to Deploy:** 4 hours  
**Difficulty:** Moderate  
**Quality:** Professional (97% STT, natural TTS)  

**Let's build! 🚀**
