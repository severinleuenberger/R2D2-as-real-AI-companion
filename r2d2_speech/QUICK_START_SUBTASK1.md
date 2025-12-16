# Quick Start Guide - Subtask 1

## Prerequisites

1. OpenAI API key with Realtime API access
2. Virtual environment activated

## Setup (One-Time)

### 1. Add API Key to Config

Edit `~/.r2d2/.env`:

```bash
OPENAI_API_KEY=sk-proj-...your-key-here...
```

Optional settings:
```bash
REALTIME_MODEL=gpt-4o-realtime-preview-2024-12-17
REALTIME_VOICE=alloy
```

### 2. Verify Dependencies

```bash
cd ~/dev/r2d2
source r2d2_speech_env/bin/activate
pip list | grep -E "(openai|dotenv|websockets)"
```

Should show:
- openai 2.9.0
- python-dotenv 1.2.1
- websockets 15.0.1

## Run Test

```bash
cd ~/dev/r2d2
source r2d2_speech_env/bin/activate
python -m r2d2_speech.test_realtime_connection
```

## Expected Results

✅ **Success Indicators**:
- "✓ WebSocket connected"
- "✓ Session created"
- "✓ Test audio sent"
- "✓ User message saved to DB"
- "✓ Assistant message saved to DB"
- "✓ TEST PASSED"

⚠️ **Partial Success** (Expected with silence):
- Connection works
- Session created
- But no transcripts (silence not detected as speech)
- This is NORMAL - real audio needed for full test

❌ **Failure Indicators**:
- "401 Unauthorized" → Check API key
- "403 Forbidden" → Check Realtime API access
- "Connection refused" → Check internet/firewall
- Import errors → Check package installation

## Verify Database

```bash
sqlite3 ~/dev/r2d2/r2d2_speech/data/conversations.db "SELECT * FROM messages;"
```

Should show user and assistant messages (if API detected speech).

## Troubleshooting

### "OPENAI_API_KEY not found"
```bash
cat ~/.r2d2/.env | grep OPENAI_API_KEY
```

### "No module named 'websockets'"
```bash
source r2d2_speech_env/bin/activate
pip install websockets
```

### "No messages found in database"
This is expected when using silent audio. The API needs real speech or audio with content to generate transcripts.

## What's Working

After successful test, you have:

✅ OpenAI Realtime WebSocket connection  
✅ Session configuration with transcription  
✅ Event handling (user + assistant)  
✅ SQLite database persistence  
✅ Immediate message writes  

## What's NOT in Subtask 1

❌ No microphone capture (coming in Subtask 2)  
❌ No speaker playback (coming in Subtask 2)  
❌ No real-time audio streaming (coming in Subtask 2)  

## Next Steps

Once Subtask 1 test passes:
1. Proceed to Subtask 2: Audio streaming
2. Implement microphone capture (HyperX QuadCast S)
3. Implement speaker playback (PAM8403)
4. Build complete speech-to-speech pipeline

## Quick Module Tests

Test individual components:

```bash
# Config
python -c "from r2d2_speech.config import get_config; config = get_config(); print('✓ Config OK')"

# Database
python -m r2d2_speech.storage.sqlite_store

# Realtime Client
python -m r2d2_speech.realtime.realtime_client

# Event Router
python -m r2d2_speech.realtime.event_router

# Transcript Handler
python -m r2d2_speech.realtime.transcript_handler
```

All should print "✓" indicators and complete without errors.

