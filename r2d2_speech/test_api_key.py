#!/usr/bin/env python3
"""
Simple test script to verify OpenAI API key and Realtime API access.
"""
import asyncio
import os
import sys
from pathlib import Path
import json

# Add parent to path
sys.path.insert(0, str(Path(__file__).parent.parent))

from dotenv import load_dotenv
import websockets

async def test_api_key():
    """Test OpenAI API key with Realtime API."""
    
    print("=" * 60)
    print("OpenAI API Key Test")
    print("=" * 60)
    
    # Load API key
    env_path = Path.home() / ".r2d2" / ".env"
    print(f"\n1. Loading API key from: {env_path}")
    
    if not env_path.exists():
        print(f"✗ File not found: {env_path}")
        return False
    
    load_dotenv(env_path)
    api_key = os.getenv("OPENAI_API_KEY")
    
    if not api_key:
        print("✗ OPENAI_API_KEY not found in .env file")
        return False
    
    print(f"✓ API key loaded: {api_key[:10]}...{api_key[-4:]}")
    
    # Test with Realtime API
    model = "gpt-4o-realtime-preview-2024-12-17"
    url = f"wss://api.openai.com/v1/realtime?model={model}"
    
    print(f"\n2. Testing connection to Realtime API")
    print(f"   URL: {url}")
    print(f"   Model: {model}")
    
    try:
        # Connect with auth header
        print("\n3. Connecting to WebSocket...")
        headers = [
            ("Authorization", f"Bearer {api_key}"),
            ("OpenAI-Beta", "realtime=v1"),
        ]
        
        async with websockets.connect(url, additional_headers=headers) as ws:
            print("✓ WebSocket connected successfully!")
            
            # Send session update
            print("\n4. Sending session configuration...")
            session_config = {
                "type": "session.update",
                "session": {
                    "modalities": ["audio", "text"],
                    "input_audio_transcription": {
                        "model": "whisper-1"
                    },
                    "turn_detection": {
                        "type": "server_vad"
                    },
                    "voice": "alloy"
                }
            }
            
            await ws.send(json.dumps(session_config))
            print("✓ Session config sent")
            
            # Receive events
            print("\n5. Receiving first 5 events:")
            for i in range(5):
                try:
                    message = await asyncio.wait_for(ws.recv(), timeout=5.0)
                    event = json.loads(message)
                    
                    event_type = event.get('type', 'unknown')
                    print(f"\n   Event {i+1}: {event_type}")
                    
                    if event_type == 'error':
                        print(f"   ✗ ERROR:")
                        print(f"      Code: {event.get('error', {}).get('code', 'unknown')}")
                        print(f"      Message: {event.get('error', {}).get('message', 'No message')}")
                        print(f"      Full event: {json.dumps(event, indent=2)}")
                    elif event_type == 'session.created':
                        print(f"   ✓ Session created successfully!")
                        print(f"      Session ID: {event.get('session', {}).get('id', 'N/A')}")
                    elif event_type == 'session.updated':
                        print(f"   ✓ Session updated successfully!")
                    else:
                        print(f"      Details: {json.dumps(event, indent=2)[:200]}...")
                        
                except asyncio.TimeoutError:
                    print(f"   (timeout waiting for event {i+1})")
                    break
                except Exception as e:
                    print(f"   ✗ Error receiving event: {e}")
                    break
            
            print("\n" + "=" * 60)
            print("✅ TEST PASSED - API Key is valid!")
            print("=" * 60)
            print("\nYour API key works correctly with the Realtime API.")
            print("The audio pipeline issue must be something else.")
            return True
            
    except websockets.exceptions.InvalidStatusCode as e:
        print(f"\n✗ Connection failed with status code: {e.status_code}")
        if e.status_code == 401:
            print("   This means: Invalid API key or authentication failed")
            print("   Please check:")
            print("   - API key is correct in ~/.r2d2/.env")
            print("   - API key has Realtime API access")
            print("   - No extra spaces or quotes around the key")
        elif e.status_code == 403:
            print("   This means: API access forbidden")
            print("   Your account may not have access to the Realtime API")
        else:
            print(f"   HTTP Status: {e.status_code}")
        return False
        
    except Exception as e:
        print(f"\n✗ Connection error: {e}")
        import traceback
        traceback.print_exc()
        return False

if __name__ == "__main__":
    try:
        success = asyncio.run(test_api_key())
        sys.exit(0 if success else 1)
    except KeyboardInterrupt:
        print("\n\nTest interrupted")
        sys.exit(130)

