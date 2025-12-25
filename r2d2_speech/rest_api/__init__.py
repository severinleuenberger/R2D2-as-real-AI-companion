"""
REST API Speech Pipeline
========================

Turn-based speech pipeline using OpenAI REST APIs:
- Whisper API for speech-to-text
- Chat Completions API for LLM (o1-preview or other models)
- TTS API for text-to-speech

This "Intelligent Mode" is triggered by open_hand gesture and provides
slower but more thoughtful responses compared to the Realtime API.
"""

from .rest_speech_client import RestSpeechClient, AudioRecorder

__all__ = ['RestSpeechClient', 'AudioRecorder']

