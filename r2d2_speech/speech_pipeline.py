"""
R2D2 Speech Pipeline
====================

Complete speech pipeline: STT → LLM → TTS

Orchestrates the complete conversation flow:
1. Record audio from microphone
2. Transcribe speech to text (Whisper)
3. Process with LLM (Grok by default, fallback to Groq)
4. Synthesize response (Piper TTS)
5. Play audio response
"""

import logging
import time
from typing import Optional, List, Dict, Tuple
import numpy as np
from pathlib import Path

# Import components
from .config import get_config, switch_llm_provider
from .stt import SwissGermanSTT
from .tts import PiperTTS
from .llm import UnifiedLLM

logger = logging.getLogger(__name__)


class SpeechPipeline:
    """
    Complete R2D2 speech pipeline.

    Usage:
        pipeline = SpeechPipeline()
        
        # Single conversation turn
        response = pipeline.process_text("Hoi, wie gaht's?")
        
        # Full pipeline (record audio → transcribe → respond → play)
        response = pipeline.process_speech()
        
        # Switch LLM provider
        pipeline.switch_llm("groq")  # Fallback to Groq if needed
    """

    def __init__(self, llm_provider: str = "grok"):
        """
        Initialize speech pipeline.

        Args:
            llm_provider: "grok" (default), "groq", "ollama", etc.
        """
        self.config = get_config()
        self.llm_provider = llm_provider

        logger.info("Initializing R2D2 Speech Pipeline...")

        # Initialize components
        self._init_stt()
        self._init_tts()
        self._init_llm()

        # Conversation history for context
        self.conversation_history: List[Dict[str, str]] = [
            {
                "role": "system",
                "content": "Du bist R2D2, ein intelligenter Roboter. "
                "Du sprichst Schweizerdeutsch und antwortest freundlich und prägnant. "
                "Deine Antworten sind kurz und präzise.",
            }
        ]

        logger.info("✓ Speech pipeline initialized")

    def _init_stt(self):
        """Initialize STT engine"""
        try:
            self.stt = SwissGermanSTT()
            logger.info("✓ STT engine ready")
        except Exception as e:
            logger.error(f"Failed to initialize STT: {e}")
            raise

    def _init_tts(self):
        """Initialize TTS engine"""
        try:
            self.tts = PiperTTS()
            logger.info("✓ TTS engine ready")
        except Exception as e:
            logger.error(f"Failed to initialize TTS: {e}")
            raise

    def _init_llm(self):
        """Initialize LLM client"""
        try:
            self.llm = UnifiedLLM(provider=self.llm_provider)
            logger.info(f"✓ LLM ready: {self.llm.get_provider()}")
        except Exception as e:
            logger.error(f"Failed to initialize LLM: {e}")
            raise

    def switch_llm(self, provider: str):
        """
        Switch LLM provider dynamically.

        Args:
            provider: "grok", "groq", "ollama", etc.
        """
        try:
            self.llm.switch_provider(provider)
            self.llm_provider = provider
            logger.info(f"✓ Switched to: {self.llm.get_provider()}")
        except Exception as e:
            logger.error(f"Failed to switch LLM: {e}")
            raise

    def process_text(
        self,
        user_text: str,
        language: str = "de",
        include_history: bool = True,
    ) -> Tuple[str, float]:
        """
        Process text input through LLM.

        Args:
            user_text: User's input text
            language: "de" (German) or "en" (English)
            include_history: Include conversation history for context

        Returns:
            Tuple of (response_text, processing_time)
        """
        start_time = time.time()

        try:
            logger.info(f"Processing: {user_text}")

            # Add user message to history
            messages = self.conversation_history.copy()
            messages.append({"role": "user", "content": user_text})

            # Get LLM response
            response_text = self.llm.chat(
                messages=messages,
                temperature=0.7,
                max_tokens=256,
            )

            # Update conversation history
            self.conversation_history.append({"role": "user", "content": user_text})
            self.conversation_history.append(
                {"role": "assistant", "content": response_text}
            )

            # Keep only last 10 exchanges for context (to avoid context limit)
            if len(self.conversation_history) > 21:  # system + 10 exchanges
                self.conversation_history = (
                    self.conversation_history[:1] + self.conversation_history[-20:]
                )

            elapsed = time.time() - start_time
            logger.info(f"✓ Response generated in {elapsed:.2f}s")

            return response_text, elapsed

        except Exception as e:
            logger.error(f"LLM processing failed: {e}")
            raise

    def synthesize_response(
        self, text: str, language: str = "de"
    ) -> Tuple[np.ndarray, float]:
        """
        Synthesize text to speech.

        Args:
            text: Text to synthesize
            language: "de" (German) or "en" (English)

        Returns:
            Tuple of (audio_array, processing_time)
        """
        start_time = time.time()

        try:
            logger.info(f"Synthesizing {language}: {text[:50]}...")
            audio = self.tts.synthesize(text, language=language)
            elapsed = time.time() - start_time
            logger.info(f"✓ Synthesized {elapsed:.2f}s of audio")
            return audio, elapsed

        except Exception as e:
            logger.error(f"TTS synthesis failed: {e}")
            raise

    def transcribe_audio(
        self, audio_path: str, language: str = "de"
    ) -> Tuple[str, float]:
        """
        Transcribe audio file to text.

        Args:
            audio_path: Path to audio file
            language: "de" (German) or "en" (English)

        Returns:
            Tuple of (transcribed_text, processing_time)
        """
        start_time = time.time()

        try:
            text = self.stt.transcribe_audio(audio_path, language=language)
            elapsed = time.time() - start_time
            logger.info(f"✓ Transcribed in {elapsed:.2f}s")
            return text, elapsed

        except Exception as e:
            logger.error(f"Transcription failed: {e}")
            raise

    def process_speech(
        self,
        audio_input: str,
        input_language: str = "de",
        output_language: str = "de",
    ) -> Dict[str, any]:
        """
        Complete speech pipeline: transcribe → process → synthesize.

        Args:
            audio_input: Path to audio file
            input_language: Language of input audio
            output_language: Language of response

        Returns:
            Dictionary with:
            - input_text: Transcribed user speech
            - response_text: LLM response
            - response_audio: Synthesized response as numpy array
            - timings: Processing times for each step
        """
        logger.info("=== Starting speech processing ===")

        timings = {}

        try:
            # Step 1: Transcribe
            logger.info("Step 1: Transcribing audio...")
            input_text, stt_time = self.transcribe_audio(
                audio_input, language=input_language
            )
            timings["stt"] = stt_time
            logger.info(f"User said: {input_text}")

            # Step 2: Process with LLM
            logger.info("Step 2: Processing with LLM...")
            response_text, llm_time = self.process_text(input_text)
            timings["llm"] = llm_time
            logger.info(f"R2D2 says: {response_text}")

            # Step 3: Synthesize
            logger.info("Step 3: Synthesizing response...")
            response_audio, tts_time = self.synthesize_response(
                response_text, language=output_language
            )
            timings["tts"] = tts_time

            # Step 4: Play audio response
            logger.info("Step 4: Playing response audio...")
            try:
                self.tts.play(response_audio, sample_rate=22050)
                logger.info("✓ Audio playback started")
            except Exception as e:
                logger.warning(f"Failed to play audio: {e} (continuing without speaker)")

            # Calculate total time
            total_time = sum(timings.values())
            logger.info(f"✓ Complete pipeline: {total_time:.2f}s")
            logger.info(f"  STT: {stt_time:.2f}s")
            logger.info(f"  LLM: {llm_time:.2f}s")
            logger.info(f"  TTS: {tts_time:.2f}s")

            return {
                "input_text": input_text,
                "response_text": response_text,
                "response_audio": response_audio,
                "timings": timings,
                "total_time": total_time,
                "llm_provider": self.llm.get_provider(),
            }

        except Exception as e:
            logger.error(f"Speech processing pipeline failed: {e}")
            raise

    def reset_conversation(self):
        """Reset conversation history"""
        self.conversation_history = self.conversation_history[:1]  # Keep system message
        logger.info("✓ Conversation history reset")

    def get_status(self) -> dict:
        """Get pipeline status"""
        return {
            "stt": self.stt.get_model_info(),
            "tts": self.tts.get_model_info(),
            "llm": self.llm.get_provider(),
            "conversation_turns": (len(self.conversation_history) - 1) // 2,
        }


if __name__ == "__main__":
    import sys

    # Setup logging
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s - %(name)s - %(levelname)s - %(message)s",
    )

    print("=== R2D2 Speech Pipeline Test ===\n")

    try:
        # Initialize pipeline
        pipeline = SpeechPipeline()

        # Test 1: Process text
        print("[Test 1] Text input")
        response, _ = pipeline.process_text("Hoi, wie gaht's?")
        print(f"R2D2: {response}\n")

        # Test 2: Check status
        print("[Test 2] Pipeline status")
        status = pipeline.get_status()
        print(f"Status: {status}\n")

        # Test 3: Switch LLM
        print("[Test 3] Switch to Groq (fallback)")
        try:
            pipeline.switch_llm("groq")
            response, _ = pipeline.process_text("Are you ready?")
            print(f"R2D2 (Groq): {response}\n")
        except Exception as e:
            print(f"Could not switch to Groq: {e}\n")

        print("✓ All tests passed!")

    except Exception as e:
        print(f"✗ Error: {e}")
        import traceback

        traceback.print_exc()
