"""
Configuration Manager
=====================

Loads configuration from ~/.r2d2/.env and provides validated settings.

All configuration comes from environment variables - single source of truth.
"""

import os
from pathlib import Path
from typing import Dict, Any
from dotenv import load_dotenv


def get_config() -> Dict[str, Any]:
    """
    Load and validate configuration from ~/.r2d2/.env
    
    Returns:
        Dict containing all configuration settings
        
    Raises:
        ValueError: If OPENAI_API_KEY is missing
        FileNotFoundError: If .env file doesn't exist
    """
    # Load .env file from ~/.r2d2/.env
    env_path = Path.home() / ".r2d2" / ".env"
    
    if not env_path.exists():
        raise FileNotFoundError(
            f"Configuration file not found: {env_path}\n"
            f"Please create ~/.r2d2/.env with OPENAI_API_KEY"
        )
    
    # Load environment variables from .env file
    load_dotenv(env_path)
    
    # Validate required fields
    api_key = os.getenv("OPENAI_API_KEY")
    if not api_key:
        raise ValueError(
            "OPENAI_API_KEY is required in ~/.r2d2/.env\n"
            "Add: OPENAI_API_KEY=sk-..."
        )
    
    # Build configuration dictionary
    config = {
        # Required
        "openai_api_key": api_key,
        
        # Realtime API settings (optional with defaults)
        "realtime_model": os.getenv(
            "REALTIME_MODEL",
            "gpt-4o-realtime-preview-2024-12-17"
        ),
        "realtime_voice": os.getenv("REALTIME_VOICE", "alloy"),
        
        # Audio device settings (optional)
        "mic_device": os.getenv("MIC_DEVICE"),  # None if not set - use auto-detection
        "mic_native_sample_rate": int(os.getenv("MIC_NATIVE_SAMPLE_RATE", "48000")),
        "mic_sample_rate": int(os.getenv("MIC_SAMPLE_RATE", "24000")),  # Target for API
        "mic_channels": int(os.getenv("MIC_CHANNELS", "1")),
        "sink_device": os.getenv("SINK_DEVICE", "default"),
        
        # Database path
        "db_path": os.path.expanduser(
            "~/dev/r2d2/r2d2_speech/data/conversations.db"
        ),
    }
    
    return config


def validate_config(config: Dict[str, Any]) -> bool:
    """
    Validate configuration dictionary.
    
    Args:
        config: Configuration dictionary to validate
        
    Returns:
        True if valid
        
    Raises:
        ValueError: If configuration is invalid
    """
    # Check API key format (should start with sk-)
    api_key = config.get("openai_api_key", "")
    if not api_key.startswith("sk-"):
        raise ValueError(
            f"Invalid OPENAI_API_KEY format. "
            f"Expected to start with 'sk-', got: {api_key[:10]}..."
        )
    
    # Check model name
    model = config.get("realtime_model", "")
    if not model:
        raise ValueError("REALTIME_MODEL cannot be empty")
    
    # Check voice
    valid_voices = ["alloy", "echo", "fable", "onyx", "nova", "shimmer"]
    voice = config.get("realtime_voice", "")
    if voice not in valid_voices:
        raise ValueError(
            f"Invalid REALTIME_VOICE: {voice}. "
            f"Must be one of: {', '.join(valid_voices)}"
        )
    
    return True


if __name__ == "__main__":
    # Test configuration loading
    try:
        config = get_config()
        print("✓ Configuration loaded successfully")
        print(f"  Model: {config['realtime_model']}")
        print(f"  Voice: {config['realtime_voice']}")
        print(f"  API Key: {config['openai_api_key'][:10]}...")
        print(f"  DB Path: {config['db_path']}")
        
        validate_config(config)
        print("✓ Configuration is valid")
        
    except Exception as e:
        print(f"✗ Configuration error: {e}")
        exit(1)

