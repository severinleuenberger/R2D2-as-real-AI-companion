"""
Configuration Management
========================

Loads configuration from ~/.r2d2/.env and provides config access.
"""

from .config_manager import get_config

__all__ = ["get_config"]

