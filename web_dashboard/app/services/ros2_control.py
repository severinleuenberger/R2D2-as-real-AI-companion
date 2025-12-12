"""ROS 2 parameter control for audio and other parameters"""
import subprocess
from typing import Optional, Dict, Any
import json


class ROS2ParameterControl:
    """Controls ROS 2 parameters via command-line interface"""
    
    def get_parameter(self, node_name: str, parameter_name: str) -> Optional[Any]:
        """
        Get a ROS 2 parameter value.
        
        Args:
            node_name: ROS 2 node name (e.g., "/audio_notification_node")
            parameter_name: Parameter name (e.g., "audio_volume")
        
        Returns:
            Parameter value or None if error
        """
        result = subprocess.run(
            ['ros2', 'param', 'get', node_name, parameter_name],
            capture_output=True, text=True
        )
        
        if result.returncode != 0:
            return None
        
        # Parse output: "Double value is: 0.05" or "String value is: 'target_person'"
        try:
            output = result.stdout.strip()
            if 'value is:' in output:
                value_str = output.split('value is:')[1].strip()
                # Remove quotes if string
                if value_str.startswith("'") and value_str.endswith("'"):
                    return value_str[1:-1]
                # Try to parse as number
                try:
                    if '.' in value_str:
                        return float(value_str)
                    else:
                        return int(value_str)
                except ValueError:
                    # Boolean or other
                    if value_str.lower() == 'true':
                        return True
                    elif value_str.lower() == 'false':
                        return False
                    return value_str
            return None
        except Exception:
            return None
    
    def set_parameter(self, node_name: str, parameter_name: str, value: Any) -> bool:
        """
        Set a ROS 2 parameter value.
        
        Args:
            node_name: ROS 2 node name
            parameter_name: Parameter name
            value: Parameter value
        
        Returns:
            True if successful, False otherwise
        """
        result = subprocess.run(
            ['ros2', 'param', 'set', node_name, parameter_name, str(value)],
            capture_output=True, text=True
        )
        
        return result.returncode == 0
    
    def get_audio_volume(self) -> Optional[float]:
        """Get current audio volume"""
        return self.get_parameter('/audio_notification_node', 'audio_volume')
    
    def set_audio_volume(self, volume: float) -> bool:
        """
        Set audio volume.
        
        Args:
            volume: Volume level (0.0 to 1.0)
        
        Returns:
            True if successful
        """
        if not 0.0 <= volume <= 1.0:
            return False
        return self.set_parameter('/audio_notification_node', 'audio_volume', volume)
    
    def get_audio_parameters(self) -> Dict[str, Any]:
        """Get all audio notification parameters"""
        params = {}
        param_names = [
            'audio_volume',
            'target_person',
            'alsa_device',
            'jitter_tolerance_seconds',
            'loss_confirmation_seconds',
            'cooldown_seconds',
            'recognition_cooldown_after_loss_seconds',
            'enabled'
        ]
        
        for param_name in param_names:
            value = self.get_parameter('/audio_notification_node', param_name)
            if value is not None:
                params[param_name] = value
        
        return params

