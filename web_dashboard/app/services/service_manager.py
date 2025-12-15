"""Systemd service management for R2D2 services"""
import subprocess
import time
from typing import Dict, Optional
from app.config import SERVICES


class ServiceManager:
    """Manages systemd services for R2D2 system"""
    
    def get_status(self, service_name: str) -> Dict[str, any]:
        """
        Get systemd service status.
        
        Args:
            service_name: Service name (e.g., "audio", "camera")
        
        Returns:
            Dictionary with status information
        """
        full_service_name = SERVICES.get(service_name)
        if not full_service_name:
            return {"error": f"Unknown service: {service_name}"}
        
        # Check if active
        result = subprocess.run(
            ['systemctl', 'is-active', full_service_name],
            capture_output=True, text=True
        )
        active = result.stdout.strip() == 'active'
        
        # Check if enabled
        result = subprocess.run(
            ['systemctl', 'is-enabled', full_service_name],
            capture_output=True, text=True
        )
        enabled = result.stdout.strip() == 'enabled'
        
        # Get detailed status
        result = subprocess.run(
            ['systemctl', 'show', full_service_name, '--property=ActiveState,SubState'],
            capture_output=True, text=True
        )
        state = "unknown"
        substate = "unknown"
        for line in result.stdout.strip().split('\n'):
            if line.startswith('ActiveState='):
                state = line.split('=')[1]
            elif line.startswith('SubState='):
                substate = line.split('=')[1]
        
        return {
            "service_name": service_name,
            "full_name": full_service_name,
            "status": "active" if active else "inactive",
            "enabled": enabled,
            "state": state,
            "substate": substate
        }
    
    def get_all_services_status(self) -> Dict[str, Dict]:
        """Get status of all R2D2 services"""
        result = {}
        for service_name in SERVICES.keys():
            status = self.get_status(service_name)
            # Include all services, even if they have errors (so UI can show them)
            result[service_name] = status
        return result
    
    def _check_topic_exists(self, topic_name: str) -> bool:
        """
        Check if a ROS 2 topic exists and is publishing.
        
        Args:
            topic_name: ROS 2 topic name (e.g., "/r2d2/audio/person_status")
        
        Returns:
            True if topic exists, False otherwise
        """
        try:
            result = subprocess.run(
                ['ros2', 'topic', 'list'],
                capture_output=True,
                text=True,
                timeout=2,
                env={**subprocess.os.environ, 'ROS_DOMAIN_ID': '0'}
            )
            if result.returncode == 0:
                topics = [line.strip() for line in result.stdout.strip().split('\n') if line.strip()]
                return topic_name in topics
        except (subprocess.TimeoutExpired, FileNotFoundError):
            pass
        return False
    
    def start_service(self, service_name: str, verify_topics: bool = False) -> Dict[str, any]:
        """
        Start a systemd service.
        
        Args:
            service_name: Service name (e.g., "audio", "camera")
            verify_topics: If True, verify expected topics are publishing after start
        
        Returns:
            Dictionary with success status
        """
        full_service_name = SERVICES.get(service_name)
        if not full_service_name:
            return {"success": False, "error": f"Unknown service: {service_name}"}
        
        result = subprocess.run(
            ['sudo', 'systemctl', 'start', full_service_name],
            capture_output=True, text=True
        )
        
        if result.returncode == 0:
            # Wait a moment for service to initialize
            time.sleep(2)
            
            # Verify service is actually running
            check_result = subprocess.run(
                ['systemctl', 'is-active', full_service_name],
                capture_output=True, text=True
            )
            if check_result.stdout.strip() != 'active':
                return {
                    "success": False,
                    "error": f"{service_name} start command succeeded but service is not active. Check logs: sudo journalctl -u {full_service_name} -n 20"
                }
            
            # Verify topics if requested
            if verify_topics:
                missing_topics = []
                if service_name == "camera":
                    # Camera-perception should publish /oak/rgb/image_raw
                    if not self._check_topic_exists("/oak/rgb/image_raw"):
                        missing_topics.append("/oak/rgb/image_raw")
                elif service_name == "audio":
                    # Audio-notification should publish /r2d2/audio/person_status
                    if not self._check_topic_exists("/r2d2/audio/person_status"):
                        missing_topics.append("/r2d2/audio/person_status")
                
                if missing_topics:
                    return {
                        "success": False,
                        "error": f"{service_name} started but expected topics not publishing: {', '.join(missing_topics)}. Check logs: sudo journalctl -u {full_service_name} -n 20"
                    }
            
            return {"success": True, "message": f"{service_name} started successfully"}
        else:
            error_msg = result.stderr.strip() if result.stderr.strip() else result.stdout.strip()
            if "password" in error_msg.lower() or "sudo" in error_msg.lower():
                error_msg = "Passwordless sudo not configured. Run: echo 'severin ALL=(ALL) NOPASSWD: /bin/systemctl start r2d2-*, /bin/systemctl stop r2d2-*, /bin/systemctl restart r2d2-*' | sudo tee /etc/sudoers.d/r2d2-services"
            return {
                "success": False,
                "error": f"Failed to start {service_name}: {error_msg}"
            }
    
    def stop_service(self, service_name: str, verify: bool = False) -> Dict[str, any]:
        """
        Stop a systemd service.
        
        Args:
            service_name: Service name (e.g., "audio", "camera")
            verify: If True, verify service is actually stopped before returning
        
        Returns:
            Dictionary with success status
        """
        full_service_name = SERVICES.get(service_name)
        if not full_service_name:
            return {"success": False, "error": f"Unknown service: {service_name}"}
        
        result = subprocess.run(
            ['sudo', 'systemctl', 'stop', full_service_name],
            capture_output=True, text=True
        )
        
        if result.returncode == 0:
            # Verify service is actually stopped if requested
            if verify:
                max_attempts = 5
                for attempt in range(max_attempts):
                    check_result = subprocess.run(
                        ['systemctl', 'is-active', full_service_name],
                        capture_output=True, text=True
                    )
                    if check_result.stdout.strip() != 'active':
                        return {"success": True, "message": f"{service_name} stopped successfully"}
                    time.sleep(0.5)
                return {"success": False, "error": f"{service_name} stop command succeeded but service still active"}
            
            return {"success": True, "message": f"{service_name} stopped successfully"}
        else:
            error_msg = result.stderr.strip() if result.stderr.strip() else result.stdout.strip()
            if "password" in error_msg.lower() or "sudo" in error_msg.lower():
                error_msg = "Passwordless sudo not configured. See documentation for setup instructions."
            return {
                "success": False,
                "error": f"Failed to stop {service_name}: {error_msg}"
            }
    
    def restart_service(self, service_name: str) -> Dict[str, any]:
        """
        Restart a systemd service.
        
        Args:
            service_name: Service name (e.g., "audio", "camera")
        
        Returns:
            Dictionary with success status
        """
        full_service_name = SERVICES.get(service_name)
        if not full_service_name:
            return {"success": False, "error": f"Unknown service: {service_name}"}
        
        result = subprocess.run(
            ['sudo', 'systemctl', 'restart', full_service_name],
            capture_output=True, text=True
        )
        
        if result.returncode == 0:
            return {"success": True, "message": f"{service_name} restarted successfully"}
        else:
            error_msg = result.stderr.strip() if result.stderr.strip() else result.stdout.strip()
            if "password" in error_msg.lower() or "sudo" in error_msg.lower():
                error_msg = "Passwordless sudo not configured. See documentation for setup instructions."
            return {
                "success": False,
                "error": f"Failed to restart {service_name}: {error_msg}"
            }
    
    def start_recognition_mode(self) -> Dict[str, any]:
        """
        Start recognition mode: stop stream, start camera-perception + audio.
        Ensures mutual exclusivity with streaming mode by stopping conflicting service
        and waiting for device release before starting camera-perception.
        """
        errors = []
        messages = []
        
        # Stop stream service first and verify it's actually stopped
        stream_result = self.stop_service("camera-stream", verify=True)
        if not stream_result.get("success"):
            errors.append(f"Failed to stop stream: {stream_result.get('error')}")
        else:
            messages.append("Stream stopped")
            # Wait for device release (OAK-D camera needs time to free up)
            time.sleep(3)
        
        # Start camera-perception service with topic verification
        camera_result = self.start_service("camera", verify_topics=True)
        if not camera_result.get("success"):
            errors.append(f"Failed to start camera: {camera_result.get('error')}")
        else:
            messages.append("Camera-perception started")
        
        # Start audio notification service with topic verification
        audio_result = self.start_service("audio", verify_topics=True)
        if not audio_result.get("success"):
            errors.append(f"Failed to start audio: {audio_result.get('error')}")
        else:
            messages.append("Audio notification started")
        
        if errors:
            return {"success": False, "error": "; ".join(errors), "partial": messages}
        return {"success": True, "message": "; ".join(messages)}
    
    def stop_recognition_mode(self) -> Dict[str, any]:
        """
        Stop recognition mode: stop camera-perception + audio.
        """
        errors = []
        messages = []
        
        # Stop audio notification service
        audio_result = self.stop_service("audio")
        if not audio_result.get("success"):
            errors.append(f"Failed to stop audio: {audio_result.get('error')}")
        else:
            messages.append("Audio notification stopped")
        
        # Stop camera-perception service
        camera_result = self.stop_service("camera")
        if not camera_result.get("success"):
            errors.append(f"Failed to stop camera: {camera_result.get('error')}")
        else:
            messages.append("Camera-perception stopped")
        
        if errors:
            return {"success": False, "error": "; ".join(errors), "partial": messages}
        return {"success": True, "message": "; ".join(messages)}
    
    def start_stream_mode(self) -> Dict[str, any]:
        """
        Start stream mode: stop recognition services, start stream.
        Ensures mutual exclusivity with recognition mode by stopping camera-perception
        and waiting for device release before starting camera-stream.
        """
        errors = []
        messages = []
        
        # Stop camera-perception service first (most critical for device exclusivity)
        camera_result = self.stop_service("camera", verify=True)
        if not camera_result.get("success"):
            errors.append(f"Failed to stop camera-perception: {camera_result.get('error')}")
        else:
            messages.append("Camera-perception stopped")
            # Wait for device release (OAK-D camera needs time to free up)
            time.sleep(3)
        
        # Stop audio notification service (not device-related, but part of recognition mode)
        audio_result = self.stop_service("audio")
        if not audio_result.get("success"):
            errors.append(f"Failed to stop audio: {audio_result.get('error')}")
        else:
            messages.append("Audio notification stopped")
        
        # Start stream service (no topic verification needed for stream service)
        stream_result = self.start_service("camera-stream", verify_topics=False)
        if not stream_result.get("success"):
            errors.append(f"Failed to start stream: {stream_result.get('error')}")
        else:
            messages.append("Camera stream started")
        
        if errors:
            return {"success": False, "error": "; ".join(errors), "partial": messages}
        return {"success": True, "message": "; ".join(messages)}
    
    def stop_stream_mode(self) -> Dict[str, any]:
        """
        Stop stream mode: stop camera-stream service.
        """
        return self.stop_service("camera-stream")
    
    def get_service_command(self, service_name: str, action: str) -> str:
        """
        Generate command-line instruction for a service action.
        
        Args:
            service_name: Service name (e.g., "audio", "camera")
            action: Action to perform ("start", "stop", "restart")
        
        Returns:
            Executable command string
        """
        full_service_name = SERVICES.get(service_name)
        if not full_service_name:
            return f"# Unknown service: {service_name}"
        
        if action not in ["start", "stop", "restart"]:
            return f"# Unknown action: {action}"
        
        return f"sudo systemctl {action} {full_service_name}"
    
    def get_recognition_mode_command(self, action: str) -> str:
        """
        Generate command-line instruction for recognition mode.
        
        Args:
            action: "start" or "stop"
        
        Returns:
            Executable command string
        """
        if action == "start":
            camera_cmd = self.get_service_command("camera", "start")
            audio_cmd = self.get_service_command("audio", "start")
            stream_cmd = self.get_service_command("camera-stream", "stop")
            return f"{stream_cmd} && sleep 3 && {camera_cmd} && {audio_cmd}"

        elif action == "stop":
            audio_cmd = self.get_service_command("audio", "stop")
            camera_cmd = self.get_service_command("camera", "stop")
            return f"{audio_cmd} && {camera_cmd}"
        else:
            return f"# Unknown action: {action}"
    
    def get_stream_mode_command(self, action: str) -> str:
        """
        Generate command-line instruction for stream mode.
        
        Args:
            action: "start" or "stop"
        
        Returns:
            Executable command string
        """
        if action == "start":
            stream_cmd = self.get_service_command("camera-stream", "start")
            audio_cmd = self.get_service_command("audio", "stop")
            camera_cmd = self.get_service_command("camera", "stop")
            return f"{audio_cmd} && {camera_cmd} && {stream_cmd}"
        elif action == "stop":
            return self.get_service_command("camera-stream", "stop")
        else:
            return f"# Unknown action: {action}"

