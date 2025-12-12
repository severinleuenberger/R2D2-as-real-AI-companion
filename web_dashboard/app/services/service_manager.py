"""Systemd service management for R2D2 services"""
import subprocess
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
        return {
            service_name: self.get_status(service_name)
            for service_name in SERVICES.keys()
        }
    
    def start_service(self, service_name: str) -> Dict[str, any]:
        """
        Start a systemd service.
        
        Args:
            service_name: Service name (e.g., "audio", "camera")
        
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
            return {"success": True, "message": f"{service_name} started successfully"}
        else:
            error_msg = result.stderr.strip() if result.stderr.strip() else result.stdout.strip()
            if "password" in error_msg.lower() or "sudo" in error_msg.lower():
                error_msg = "Passwordless sudo not configured. Run: echo 'severin ALL=(ALL) NOPASSWD: /bin/systemctl start r2d2-*, /bin/systemctl stop r2d2-*, /bin/systemctl restart r2d2-*' | sudo tee /etc/sudoers.d/r2d2-services"
            return {
                "success": False,
                "error": f"Failed to start {service_name}: {error_msg}"
            }
    
    def stop_service(self, service_name: str) -> Dict[str, any]:
        """
        Stop a systemd service.
        
        Args:
            service_name: Service name (e.g., "audio", "camera")
        
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

