#!/usr/bin/env python3
"""
Jetson Freeze Monitor
=====================

Comprehensive system monitoring to diagnose Jetson freezes.
Logs hardware metrics, kernel messages, system resources, and process info.

Runs as a systemd service and logs to /var/log/freeze_logs/
"""

import subprocess
import time
import os
import sys
from datetime import datetime
from pathlib import Path
import json
import re

# Configuration
LOG_DIR = Path("/var/log/freeze_logs")
LOG_INTERVAL = 60  # seconds between logs
DISK_WARN_THRESHOLD_GB = 1  # warn if free space below this
DISK_WARN_THRESHOLD_PERCENT = 92  # warn if disk usage above this percentage
DISK_WARN_COOLDOWN_SECONDS = 3600  # 1 hour between audio warnings
DISK_WARN_SOUND = "/usr/local/share/r2d2/sounds/disk_warning.mp3"

# Thermal thresholds for Jetson Orin Nano (official specs)
TEMP_WARN_THRESHOLD = 76  # Warning at 76°C (early detection for freeze diagnosis)
TEMP_CRITICAL_THRESHOLD = 99  # Critical at 99°C (max operating temp)
TEMP_WARN_COOLDOWN_SECONDS = 300  # 5 minutes between thermal warnings

# Log files
HARDWARE_LOG = LOG_DIR / "hardware.log"
KERNEL_LOG = LOG_DIR / "kernel.log"
SYSTEM_LOG = LOG_DIR / "system.log"
PROCESSES_LOG = LOG_DIR / "processes.log"
SUMMARY_LOG = LOG_DIR / "summary.log"


class FreezeMonitor:
    """Main monitoring class"""
    
    def __init__(self):
        self.last_kernel_line = 0
        self.start_time = datetime.now()
        self.log_counter = 0
        self.last_disk_warning = 0  # timestamp of last disk warning sound
        self.last_temp_warning = 0  # timestamp of last temperature warning
        
    def get_timestamp(self):
        """Get ISO formatted timestamp"""
        return datetime.now().isoformat(sep=' ', timespec='milliseconds')
    
    def check_disk_space(self):
        """Check available disk space and warn if low"""
        try:
            result = subprocess.run(
                ['df', '-BG', str(LOG_DIR.parent)],
                capture_output=True, text=True, timeout=5
            )
            if result.returncode == 0:
                lines = result.stdout.strip().split('\n')
                if len(lines) >= 2:
                    parts = lines[1].split()
                    if len(parts) >= 5:
                        # Get usage percentage (4th column, format: XX%)
                        usage_str = parts[4].rstrip('%')
                        available = parts[3].rstrip('G')
                        try:
                            usage_percent = int(usage_str)
                            available_gb = int(available)
                            
                            # Check percentage threshold for audio warning
                            if usage_percent >= DISK_WARN_THRESHOLD_PERCENT:
                                current_time = time.time()
                                # Play sound only if cooldown period has passed
                                if current_time - self.last_disk_warning > DISK_WARN_COOLDOWN_SECONDS:
                                    self.log_to_file(
                                        SUMMARY_LOG,
                                        f"⚠️ DISK SPACE WARNING: {usage_percent}% used ({available_gb}GB free)!"
                                    )
                                    self.play_warning_sound("disk")
                                    self.last_disk_warning = current_time
                            
                            # Also check absolute GB threshold
                            if available_gb < DISK_WARN_THRESHOLD_GB:
                                self.log_to_file(
                                    SUMMARY_LOG,
                                    f"WARNING: Low disk space: {available_gb}GB available"
                                )
                                return False
                        except ValueError:
                            pass
            return True
        except Exception as e:
            return True  # Continue logging even if check fails
    
    def play_warning_sound(self, alert_type="disk"):
        """Play R2-D2 warning sound"""
        try:
            # Try multiple audio players in order of preference
            for player_cmd in [
                ['ffplay', '-nodisp', '-autoexit', '-loglevel', 'quiet', DISK_WARN_SOUND],
                ['paplay', DISK_WARN_SOUND],
                ['aplay', DISK_WARN_SOUND],
            ]:
                try:
                    subprocess.Popen(
                        player_cmd,
                        stdout=subprocess.DEVNULL,
                        stderr=subprocess.DEVNULL
                    )
                    self.log_to_file(SUMMARY_LOG, f"🔊 Playing {alert_type} warning sound")
                    break
                except FileNotFoundError:
                    continue
        except Exception as e:
            self.log_to_file(SUMMARY_LOG, f"Failed to play warning sound: {e}")
    
    def log_to_file(self, filepath, message):
        """Append message to log file with timestamp"""
        try:
            timestamp = self.get_timestamp()
            with open(filepath, 'a') as f:
                f.write(f"[{timestamp}] {message}\n")
        except Exception as e:
            print(f"Error writing to {filepath}: {e}", file=sys.stderr)
    
    def get_tegrastats(self):
        """Get Jetson hardware stats via tegrastats"""
        try:
            # Run tegrastats for 1 second to get one reading
            result = subprocess.run(
                ['tegrastats', '--interval', '100'],
                capture_output=True, text=True, timeout=2,
                input='q\n'  # Send quit command
            )
            
            if result.stdout:
                # Parse tegrastats output (single line)
                lines = result.stdout.strip().split('\n')
                if lines:
                    return lines[-1]  # Get last line
            return None
        except Exception as e:
            return f"Error: {e}"
    
    def get_kernel_messages(self):
        """Get recent kernel messages from dmesg"""
        try:
            result = subprocess.run(
                ['dmesg', '-T', '--level=emerg,alert,crit,err,warn'],
                capture_output=True, text=True, timeout=5
            )
            if result.returncode == 0:
                lines = result.stdout.strip().split('\n')
                # Get last 10 lines
                return lines[-10:] if lines else []
            return []
        except Exception as e:
            return [f"Error getting dmesg: {e}"]
    
    def get_system_resources(self):
        """Get system resource usage"""
        stats = {}
        
        # Memory info
        try:
            with open('/proc/meminfo', 'r') as f:
                meminfo = f.read()
                for line in meminfo.split('\n'):
                    if 'MemTotal:' in line:
                        stats['mem_total_kb'] = int(line.split()[1])
                    elif 'MemAvailable:' in line:
                        stats['mem_available_kb'] = int(line.split()[1])
                    elif 'MemFree:' in line:
                        stats['mem_free_kb'] = int(line.split()[1])
                    elif 'SwapTotal:' in line:
                        stats['swap_total_kb'] = int(line.split()[1])
                    elif 'SwapFree:' in line:
                        stats['swap_free_kb'] = int(line.split()[1])
        except Exception as e:
            stats['mem_error'] = str(e)
        
        # Load average
        try:
            with open('/proc/loadavg', 'r') as f:
                loadavg = f.read().strip().split()
                stats['load_1min'] = float(loadavg[0])
                stats['load_5min'] = float(loadavg[1])
                stats['load_15min'] = float(loadavg[2])
        except Exception as e:
            stats['load_error'] = str(e)
        
        # Disk I/O stats
        try:
            result = subprocess.run(
                ['iostat', '-x', '1', '1'],
                capture_output=True, text=True, timeout=3
            )
            if result.returncode == 0:
                stats['iostat'] = result.stdout.strip().split('\n')[-1]
        except Exception:
            pass  # iostat might not be installed
        
        # Uptime
        try:
            with open('/proc/uptime', 'r') as f:
                uptime_seconds = float(f.read().split()[0])
                stats['uptime_hours'] = round(uptime_seconds / 3600, 2)
        except Exception as e:
            stats['uptime_error'] = str(e)
        
        return stats
    
    def get_top_processes(self):
        """Get top processes by CPU and memory"""
        processes = []
        try:
            # Get top 10 processes by CPU
            result = subprocess.run(
                ['ps', 'aux', '--sort=-%cpu'],
                capture_output=True, text=True, timeout=5
            )
            if result.returncode == 0:
                lines = result.stdout.strip().split('\n')
                # Get header and top 10 processes
                processes = lines[:11]
        except Exception as e:
            processes = [f"Error getting processes: {e}"]
        
        return processes
    
    def get_thermal_info(self):
        """Get thermal zone temperatures and check for overheat"""
        thermal_info = {}
        max_temp = 0.0
        max_zone = ""
        
        try:
            thermal_zones = Path('/sys/class/thermal')
            if thermal_zones.exists():
                for zone in thermal_zones.glob('thermal_zone*'):
                    try:
                        zone_type = (zone / 'type').read_text().strip()
                        temp = int((zone / 'temp').read_text().strip())
                        temp_celsius = temp / 1000.0
                        thermal_info[zone_type] = f"{temp_celsius:.1f}°C"
                        
                        # Track maximum temperature
                        if temp_celsius > max_temp:
                            max_temp = temp_celsius
                            max_zone = zone_type
                    except Exception:
                        pass
                
                # Check for overheat conditions (every cycle, i.e., every 60 seconds)
                if max_temp >= TEMP_CRITICAL_THRESHOLD:
                    current_time = time.time()
                    if current_time - self.last_temp_warning > TEMP_WARN_COOLDOWN_SECONDS:
                        self.log_to_file(
                            SUMMARY_LOG,
                            f"🔥 CRITICAL TEMPERATURE: {max_zone} at {max_temp:.1f}°C (limit: {TEMP_CRITICAL_THRESHOLD}°C)!"
                        )
                        self.play_warning_sound("thermal")
                        self.last_temp_warning = current_time
                elif max_temp >= TEMP_WARN_THRESHOLD:
                    current_time = time.time()
                    if current_time - self.last_temp_warning > TEMP_WARN_COOLDOWN_SECONDS:
                        self.log_to_file(
                            SUMMARY_LOG,
                            f"⚠️ HIGH TEMPERATURE WARNING: {max_zone} at {max_temp:.1f}°C (warning: {TEMP_WARN_THRESHOLD}°C)"
                        )
                        self.play_warning_sound("thermal")
                        self.last_temp_warning = current_time
                        
        except Exception as e:
            thermal_info['error'] = str(e)
        
        return thermal_info
    
    def log_hardware_metrics(self):
        """Log hardware metrics"""
        self.log_to_file(HARDWARE_LOG, "=== Hardware Metrics ===")
        
        # Tegrastats
        tegrastats = self.get_tegrastats()
        if tegrastats:
            self.log_to_file(HARDWARE_LOG, f"Tegrastats: {tegrastats}")
        
        # Thermal zones
        thermal = self.get_thermal_info()
        if thermal:
            self.log_to_file(HARDWARE_LOG, f"Thermal: {json.dumps(thermal)}")
        
        self.log_to_file(HARDWARE_LOG, "")
    
    def log_kernel_messages(self):
        """Log kernel messages"""
        self.log_to_file(KERNEL_LOG, "=== Kernel Messages ===")
        
        messages = self.get_kernel_messages()
        if messages:
            for msg in messages:
                if msg:
                    self.log_to_file(KERNEL_LOG, msg)
        else:
            self.log_to_file(KERNEL_LOG, "No critical kernel messages")
        
        self.log_to_file(KERNEL_LOG, "")
    
    def log_system_resources(self):
        """Log system resource usage"""
        self.log_to_file(SYSTEM_LOG, "=== System Resources ===")
        
        resources = self.get_system_resources()
        
        # Calculate memory usage percentage
        if 'mem_total_kb' in resources and 'mem_available_kb' in resources:
            mem_used_pct = (1 - resources['mem_available_kb'] / resources['mem_total_kb']) * 100
            self.log_to_file(
                SYSTEM_LOG,
                f"Memory: {mem_used_pct:.1f}% used "
                f"({resources['mem_available_kb']/1024:.0f}MB available / "
                f"{resources['mem_total_kb']/1024:.0f}MB total)"
            )
        
        # Swap usage
        if 'swap_total_kb' in resources and resources['swap_total_kb'] > 0:
            swap_used = resources['swap_total_kb'] - resources['swap_free_kb']
            swap_used_pct = (swap_used / resources['swap_total_kb']) * 100
            self.log_to_file(
                SYSTEM_LOG,
                f"Swap: {swap_used_pct:.1f}% used ({swap_used/1024:.0f}MB / {resources['swap_total_kb']/1024:.0f}MB)"
            )
        
        # Load average
        if 'load_1min' in resources:
            self.log_to_file(
                SYSTEM_LOG,
                f"Load: {resources['load_1min']:.2f} (1m), "
                f"{resources['load_5min']:.2f} (5m), "
                f"{resources['load_15min']:.2f} (15m)"
            )
        
        # Uptime
        if 'uptime_hours' in resources:
            self.log_to_file(SYSTEM_LOG, f"Uptime: {resources['uptime_hours']:.2f} hours")
        
        self.log_to_file(SYSTEM_LOG, "")
    
    def log_processes(self):
        """Log top processes"""
        self.log_to_file(PROCESSES_LOG, "=== Top Processes (by CPU) ===")
        
        processes = self.get_top_processes()
        for proc in processes:
            if proc:
                self.log_to_file(PROCESSES_LOG, proc)
        
        self.log_to_file(PROCESSES_LOG, "")
    
    def log_summary(self):
        """Log brief summary to summary log"""
        self.log_counter += 1
        
        # Get key metrics for summary
        resources = self.get_system_resources()
        thermal = self.get_thermal_info()
        
        summary_parts = [f"Cycle {self.log_counter}"]
        
        if 'mem_total_kb' in resources and 'mem_available_kb' in resources:
            mem_used_pct = (1 - resources['mem_available_kb'] / resources['mem_total_kb']) * 100
            summary_parts.append(f"Mem:{mem_used_pct:.0f}%")
        
        if 'load_1min' in resources:
            summary_parts.append(f"Load:{resources['load_1min']:.1f}")
        
        if thermal:
            # Get first thermal zone temp
            first_temp = list(thermal.values())[0] if thermal else "N/A"
            summary_parts.append(f"Temp:{first_temp}")
        
        self.log_to_file(SUMMARY_LOG, " | ".join(summary_parts))
    
    def run_monitoring_cycle(self):
        """Run one complete monitoring cycle"""
        try:
            # Log to all files
            self.log_hardware_metrics()
            self.log_kernel_messages()
            self.log_system_resources()
            self.log_processes()
            self.log_summary()
            
            # Periodically check disk space (every 20 cycles = 1200 seconds)
            if self.log_counter % 20 == 0:
                self.check_disk_space()
                
        except Exception as e:
            self.log_to_file(SUMMARY_LOG, f"ERROR in monitoring cycle: {e}")
            print(f"Error in monitoring cycle: {e}", file=sys.stderr)
    
    def run(self):
        """Main monitoring loop"""
        print(f"Freeze Monitor started at {self.get_timestamp()}")
        print(f"Logging to {LOG_DIR}")
        print(f"Log interval: {LOG_INTERVAL} seconds")
        
        # Log startup
        self.log_to_file(SUMMARY_LOG, "=== Freeze Monitor Started ===")
        self.log_to_file(SUMMARY_LOG, f"Start time: {self.start_time.isoformat()}")
        self.log_to_file(SUMMARY_LOG, f"Log interval: {LOG_INTERVAL} seconds")
        self.log_to_file(SUMMARY_LOG, "")
        
        try:
            while True:
                self.run_monitoring_cycle()
                time.sleep(LOG_INTERVAL)
                
        except KeyboardInterrupt:
            print("\nMonitoring stopped by user")
            self.log_to_file(SUMMARY_LOG, "=== Freeze Monitor Stopped (KeyboardInterrupt) ===")
        except Exception as e:
            print(f"Fatal error: {e}", file=sys.stderr)
            self.log_to_file(SUMMARY_LOG, f"=== Freeze Monitor Crashed: {e} ===")
            raise


def main():
    """Main entry point"""
    # Create log directory if it doesn't exist
    try:
        LOG_DIR.mkdir(parents=True, exist_ok=True)
    except Exception as e:
        print(f"Error creating log directory {LOG_DIR}: {e}", file=sys.stderr)
        sys.exit(1)
    
    # Check if we can write to log directory
    if not os.access(LOG_DIR, os.W_OK):
        print(f"Error: No write permission to {LOG_DIR}", file=sys.stderr)
        print("This script should be run as root or with sudo", file=sys.stderr)
        sys.exit(1)
    
    # Start monitoring
    monitor = FreezeMonitor()
    monitor.run()


if __name__ == "__main__":
    main()

