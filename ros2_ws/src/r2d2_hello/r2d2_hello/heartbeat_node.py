import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from datetime import datetime
import subprocess
import json
import re
import os
import threading

class HeartbeatNode(Node):
    def __init__(self):
        super().__init__('r2d2_heartbeat_node')
        self.publisher = self.create_publisher(String, '/r2d2/heartbeat', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.get_logger().info('HeartbeatNode gestartet ❤️')
        self.current_metrics = {
            'cpu_percent': 0.0,
            'gpu_percent': 0.0,
            'temperature_c': 0.0
        }
        self.metrics_lock = threading.Lock()
        # Start background thread to collect metrics
        self.metrics_thread = threading.Thread(target=self._collect_metrics_loop, daemon=True)
        self.metrics_thread.start()

    def _collect_metrics_loop(self):
        """Background thread to collect metrics periodically"""
        import time
        while True:
            try:
                metrics = self._collect_metrics()
                with self.metrics_lock:
                    self.current_metrics = metrics
            except Exception as e:
                self.get_logger().debug(f'Error collecting metrics: {e}')
            time.sleep(1.0)  # Update every second
    
    def _collect_metrics(self):
        """Collect system metrics using tegrastats and thermal zones"""
        metrics = {
            'cpu_percent': 0.0,
            'gpu_percent': 0.0,
            'temperature_c': 0.0
        }
        
        # Try to get metrics from tegrastats (non-blocking with timeout)
        try:
            # Run tegrastats with timeout, capture first line
            process = subprocess.Popen(
                ['sh', '-c', 'timeout 1.5 tegrastats --interval 1000 2>&1 | head -1'],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True
            )
            
            try:
                stdout, stderr = process.communicate(timeout=2.0)
                if stdout:
                    output = stdout.strip()
                    
                    # Parse CPU usage: CPU [19%@729,28%@729,...]
                    cpu_match = re.search(r'CPU \[([^\]]+)\]', output)
                    if cpu_match:
                        cpu_str = cpu_match.group(1)
                        # Extract all percentages: "19%@729,28%@729,..."
                        cpu_percentages = re.findall(r'(\d+)%@', cpu_str)
                        if cpu_percentages:
                            cpu_values = [float(p) for p in cpu_percentages]
                            metrics['cpu_percent'] = sum(cpu_values) / len(cpu_values)
                    
                    # Parse GPU usage: GR3D_FREQ 0%
                    gpu_match = re.search(r'GR3D_FREQ\s+(\d+)%', output)
                    if gpu_match:
                        metrics['gpu_percent'] = float(gpu_match.group(1))
                    
                    # Parse CPU temperature: cpu@41.562C
                    temp_match = re.search(r'cpu@([\d.]+)C', output)
                    if temp_match:
                        metrics['temperature_c'] = float(temp_match.group(1))
            except subprocess.TimeoutExpired:
                process.kill()
                process.communicate()
            except Exception as e:
                self.get_logger().debug(f'tegrastats error: {e}')
                    
        except FileNotFoundError:
            pass
        except Exception as e:
            self.get_logger().debug(f'tegrastats unavailable: {e}')
        
        # Fallback to thermal zones for temperature if not set
        if metrics['temperature_c'] == 0.0:
            try:
                thermal_zones = ['/sys/class/thermal/thermal_zone0/temp',
                               '/sys/class/thermal/thermal_zone1/temp']
                for zone in thermal_zones:
                    if os.path.exists(zone):
                        with open(zone, 'r') as f:
                            temp_millidegrees = int(f.read().strip())
                            temp_c = temp_millidegrees / 1000.0
                            if temp_c > 0:  # Valid reading
                                metrics['temperature_c'] = temp_c
                                break
            except Exception as e2:
                pass
        
        # Use last known values if current read failed
        with self.metrics_lock:
            if metrics['cpu_percent'] == 0.0 and self.current_metrics['cpu_percent'] > 0:
                metrics['cpu_percent'] = self.current_metrics['cpu_percent']
            if metrics['gpu_percent'] == 0.0 and self.current_metrics['gpu_percent'] > 0:
                metrics['gpu_percent'] = self.current_metrics['gpu_percent']
            if metrics['temperature_c'] == 0.0 and self.current_metrics['temperature_c'] > 0:
                metrics['temperature_c'] = self.current_metrics['temperature_c']
        
        return metrics
    
    def get_system_metrics(self):
        """Get current cached metrics (non-blocking)"""
        with self.metrics_lock:
            return self.current_metrics.copy()

    def timer_callback(self):
        # Get system metrics
        metrics = self.get_system_metrics()
        
        # Create JSON message
        heartbeat_data = {
            'timestamp': datetime.now().isoformat(timespec="seconds"),
            'status': 'running',
            'cpu_percent': round(metrics['cpu_percent'], 1),
            'gpu_percent': round(metrics['gpu_percent'], 1),
            'temperature_c': round(metrics['temperature_c'], 1)
        }
        
        msg = String()
        msg.data = json.dumps(heartbeat_data)
        self.publisher.publish(msg)
        self.get_logger().debug(f'Published heartbeat: {msg.data}')

def main(args=None):
    rclpy.init(args=args)
    node = HeartbeatNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
