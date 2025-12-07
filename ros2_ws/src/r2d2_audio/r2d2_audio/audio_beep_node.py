#!/usr/bin/env python3
"""
R2D2 Audio Beep Node

A simple ROS 2 node that provides audio beep services and publishes audio status.
This is a basic implementation for Phase 1 testing.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32, UInt32
from r2d2_audio import AudioBeepGenerator, AudioStatus


class AudioBeepNode(Node):
    """ROS 2 node for audio output and beep generation"""
    
    def __init__(self):
        super().__init__('audio_beep_node')
        
        self.get_logger().info("R2D2 Audio Beep Node starting...")
        
        # Store audio status
        self.audio_status = AudioStatus()
        
        # Publishers
        self.status_pub = self.create_publisher(
            String, '/r2d2/audio/status', 10
        )
        self.beep_count_pub = self.create_publisher(
            UInt32, '/r2d2/audio/beep_count', 10
        )
        self.last_frequency_pub = self.create_publisher(
            Float32, '/r2d2/audio/last_frequency', 10
        )
        
        # Simple parameter for ALSA device
        self.declare_parameter('device', 'hw:1,0')
        self.audio_status.device = self.get_parameter('device').value
        
        self.get_logger().info(f"Audio device: {self.audio_status.device}")
        self.publish_status("initialized")
    
    def publish_status(self, status: str):
        """Publish current audio status"""
        msg = String()
        msg.data = status
        self.status_pub.publish(msg)
        self.get_logger().debug(f"Audio status: {status}")
    
    def publish_beep_count(self):
        """Publish beep count"""
        msg = UInt32()
        msg.data = self.audio_status.beep_count
        self.beep_count_pub.publish(msg)
    
    def publish_frequency(self):
        """Publish last frequency"""
        msg = Float32()
        msg.data = self.audio_status.last_beep_frequency
        self.last_frequency_pub.publish(msg)
    
    def play_beep(
        self,
        frequency: float = 1000.0,
        duration: float = 0.5,
        volume: float = 0.5,
    ) -> bool:
        """
        Play a beep sound
        
        Args:
            frequency: Tone frequency in Hz
            duration: Duration in seconds
            volume: Volume 0.0-1.0
        
        Returns:
            True if successful
        """
        self.get_logger().info(
            f"Playing beep: {frequency}Hz, {duration}s, volume {volume*100:.0f}%"
        )
        
        success = AudioBeepGenerator.beep(
            frequency=frequency,
            duration=duration,
            volume=volume,
            device=self.audio_status.device,
        )
        
        if success:
            self.audio_status.update_beep(frequency, duration, volume)
            self.publish_beep_count()
            self.publish_frequency()
            self.get_logger().info("Beep played successfully")
            self.publish_status(f"beep_ok_{frequency}Hz")
        else:
            self.get_logger().error("Failed to play beep")
            self.publish_status("beep_failed")
        
        return success


def main(args=None):
    """Main entry point for the node"""
    rclpy.init(args=args)
    
    node = AudioBeepNode()
    
    # Test: Play initial startup beep
    node.get_logger().info("Playing startup beep...")
    node.play_beep(frequency=800.0, duration=0.3, volume=0.5)
    
    # Spin the node (will stay running to serve subscribers/services)
    rclpy.spin(node)
    
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
