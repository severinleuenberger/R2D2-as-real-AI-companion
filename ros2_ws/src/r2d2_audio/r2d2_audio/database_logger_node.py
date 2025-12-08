#!/usr/bin/env python3
"""
Database Logger Node - Persists person recognition events.

CURRENTLY: Structure only - no actual database operations.
FUTURE: Will log to SQLite database for conversation history and analytics.

Subscribes to /r2d2/audio/person_status (JSON String messages) and logs:
  - Recognition events (person_identity, status, timestamp, confidence)
  - Duration tracking (how long in each state)
  - Event transitions (RED→BLUE→RED, etc.)

Integration points:
  - STT-LLM-TTS: Query recognition history to build conversation context
  - Analytics: Analyze engagement patterns per person
  - Memory: Associate responses with specific people and time windows
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from datetime import datetime
from pathlib import Path
from typing import Optional
import json


class DatabaseLoggerNode(Node):
    """
    Logs person recognition events for conversation history and analytics.
    
    Database schema (future implementation):
    - recognition_events: timestamp, person_identity, status, confidence, duration
    - conversation_context: timestamp, person_identity, status, transcript, response
    """
    
    def __init__(self):
        super().__init__('database_logger')
        
        self.get_logger().info("Database Logger Node initializing...")
        
        # Declare parameters
        self.declare_parameter('db_path', '/home/severin/dev/r2d2/r2d2_conversations.db')
        self.declare_parameter('enabled', True)
        self.declare_parameter('log_to_console', True)  # For now, just log to console
        
        db_path = self.get_parameter('db_path').value
        self.db_path = Path(db_path)
        self.log_to_console = self.get_parameter('log_to_console').value
        
        # State tracking
        self.last_person = "no_person"
        self.last_status = "blue"
        self.event_counter = 0
        
        # Subscribe to status (JSON String messages)
        self.status_sub = self.create_subscription(
            String,
            '/r2d2/audio/person_status',
            self.status_callback,
            qos_profile=rclpy.qos.QoSProfile(depth=10)
        )
        
        self.get_logger().info(
            f"Database Logger initialized:\n"
            f"  Database path: {self.db_path}\n"
            f"  Mode: {'Console logging' if self.log_to_console else 'Database'}\n"
            f"  Status: Structure only (database schema ready for implementation)\n"
            f"  Enabled: {self.get_parameter('enabled').value}"
        )
    
    def status_callback(self, msg: String):
        """
        Log status changes to database (structure ready for implementation).
        
        Args:
            msg: String message containing JSON PersonStatus data
        """
        if not self.get_parameter('enabled').value:
            return
        
        try:
            status_data = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().error(f"Failed to parse status JSON")
            return
        
        self.event_counter += 1
        
        # Check for state transitions
        status_changed = (status_data.get('status') != self.last_status)
        person_changed = (status_data.get('person_identity') != self.last_person)
        
        # Get timestamp
        timestamp_sec = status_data.get('timestamp_sec', 0)
        timestamp_nanosec = status_data.get('timestamp_nanosec', 0)
        timestamp = datetime.fromtimestamp(
            timestamp_sec + timestamp_nanosec / 1e9
        ).isoformat()
        
        if self.log_to_console:
            # Console logging
            transition = ""
            if status_changed:
                transition = f" [TRANSITION: {self.last_status.upper()} → {status_data.get('status').upper()}]"
            
            person_id = status_data.get('person_identity', 'unknown')
            status = status_data.get('status', 'unknown')
            confidence = status_data.get('confidence', 0)
            duration = status_data.get('duration_in_state', 0)
            
            self.get_logger().info(
                f"📝 Event #{self.event_counter}: {person_id} ({status}) "
                f"| Confidence: {confidence*100:.0f}% | Duration: {duration:.1f}s "
                f"| {timestamp}{transition}"
            )
        
        # Update state
        self.last_person = status_data.get('person_identity', 'no_person')
        self.last_status = status_data.get('status', 'blue')
        
        # FUTURE: Database logging would go here
        # self._log_to_database(timestamp, status_data)
    
    def _log_to_database(self, timestamp: str, status_data: dict):
        """
        FUTURE IMPLEMENTATION: Log event to SQLite database.
        
        Args:
            timestamp: ISO format timestamp
            status_data: Dictionary with PersonStatus data
        """
        # TODO: Implement SQLite logging
        # import sqlite3
        # conn = sqlite3.connect(self.db_path)
        # cursor = conn.cursor()
        # cursor.execute('''
        #     INSERT INTO recognition_events
        #     (timestamp, person_identity, status, confidence, duration_in_state)
        #     VALUES (?, ?, ?, ?, ?)
        # ''', (
        #     timestamp,
        #     status_data.get('person_identity'),
        #     status_data.get('status'),
        #     status_data.get('confidence'),
        #     status_data.get('duration_in_state')
        # ))
        # conn.commit()
        # conn.close()
        pass


def main(args=None):
    rclpy.init(args=args)
    node = DatabaseLoggerNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Database Logger shutting down...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
