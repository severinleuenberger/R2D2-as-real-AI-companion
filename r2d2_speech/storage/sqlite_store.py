"""
SQLite Storage Layer
====================

Persistent storage for Realtime API sessions and messages.

Database: ~/dev/r2d2/r2d2_speech/data/conversations.db

Tables:
- sessions: Realtime API session metadata
- messages: User and assistant messages with transcripts
"""

import sqlite3
import json
from datetime import datetime
from pathlib import Path
from typing import Dict, List, Optional, Any


def init_db(db_path: str) -> None:
    """
    Initialize database and create tables if they don't exist.
    
    Args:
        db_path: Path to SQLite database file
    """
    # Ensure directory exists
    db_file = Path(db_path)
    db_file.parent.mkdir(parents=True, exist_ok=True)
    
    # Connect and create tables
    conn = sqlite3.connect(db_path)
    cursor = conn.cursor()
    
    # Create sessions table
    cursor.execute("""
        CREATE TABLE IF NOT EXISTS sessions (
            session_id TEXT PRIMARY KEY,
            started_at TIMESTAMP NOT NULL,
            metadata_json TEXT
        )
    """)
    
    # Create messages table
    cursor.execute("""
        CREATE TABLE IF NOT EXISTS messages (
            id INTEGER PRIMARY KEY AUTOINCREMENT,
            session_id TEXT NOT NULL,
            role TEXT NOT NULL,
            item_id TEXT,
            response_id TEXT,
            text TEXT NOT NULL,
            created_at TIMESTAMP NOT NULL,
            raw_event_json TEXT,
            FOREIGN KEY (session_id) REFERENCES sessions(session_id)
        )
    """)
    
    # Create index for faster queries
    cursor.execute("""
        CREATE INDEX IF NOT EXISTS idx_messages_session_id 
        ON messages(session_id)
    """)
    
    cursor.execute("""
        CREATE INDEX IF NOT EXISTS idx_messages_created_at 
        ON messages(created_at)
    """)
    
    conn.commit()
    conn.close()


def create_session(
    db_path: str,
    session_id: str,
    metadata: Optional[Dict[str, Any]] = None
) -> None:
    """
    Create a new session record.
    
    Args:
        db_path: Path to SQLite database file
        session_id: Unique session identifier
        metadata: Optional session metadata (model, voice, config, etc.)
    """
    conn = sqlite3.connect(db_path)
    cursor = conn.cursor()
    
    metadata_json = json.dumps(metadata) if metadata else None
    started_at = datetime.now().isoformat()
    
    cursor.execute(
        """
        INSERT INTO sessions (session_id, started_at, metadata_json)
        VALUES (?, ?, ?)
        """,
        (session_id, started_at, metadata_json)
    )
    
    conn.commit()
    conn.close()


def insert_message(
    db_path: str,
    session_id: str,
    role: str,
    item_id: Optional[str],
    response_id: Optional[str],
    text: str,
    raw_event: Optional[Dict[str, Any]] = None
) -> int:
    """
    Insert a message immediately when event arrives.
    
    Args:
        db_path: Path to SQLite database file
        session_id: Session identifier
        role: Message role ('user' or 'assistant')
        item_id: Item ID from conversation.item.id (for user messages)
        response_id: Response ID from response.id (for assistant messages)
        text: Transcript text
        raw_event: Optional full event JSON for debugging
        
    Returns:
        Message ID (row ID)
    """
    conn = sqlite3.connect(db_path)
    cursor = conn.cursor()
    
    created_at = datetime.now().isoformat()
    raw_event_json = json.dumps(raw_event) if raw_event else None
    
    cursor.execute(
        """
        INSERT INTO messages (
            session_id, role, item_id, response_id, 
            text, created_at, raw_event_json
        )
        VALUES (?, ?, ?, ?, ?, ?, ?)
        """,
        (session_id, role, item_id, response_id, text, created_at, raw_event_json)
    )
    
    message_id = cursor.lastrowid
    conn.commit()
    conn.close()
    
    return message_id


def get_session_messages(
    db_path: str,
    session_id: str
) -> List[Dict[str, Any]]:
    """
    Query all messages for a session.
    
    Args:
        db_path: Path to SQLite database file
        session_id: Session identifier
        
    Returns:
        List of message dictionaries ordered by created_at
    """
    conn = sqlite3.connect(db_path)
    conn.row_factory = sqlite3.Row  # Enable dict-like access
    cursor = conn.cursor()
    
    cursor.execute(
        """
        SELECT 
            id, session_id, role, item_id, response_id,
            text, created_at, raw_event_json
        FROM messages
        WHERE session_id = ?
        ORDER BY created_at ASC
        """,
        (session_id,)
    )
    
    rows = cursor.fetchall()
    conn.close()
    
    # Convert to list of dicts
    messages = []
    for row in rows:
        message = {
            "id": row["id"],
            "session_id": row["session_id"],
            "role": row["role"],
            "item_id": row["item_id"],
            "response_id": row["response_id"],
            "text": row["text"],
            "created_at": row["created_at"],
        }
        
        # Parse raw_event_json if present
        if row["raw_event_json"]:
            try:
                message["raw_event"] = json.loads(row["raw_event_json"])
            except json.JSONDecodeError:
                message["raw_event"] = None
        
        messages.append(message)
    
    return messages


def get_all_sessions(db_path: str) -> List[Dict[str, Any]]:
    """
    Get all sessions from the database.
    
    Args:
        db_path: Path to SQLite database file
        
    Returns:
        List of session dictionaries
    """
    conn = sqlite3.connect(db_path)
    conn.row_factory = sqlite3.Row
    cursor = conn.cursor()
    
    cursor.execute(
        """
        SELECT session_id, started_at, metadata_json
        FROM sessions
        ORDER BY started_at DESC
        """
    )
    
    rows = cursor.fetchall()
    conn.close()
    
    sessions = []
    for row in rows:
        session = {
            "session_id": row["session_id"],
            "started_at": row["started_at"],
        }
        
        if row["metadata_json"]:
            try:
                session["metadata"] = json.loads(row["metadata_json"])
            except json.JSONDecodeError:
                session["metadata"] = None
        
        sessions.append(session)
    
    return sessions


if __name__ == "__main__":
    # Test database operations
    import tempfile
    import os
    
    # Use temporary database for testing
    test_db = os.path.join(tempfile.gettempdir(), "test_conversations.db")
    
    print(f"Testing SQLite store with: {test_db}")
    
    # Initialize database
    init_db(test_db)
    print("✓ Database initialized")
    
    # Create session
    session_id = "test-session-001"
    create_session(
        test_db,
        session_id,
        metadata={"model": "gpt-4o-realtime-preview", "voice": "alloy"}
    )
    print(f"✓ Session created: {session_id}")
    
    # Insert user message
    msg_id_1 = insert_message(
        test_db,
        session_id,
        role="user",
        item_id="item_001",
        response_id=None,
        text="Hello, how are you?",
        raw_event={"type": "test_event"}
    )
    print(f"✓ User message inserted: ID={msg_id_1}")
    
    # Insert assistant message
    msg_id_2 = insert_message(
        test_db,
        session_id,
        role="assistant",
        item_id=None,
        response_id="resp_001",
        text="I'm doing well, thank you!",
        raw_event={"type": "test_event"}
    )
    print(f"✓ Assistant message inserted: ID={msg_id_2}")
    
    # Query messages
    messages = get_session_messages(test_db, session_id)
    print(f"✓ Retrieved {len(messages)} messages")
    
    for msg in messages:
        print(f"  - {msg['role']}: {msg['text']}")
    
    # Cleanup
    os.remove(test_db)
    print("✓ Test database cleaned up")

