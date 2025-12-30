#!/usr/bin/env python3
"""
Person Registry - Central management for person entities

Manages persons as first-class entities with links to face recognition models,
gesture recognition models, and future Google account integration.

Uses explicit SQL column queries for forward compatibility.
"""

import sqlite3
import uuid
from datetime import datetime
from pathlib import Path
from typing import Optional, List, Dict
import json


class PersonRegistry:
    """
    SQLite-based person registry with explicit column queries.
    
    Avoids SELECT * to ensure forward compatibility when new columns are added.
    """
    
    # SQL Queries - Explicit columns for forward compatibility
    QUERY_GET_PERSON = """
        SELECT id, name, display_name, created_at, updated_at,
               face_model_path, gesture_model_path, google_account, metadata
        FROM persons WHERE name = ?
    """
    
    QUERY_GET_PERSON_BY_ID = """
        SELECT id, name, display_name, created_at, updated_at,
               face_model_path, gesture_model_path, google_account, metadata
        FROM persons WHERE id = ?
    """
    
    QUERY_LIST_PERSONS = """
        SELECT id, name, display_name, created_at, face_model_path, 
               gesture_model_path
        FROM persons ORDER BY created_at DESC
    """
    
    QUERY_INSERT_PERSON = """
        INSERT INTO persons (id, name, display_name, created_at, updated_at,
                            face_model_path, gesture_model_path, google_account, metadata)
        VALUES (?, ?, ?, ?, ?, NULL, NULL, NULL, NULL)
    """
    
    QUERY_UPDATE_FACE = """
        UPDATE persons 
        SET face_model_path = ?, updated_at = ?
        WHERE id = ?
    """
    
    QUERY_UPDATE_GESTURE = """
        UPDATE persons 
        SET gesture_model_path = ?, updated_at = ?
        WHERE id = ?
    """
    
    QUERY_DELETE_PERSON = """
        DELETE FROM persons WHERE id = ?
    """
    
    # Learning Progress Queries
    QUERY_INSERT_LEARNING_TOPIC = """
        INSERT INTO learning_topics (id, person_id, category, subcategory, topic,
                                     understanding_level, bi_analogy, code_reference,
                                     first_encountered, last_reviewed, times_reviewed)
        VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, 1)
    """
    
    QUERY_GET_LEARNING_TOPIC = """
        SELECT id, person_id, category, subcategory, topic, understanding_level,
               bi_analogy, code_reference, first_encountered, last_reviewed, times_reviewed
        FROM learning_topics WHERE person_id = ? AND topic = ?
    """
    
    QUERY_UPDATE_LEARNING_TOPIC = """
        UPDATE learning_topics 
        SET understanding_level = ?, last_reviewed = ?, times_reviewed = times_reviewed + 1
        WHERE id = ?
    """
    
    QUERY_LIST_LEARNING_TOPICS = """
        SELECT id, person_id, category, subcategory, topic, understanding_level,
               bi_analogy, first_encountered, last_reviewed, times_reviewed
        FROM learning_topics WHERE person_id = ?
        ORDER BY last_reviewed DESC
    """
    
    QUERY_LIST_LEARNING_BY_CATEGORY = """
        SELECT id, person_id, category, subcategory, topic, understanding_level,
               bi_analogy, first_encountered, last_reviewed, times_reviewed
        FROM learning_topics WHERE person_id = ? AND category = ?
        ORDER BY subcategory, topic
    """
    
    QUERY_INSERT_LEARNING_SESSION = """
        INSERT INTO learning_sessions (id, person_id, started_at, ended_at, summary, topics_learned)
        VALUES (?, ?, ?, NULL, NULL, NULL)
    """
    
    QUERY_UPDATE_LEARNING_SESSION = """
        UPDATE learning_sessions
        SET ended_at = ?, summary = ?, topics_learned = ?
        WHERE id = ?
    """
    
    QUERY_LIST_LEARNING_SESSIONS = """
        SELECT id, person_id, started_at, ended_at, summary, topics_learned
        FROM learning_sessions WHERE person_id = ?
        ORDER BY started_at DESC LIMIT ?
    """
    
    def __init__(self, db_path: str = None):
        """
        Initialize person registry.
        
        Args:
            db_path: Path to SQLite database. Defaults to ~/dev/r2d2/data/persons.db
        """
        if db_path is None:
            # Use persistent path instead of relative path (critical for installed ROS packages)
            base_dir = Path.home() / 'dev' / 'r2d2' / 'data'
            base_dir.mkdir(parents=True, exist_ok=True)
            db_path = base_dir / 'persons.db'
        
        self.db_path = Path(db_path)
        self._init_db()
    
    def _init_db(self):
        """Initialize database schema if it doesn't exist."""
        conn = sqlite3.connect(str(self.db_path))
        cursor = conn.cursor()
        
        cursor.execute("""
            CREATE TABLE IF NOT EXISTS persons (
                id TEXT PRIMARY KEY,
                name TEXT UNIQUE NOT NULL,
                display_name TEXT,
                created_at TEXT NOT NULL,
                updated_at TEXT NOT NULL,
                face_model_path TEXT,
                gesture_model_path TEXT,
                google_account TEXT,
                metadata TEXT
            )
        """)
        
        # Learning progress tables
        cursor.execute("""
            CREATE TABLE IF NOT EXISTS learning_topics (
                id TEXT PRIMARY KEY,
                person_id TEXT NOT NULL,
                category TEXT NOT NULL,
                subcategory TEXT,
                topic TEXT NOT NULL,
                understanding_level INTEGER DEFAULT 1,
                bi_analogy TEXT,
                code_reference TEXT,
                first_encountered TEXT NOT NULL,
                last_reviewed TEXT NOT NULL,
                times_reviewed INTEGER DEFAULT 1,
                FOREIGN KEY (person_id) REFERENCES persons(id)
            )
        """)
        
        cursor.execute("""
            CREATE TABLE IF NOT EXISTS learning_sessions (
                id TEXT PRIMARY KEY,
                person_id TEXT NOT NULL,
                started_at TEXT NOT NULL,
                ended_at TEXT,
                summary TEXT,
                topics_learned TEXT,
                FOREIGN KEY (person_id) REFERENCES persons(id)
            )
        """)
        
        # Create indexes for faster queries
        cursor.execute("""
            CREATE INDEX IF NOT EXISTS idx_learning_topics_person 
            ON learning_topics(person_id)
        """)
        cursor.execute("""
            CREATE INDEX IF NOT EXISTS idx_learning_topics_category 
            ON learning_topics(person_id, category)
        """)
        cursor.execute("""
            CREATE INDEX IF NOT EXISTS idx_learning_sessions_person 
            ON learning_sessions(person_id)
        """)
        
        conn.commit()
        conn.close()
    
    def _get_connection(self) -> sqlite3.Connection:
        """Get database connection."""
        return sqlite3.connect(str(self.db_path))
    
    def _row_to_dict(self, row: tuple) -> Dict:
        """
        Convert database row to person dictionary.
        
        Explicitly maps each column to avoid breakage when schema changes.
        
        Args:
            row: Database row tuple
            
        Returns:
            Person dictionary
        """
        return {
            'id': row[0],
            'name': row[1],
            'display_name': row[2],
            'created_at': row[3],
            'updated_at': row[4],
            'face_model_path': row[5],
            'gesture_model_path': row[6],
            'google_account': row[7],
            'metadata': json.loads(row[8]) if row[8] else {}
        }
    
    def register_person(self, name: str) -> str:
        """
        Register a new person in the registry.
        
        Args:
            name: Person's name (will be normalized to lowercase)
            
        Returns:
            person_id: UUID of created person
            
        Raises:
            ValueError: If person already exists
        """
        normalized_name = name.lower()
        
        # Check if person already exists
        existing = self.get_person(normalized_name)
        if existing:
            raise ValueError(f"Person '{name}' already exists")
        
        person_id = str(uuid.uuid4())
        now = datetime.now().isoformat()
        
        conn = self._get_connection()
        cursor = conn.cursor()
        
        cursor.execute(self.QUERY_INSERT_PERSON, 
                      (person_id, normalized_name, name, now, now))
        
        conn.commit()
        conn.close()
        
        return person_id
    
    def get_person(self, name: str) -> Optional[Dict]:
        """
        Get person by name.
        
        Args:
            name: Person's name
            
        Returns:
            Person dictionary or None if not found
        """
        normalized_name = name.lower()
        
        conn = self._get_connection()
        cursor = conn.cursor()
        
        cursor.execute(self.QUERY_GET_PERSON, (normalized_name,))
        row = cursor.fetchone()
        
        conn.close()
        
        return self._row_to_dict(row) if row else None
    
    def get_person_by_id(self, person_id: str) -> Optional[Dict]:
        """
        Get person by ID.
        
        Args:
            person_id: Person's UUID
            
        Returns:
            Person dictionary or None if not found
        """
        conn = self._get_connection()
        cursor = conn.cursor()
        
        cursor.execute(self.QUERY_GET_PERSON_BY_ID, (person_id,))
        row = cursor.fetchone()
        
        conn.close()
        
        return self._row_to_dict(row) if row else None
    
    def list_persons(self) -> List[Dict]:
        """
        List all registered persons.
        
        Returns:
            List of person dictionaries (subset of fields for listing)
        """
        conn = self._get_connection()
        cursor = conn.cursor()
        
        cursor.execute(self.QUERY_LIST_PERSONS)
        rows = cursor.fetchall()
        
        conn.close()
        
        # Map to simplified dict for listing
        persons = []
        for row in rows:
            persons.append({
                'id': row[0],
                'name': row[1],
                'display_name': row[2],
                'created_at': row[3],
                'face_model_path': row[4],
                'gesture_model_path': row[5]
            })
        
        return persons
    
    def update_face_model(self, person_id: str, model_path: str):
        """
        Update person's face recognition model path.
        
        Args:
            person_id: Person's UUID
            model_path: Path to face model .yml file
        """
        now = datetime.now().isoformat()
        
        conn = self._get_connection()
        cursor = conn.cursor()
        
        cursor.execute(self.QUERY_UPDATE_FACE, (model_path, now, person_id))
        
        conn.commit()
        conn.close()
    
    def update_gesture_model(self, person_id: str, model_path: str):
        """
        Update person's gesture recognition model path.
        
        Args:
            person_id: Person's UUID
            model_path: Path to gesture model .pkl file
        """
        now = datetime.now().isoformat()
        
        conn = self._get_connection()
        cursor = conn.cursor()
        
        cursor.execute(self.QUERY_UPDATE_GESTURE, (model_path, now, person_id))
        
        conn.commit()
        conn.close()
    
    def delete_person(self, person_id: str):
        """
        Delete person from registry.
        
        Note: Does not delete model files - only registry entry.
        
        Args:
            person_id: Person's UUID
        """
        conn = self._get_connection()
        cursor = conn.cursor()
        
        cursor.execute(self.QUERY_DELETE_PERSON, (person_id,))
        
        conn.commit()
        conn.close()
    
    def auto_migrate(self) -> Dict[str, int]:
        """
        Auto-migrate existing face and gesture models into registry.
        
        Scans data directories for existing models and creates person entries.
        
        Returns:
            Dictionary with migration statistics
        """
        # Use persistent data directory instead of relative path
        base_dir = Path.home() / 'dev' / 'r2d2' / 'data'
        
        face_models_dir = base_dir / 'face_recognition' / 'models'
        gesture_models_dir = base_dir / 'gesture_recognition' / 'models'
        
        migrated = 0
        skipped = 0
        errors = 0
        
        # Scan face recognition models (.xml for LBPH, .yml for alternative formats)
        if face_models_dir.exists():
            for model_file in list(face_models_dir.glob('*.yml')) + list(face_models_dir.glob('*.xml')):
                # Extract person name (remove _lbph suffix for .xml files)
                person_name = model_file.stem.replace('_lbph', '')
                
                try:
                    # Check if person already exists
                    person = self.get_person(person_name)
                    if person:
                        # Update face model path if not set
                        if not person['face_model_path']:
                            self.update_face_model(person['id'], str(model_file))
                            migrated += 1
                        else:
                            skipped += 1
                    else:
                        # Register new person
                        person_id = self.register_person(person_name)
                        self.update_face_model(person_id, str(model_file))
                        migrated += 1
                except Exception as e:
                    print(f"Error migrating face model for {person_name}: {e}")
                    errors += 1
        
        # Scan gesture recognition models
        if gesture_models_dir.exists():
            for model_file in gesture_models_dir.glob('*_gesture_classifier.pkl'):
                person_name = model_file.stem.replace('_gesture_classifier', '')
                
                try:
                    # Check if person already exists
                    person = self.get_person(person_name)
                    if person:
                        # Update gesture model path if not set
                        if not person['gesture_model_path']:
                            self.update_gesture_model(person['id'], str(model_file))
                            migrated += 1
                        else:
                            skipped += 1
                    else:
                        # Register new person
                        person_id = self.register_person(person_name)
                        self.update_gesture_model(person_id, str(model_file))
                        migrated += 1
                except Exception as e:
                    print(f"Error migrating gesture model for {person_name}: {e}")
                    errors += 1
        
        return {
            'migrated': migrated,
            'skipped': skipped,
            'errors': errors
        }
    
    # ========================================================================
    # Learning Progress Methods
    # ========================================================================
    
    def log_learning_topic(self, person_id: str, category: str, topic: str,
                          subcategory: str = None, understanding_level: int = 1,
                          bi_analogy: str = None, code_reference: str = None) -> str:
        """
        Log a new learning topic or update existing one.
        
        Args:
            person_id: Person's UUID
            category: Main category (e.g., "ROS 2", "Python", "Hardware")
            topic: Specific topic learned
            subcategory: Optional subcategory (e.g., "Subscribers", "GPIO")
            understanding_level: 1-5 scale (1=heard of, 5=mastered)
            bi_analogy: BI concept analogy used to explain
            code_reference: File:line reference to example code
            
        Returns:
            topic_id: UUID of the learning topic entry
        """
        now = datetime.now().isoformat()
        
        conn = self._get_connection()
        cursor = conn.cursor()
        
        # Check if topic already exists for this person
        cursor.execute(self.QUERY_GET_LEARNING_TOPIC, (person_id, topic))
        existing = cursor.fetchone()
        
        if existing:
            # Update existing topic
            topic_id = existing[0]
            cursor.execute(self.QUERY_UPDATE_LEARNING_TOPIC, 
                          (understanding_level, now, topic_id))
        else:
            # Create new topic
            topic_id = str(uuid.uuid4())
            cursor.execute(self.QUERY_INSERT_LEARNING_TOPIC,
                          (topic_id, person_id, category, subcategory, topic,
                           understanding_level, bi_analogy, code_reference, now, now))
        
        conn.commit()
        conn.close()
        
        return topic_id
    
    def get_learning_topics(self, person_id: str, category: str = None) -> List[Dict]:
        """
        Get learning topics for a person.
        
        Args:
            person_id: Person's UUID
            category: Optional filter by category
            
        Returns:
            List of learning topic dictionaries
        """
        conn = self._get_connection()
        cursor = conn.cursor()
        
        if category:
            cursor.execute(self.QUERY_LIST_LEARNING_BY_CATEGORY, (person_id, category))
        else:
            cursor.execute(self.QUERY_LIST_LEARNING_TOPICS, (person_id,))
        
        rows = cursor.fetchall()
        conn.close()
        
        topics = []
        for row in rows:
            topics.append({
                'id': row[0],
                'person_id': row[1],
                'category': row[2],
                'subcategory': row[3],
                'topic': row[4],
                'understanding_level': row[5],
                'bi_analogy': row[6],
                'first_encountered': row[7],
                'last_reviewed': row[8],
                'times_reviewed': row[9]
            })
        
        return topics
    
    def get_learning_summary(self, person_id: str) -> Dict:
        """
        Get learning progress summary for a person.
        
        Args:
            person_id: Person's UUID
            
        Returns:
            Dictionary with learning statistics by category
        """
        topics = self.get_learning_topics(person_id)
        
        summary = {
            'total_topics': len(topics),
            'by_category': {},
            'by_understanding': {1: 0, 2: 0, 3: 0, 4: 0, 5: 0},
            'recent_topics': []
        }
        
        for topic in topics:
            cat = topic['category']
            if cat not in summary['by_category']:
                summary['by_category'][cat] = {'count': 0, 'avg_understanding': 0, 'topics': []}
            summary['by_category'][cat]['count'] += 1
            summary['by_category'][cat]['topics'].append(topic['topic'])
            
            level = topic['understanding_level'] or 1
            if level in summary['by_understanding']:
                summary['by_understanding'][level] += 1
        
        # Calculate averages
        for cat in summary['by_category']:
            cat_topics = [t for t in topics if t['category'] == cat]
            if cat_topics:
                avg = sum(t['understanding_level'] or 1 for t in cat_topics) / len(cat_topics)
                summary['by_category'][cat]['avg_understanding'] = round(avg, 1)
        
        # Get 5 most recent topics
        summary['recent_topics'] = [t['topic'] for t in topics[:5]]
        
        return summary
    
    def start_learning_session(self, person_id: str) -> str:
        """
        Start a new learning session.
        
        Args:
            person_id: Person's UUID
            
        Returns:
            session_id: UUID of the new session
        """
        session_id = str(uuid.uuid4())
        now = datetime.now().isoformat()
        
        conn = self._get_connection()
        cursor = conn.cursor()
        
        cursor.execute(self.QUERY_INSERT_LEARNING_SESSION,
                      (session_id, person_id, now))
        
        conn.commit()
        conn.close()
        
        return session_id
    
    def end_learning_session(self, session_id: str, summary: str = None, 
                            topics_learned: List[str] = None):
        """
        End a learning session with summary.
        
        Args:
            session_id: Session UUID
            summary: Optional session summary
            topics_learned: Optional list of topic IDs learned
        """
        now = datetime.now().isoformat()
        topics_json = json.dumps(topics_learned) if topics_learned else None
        
        conn = self._get_connection()
        cursor = conn.cursor()
        
        cursor.execute(self.QUERY_UPDATE_LEARNING_SESSION,
                      (now, summary, topics_json, session_id))
        
        conn.commit()
        conn.close()
    
    def get_learning_sessions(self, person_id: str, limit: int = 10) -> List[Dict]:
        """
        Get recent learning sessions for a person.
        
        Args:
            person_id: Person's UUID
            limit: Maximum number of sessions to return
            
        Returns:
            List of session dictionaries
        """
        conn = self._get_connection()
        cursor = conn.cursor()
        
        cursor.execute(self.QUERY_LIST_LEARNING_SESSIONS, (person_id, limit))
        rows = cursor.fetchall()
        
        conn.close()
        
        sessions = []
        for row in rows:
            sessions.append({
                'id': row[0],
                'person_id': row[1],
                'started_at': row[2],
                'ended_at': row[3],
                'summary': row[4],
                'topics_learned': json.loads(row[5]) if row[5] else []
            })
        
        return sessions

