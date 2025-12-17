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
    
    def __init__(self, db_path: str = None):
        """
        Initialize person registry.
        
        Args:
            db_path: Path to SQLite database. Defaults to data/persons.db
        """
        if db_path is None:
            base_dir = Path(__file__).parent / 'data'
            base_dir.mkdir(exist_ok=True)
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
        base_dir = Path(__file__).parent / 'data'
        
        face_models_dir = base_dir / 'face_recognition' / 'models'
        gesture_models_dir = base_dir / 'gesture_recognition' / 'models'
        
        migrated = 0
        skipped = 0
        errors = 0
        
        # Scan face recognition models
        if face_models_dir.exists():
            for model_file in face_models_dir.glob('*.yml'):
                person_name = model_file.stem
                
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

