#!/usr/bin/env python3
"""
Initialize Person Registry
--------------------------
Creates the person registry database and registers 'severin' as the default person
linked to existing models.
"""

import sys
import os
from pathlib import Path

# Add tests/face_recognition to path to import PersonRegistry
# This is a temporary measure until we move PersonRegistry to a proper package
repo_root = Path.home() / 'dev' / 'r2d2'
sys.path.insert(0, str(repo_root / 'tests' / 'face_recognition'))

try:
    from person_registry import PersonRegistry
except ImportError:
    print("Error: Could not import PersonRegistry. Check paths.")
    sys.exit(1)

def main():
    print("Initializing Person Registry...")
    
    # Initialize registry (creates DB if missing)
    registry = PersonRegistry()
    print(f"Database initialized at: {repo_root / 'data' / 'persons.db'}")
    
    # Check if severin exists
    person_name = "severin"
    existing_person = registry.get_person(person_name)
    
    if existing_person:
        print(f"Person '{person_name}' already exists. ID: {existing_person['id']}")
        person_id = existing_person['id']
    else:
        print(f"Registering new person: '{person_name}'")
        try:
            person_id = registry.register_person(person_name)
            print(f"Successfully registered '{person_name}'. ID: {person_id}")
        except Exception as e:
            print(f"Error registering person: {e}")
            sys.exit(1)
            
    # Link models
    face_model = repo_root / 'data' / 'face_recognition' / 'models' / 'severin_lbph.xml'
    gesture_model = repo_root / 'data' / 'gesture_recognition' / 'models' / 'severin_gesture_classifier.pkl'
    
    if face_model.exists():
        print(f"Linking face model: {face_model}")
        registry.update_face_model(person_id, str(face_model))
    else:
        print(f"Warning: Face model not found at {face_model}")
        
    if gesture_model.exists():
        print(f"Linking gesture model: {gesture_model}")
        registry.update_gesture_model(person_id, str(gesture_model))
    else:
        print(f"Warning: Gesture model not found at {gesture_model}")
        
    print("\nInitialization complete!")
    
    # Verification
    person = registry.get_person(person_name)
    print("\nVerification:")
    print(f"Name: {person['display_name']}")
    print(f"Face Model: {'OK' if person['face_model_path'] else 'Missing'}")
    print(f"Gesture Model: {'OK' if person['gesture_model_path'] else 'Missing'}")

if __name__ == "__main__":
    main()

