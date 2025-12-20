import sys
from pathlib import Path
import os

class PersonConfig:
    """Helper class to query person registry for ROS nodes."""
    
    _registry = None
    _default_person = None
    
    @classmethod
    def _init_registry(cls):
        """Initialize registry connection."""
        if cls._registry is None:
            # Add person registry to path
            # This path logic assumes standard layout: ~/dev/r2d2
            repo_root = Path.home() / 'dev' / 'r2d2'
            registry_path = repo_root / 'tests' / 'face_recognition'
            
            if str(registry_path) not in sys.path:
                sys.path.insert(0, str(registry_path))
                
            try:
                from person_registry import PersonRegistry
                cls._registry = PersonRegistry()
            except ImportError:
                print(f"Error: Could not import PersonRegistry from {registry_path}")
                return None
            except Exception as e:
                print(f"Error initializing PersonRegistry: {e}")
                return None
                
        return cls._registry
    
    @classmethod
    def get_default_person(cls):
        """Get default person (first registered person)."""
        if cls._default_person is None:
            registry = cls._init_registry()
            if registry:
                persons = registry.list_persons()
                # Sort by created_at to get the first one if needed, 
                # but list_persons default order is created_at DESC usually
                # We want the oldest/first registered usually, or just the first in list
                if persons:
                    cls._default_person = persons[-1] # Assuming DESC order, last is oldest/first
                    # Wait, list_persons query says ORDER BY created_at DESC
                    # So persons[-1] is the OLDEST (first created)
                    # Let's verify behavior. If list is [Newest...Oldest], then [-1] is correct.
        return cls._default_person
    
    @classmethod
    def get_person_name(cls, person_name='target_person'):
        """
        Get resolved person name.
        If 'target_person', resolves to default registered person name.
        Otherwise returns provided name.
        """
        if person_name == 'target_person':
            person = cls.get_default_person()
            if person:
                return person['name']
        return person_name

    @classmethod
    def get_face_model_path(cls, person_name='target_person'):
        """Get face recognition model path for person."""
        registry = cls._init_registry()
        if not registry:
            return None
            
        if person_name == 'target_person':
            person = cls.get_default_person()
        else:
            person = registry.get_person(person_name)
            
        if person:
            return person.get('face_model_path')
        return None
    
    @classmethod
    def get_gesture_model_path(cls, person_name='target_person'):
        """Get gesture recognition model path for person."""
        registry = cls._init_registry()
        if not registry:
            return None
            
        if person_name == 'target_person':
            person = cls.get_default_person()
        else:
            person = registry.get_person(person_name)
            
        if person:
            return person.get('gesture_model_path')
        return None

