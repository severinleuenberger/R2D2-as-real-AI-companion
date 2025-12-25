#!/usr/bin/env python3
"""
R2D2 Training Manager - Face & Gesture Recognition System

Purpose:
  Central hub for training and testing face recognition and gesture recognition models.
  Supports multiple people, flexible workflows, and clear instructions.
  Calls modular scripts for both face and gesture training.

Features:
  FACE RECOGNITION:
  • Train multiple people with LBPH face recognizer
  • Interactive capture with stage-by-stage guidance
  • Test models at different distances
  • Calls: _capture_module.py, _train_module.py, _test_module.py
  
  GESTURE RECOGNITION:
  • Train person-specific gesture classifiers (index finger up, fist)
  • MediaPipe Hands for landmark extraction
  • SVM classifier for gesture recognition
  • Calls: _gesture_capture_module.py, _gesture_train_module.py, _gesture_test_module.py

  GENERAL:
  • Model management (list, delete, info)
  • Progress tracking
  • Person-specific training data organization

Usage:
  source ~/depthai_env/bin/activate
  export OPENBLAS_CORETYPE=ARMV8
  python3 train_manager.py

Author: R2D2 Perception Pipeline
Date: December 6, 2025 (Face Recognition), December 17, 2025 (Gesture Recognition)
"""

import subprocess
import sys
import os
from pathlib import Path
from datetime import datetime
from person_registry import PersonRegistry


class TrainingManager:
    """Central hub for face recognition and gesture recognition training and testing."""
    
    def __init__(self):
        """Initialize training manager."""
        # Base directory for training data
        self.base_dir = Path.home() / 'dev' / 'r2d2' / 'data' / 'face_recognition'
        self.base_dir.mkdir(parents=True, exist_ok=True)
        
        # Script directory
        self.script_dir = Path.home() / 'dev' / 'r2d2' / 'tests' / 'face_recognition'
        
        # Models directory
        self.models_dir = self.base_dir / 'models'
        self.models_dir.mkdir(parents=True, exist_ok=True)
        
        # Gesture recognition directories
        self.gesture_base_dir = Path.home() / 'dev' / 'r2d2' / 'data' / 'gesture_recognition'
        self.gesture_base_dir.mkdir(parents=True, exist_ok=True)
        self.gesture_models_dir = self.gesture_base_dir / 'models'
        self.gesture_models_dir.mkdir(parents=True, exist_ok=True)
        
        # Person registry for central person management
        self.person_registry = PersonRegistry()
    
    def clear_screen(self):
        """Clear terminal screen."""
        os.system('clear' if os.name != 'nt' else 'cls')
    
    def show_header(self, title):
        """Display formatted header."""
        self.clear_screen()
        print('='*70)
        print(f'{title.center(70)}')
        print('='*70)
        print()
    
    def list_trained_people(self):
        """List all trained people."""
        trained = []
        for model_file in sorted(self.models_dir.glob('*_lbph.xml')):
            person_name = model_file.stem.replace('_lbph', '')
            trained.append(person_name)
        return trained
    
    def list_training_datasets(self):
        """List all training datasets (captured images)."""
        datasets = {}
        for person_dir in self.base_dir.iterdir():
            if person_dir.is_dir() and person_dir.name != 'models':
                image_count = len(list(person_dir.glob('*.jpg')))
                if image_count > 0:
                    datasets[person_dir.name] = image_count
        return datasets
    
    def list_gesture_trained_people(self):
        """List people with trained gesture models."""
        trained = []
        for model_file in sorted(self.gesture_models_dir.glob('*_gesture_classifier.pkl')):
            person_name = model_file.stem.replace('_gesture_classifier', '')
            trained.append(person_name)
        return trained
    
    def list_gesture_datasets(self):
        """List all gesture training datasets."""
        datasets = {}
        for person_dir in self.gesture_base_dir.iterdir():
            if person_dir.is_dir() and person_dir.name != 'models':
                gesture_counts = {}
                for gesture_dir in person_dir.iterdir():
                    if gesture_dir.is_dir():
                        image_count = len(list(gesture_dir.glob('*.jpg')))
                        if image_count > 0:
                            gesture_counts[gesture_dir.name] = image_count
                if gesture_counts:
                    datasets[person_dir.name] = gesture_counts
        return datasets
    
    def run_module(self, module_name, person_name):
        """
        Run a training module.
        
        Args:
            module_name: '_capture_module', '_train_module', or '_test_module'
            person_name: Name of person being trained
        
        Returns:
            True if successful, False otherwise
        """
        module_path = self.script_dir / f'{module_name}.py'
        
        if not module_path.exists():
            print(f'❌ Error: {module_name}.py not found at {module_path}')
            return False
        
        # Set up environment
        env = os.environ.copy()
        
        # Try to use the depthai virtual environment
        depthai_env = Path.home() / 'depthai_env'
        if depthai_env.exists():
            env['PATH'] = str(depthai_env / 'bin') + ':' + env['PATH']
            env['VIRTUAL_ENV'] = str(depthai_env)
        
        # Set ARM optimization if needed
        env['OPENBLAS_CORETYPE'] = 'ARMV8'
        
        try:
            result = subprocess.run(
                ['python3', str(module_path), person_name, str(self.base_dir)],
                env=env,
                check=False
            )
            return result.returncode == 0
        except Exception as e:
            print(f'❌ Error running module: {e}')
            return False
    
    def run_interactive_training(self, person_name):
        """
        Run the improved capture system with quality filtering (NEW METHOD).
        
        Args:
            person_name: Name of person being trained
        """
        script_path = self.script_dir / '1_capture_training_data.py'
        
        if not script_path.exists():
            print(f'❌ Error: 1_capture_training_data.py not found at {script_path}')
            return False
        
        # Set up environment
        env = os.environ.copy()
        
        # Try to use the depthai virtual environment
        depthai_env = Path.home() / 'depthai_env'
        if depthai_env.exists():
            env['PATH'] = str(depthai_env / 'bin') + ':' + env['PATH']
            env['VIRTUAL_ENV'] = str(depthai_env)
        
        # Set ARM optimization if needed
        env['OPENBLAS_CORETYPE'] = 'ARMV8'
        
        try:
            # Run improved capture script with person_name and base_dir as arguments
            result = subprocess.run(
                ['python3', str(script_path), person_name, str(self.base_dir)],
                env=env,
                check=False
            )
            return result.returncode == 0
        
        except Exception as e:
            print(f'❌ Error running improved capture: {e}')
            return False
    
    def main_menu(self):
        """Main menu loop."""
        while True:
            self.show_header('R2D2 Training Manager - Face & Gesture Recognition')
            
            trained = self.list_trained_people()
            datasets = self.list_training_datasets()
            gesture_trained = self.list_gesture_trained_people()
            gesture_datasets = self.list_gesture_datasets()
            
            print('📦 TRAINED FACE MODELS:')
            if trained:
                for person in trained:
                    model_file = self.models_dir / f'{person}_lbph.xml'
                    size = model_file.stat().st_size / 1024
                    print(f'   ✓ {person.capitalize()} ({size:.1f} KB)')
            else:
                print('   (none yet)')
            
            print()
            print('📷 FACE TRAINING DATASETS:')
            if datasets:
                for person, count in sorted(datasets.items()):
                    print(f'   • {person.capitalize()} ({count} images)')
            else:
                print('   (none yet)')
            
            print()
            print('🤲 TRAINED GESTURE MODELS:')
            if gesture_trained:
                for person in gesture_trained:
                    model_file = self.gesture_models_dir / f'{person}_gesture_classifier.pkl'
                    size = model_file.stat().st_size / 1024
                    print(f'   ✓ {person.capitalize()} ({size:.1f} KB)')
            else:
                print('   (none yet)')
            
            print()
            print('📸 GESTURE TRAINING DATASETS:')
            if gesture_datasets:
                for person, gesture_counts in sorted(gesture_datasets.items()):
                    print(f'   • {person.capitalize()}')
                    for gesture, count in sorted(gesture_counts.items()):
                        print(f'      - {gesture}: {count} images')
            else:
                print('   (none yet)')
            
            print()
            print('='*70)
            print('FACE RECOGNITION:')
            print('  [1] Train new person (capture + train + test)')
            print('  [2] Add additional training pictures')
            print('  [3] Train model from existing images')
            print('  [4] Test trained model (accuracy at distances)')
            print('  [5] Real-time recognition test (instant feedback)')
            print('  [6] List all people and models')
            print('  [7] Delete person (images + model)')
            print()
            print('GESTURE RECOGNITION:')
            print('  [8] Train gestures for person (capture + train + test)')
            print('  [9] Add additional gesture pictures')
            print('  [10] Train gesture model from existing images')
            print('  [11] Test gesture classifier (real-time)')
            print('  [12] List all gesture datasets and models')
            print('  [13] Delete person gestures (images + model)')
            print()
            print('PERSON MANAGEMENT:')
            print('  [14] Manage persons (launch person_manager.py)')
            print()
            print('  [0] Exit')
            print('='*70)
            print()
            
            choice = input('Enter choice (0-14): ').strip()
            
            if choice == '1':
                self.train_person()
            elif choice == '2':
                self.add_training_pictures()
            elif choice == '3':
                self.train_existing_person()
            elif choice == '4':
                self.test_trained_model()
            elif choice == '5':
                self.realtime_recognition_test()
            elif choice == '6':
                self.show_all_people()
            elif choice == '7':
                self.delete_person()
            elif choice == '8':
                self.train_person_gestures()
            elif choice == '9':
                self.add_gesture_pictures()
            elif choice == '10':
                self.train_existing_gestures()
            elif choice == '11':
                self.test_gesture_classifier()
            elif choice == '12':
                self.show_all_gesture_info()
            elif choice == '13':
                self.delete_person_gestures()
            elif choice == '14':
                self.launch_person_manager()
            elif choice == '0':
                self.show_header('Exit')
                print('Thank you for using R2D2 Training Manager!\n')
                return
            else:
                print('❌ Invalid choice. Try again.')
                input('Press ENTER...')
    
    def get_person_name(self, prompt='Enter person name: '):
        """Get valid person name from user."""
        while True:
            name = input(prompt).strip().lower().replace(' ', '_')
            if not name:
                print('❌ Name cannot be empty.')
                continue
            if not all(c.isalnum() or c == '_' for c in name):
                print('❌ Use only letters, numbers, and underscores.')
                continue
            return name
    
    def train_person(self):
        """Complete training workflow for new person."""
        self.show_header('Train New Person')
        
        print('Complete workflow: interactive capture → train → test')
        print()
        
        person_name = self.get_person_name()
        
        # Check if person already exists
        trained = self.list_trained_people()
        datasets = self.list_training_datasets()
        
        if person_name in trained or person_name in datasets:
            print()
            print(f'⚠️  Person "{person_name.capitalize()}" already exists!')
            if person_name in trained:
                model_file = self.models_dir / f'{person_name}_lbph.xml'
                size = model_file.stat().st_size / 1024
                print(f'   - Has trained model: {size:.1f} KB')
            if person_name in datasets:
                count = len(list((self.base_dir / person_name).glob('*.jpg')))
                print(f'   - Has {count} training images')
            print()
            confirm = input('Overwrite existing data? (yes/no): ').strip().lower()
            if confirm != 'yes':
                print('❌ Cancelled.')
                input('Press ENTER to return...')
                return
        
        # Create directory
        person_dir = self.base_dir / person_name
        person_dir.mkdir(parents=True, exist_ok=True)
        
        print(f'\n✓ Set up for: {person_name}')
        print()
        
        # Step 1: Interactive Capture with 4 tasks
        print('STEP 1: Interactive Capture Training Images')
        print('-' * 70)
        proceed = input('Ready to start? (y/n): ').strip().lower()
        if proceed == 'y':
            self.run_interactive_training(person_name)
        
        # Check if images exist
        images = list(person_dir.glob('*.jpg'))
        if not images:
            print('\n❌ No training images captured. Aborting.')
            input('Press ENTER to return...')
            return
        
        # Step 2: Train
        print('\nSTEP 2: Train Model')
        print('-' * 70)
        print(f'Found {len(images)} training images.')
        proceed = input('Ready to train? (y/n): ').strip().lower()
        if proceed == 'y':
            self.run_module('_train_module', person_name)
        
        # Check if model exists
        model_file = self.models_dir / f'{person_name}_lbph.xml'
        if not model_file.exists():
            print('\n❌ Model training failed. Aborting.')
            input('Press ENTER to return...')
            return
        
        # Step 3: Test
        print('\nSTEP 3: Test Model')
        print('-' * 70)
        proceed = input('Ready to test? (y/n): ').strip().lower()
        if proceed == 'y':
            self.run_module('_test_module', person_name)
        
        # Register person in person registry
        try:
            person = self.person_registry.get_person(person_name)
            if not person:
                person_id = self.person_registry.register_person(person_name)
            else:
                person_id = person['id']
            self.person_registry.update_face_model(person_id, str(model_file))
            print(f'\n✓ Registered in person registry')
        except Exception as e:
            print(f'\n⚠️  Warning: Could not register in person registry: {e}')
        
        print('\n' + '='*70)
        print('✅ TRAINING COMPLETE')
        print('='*70)
        print(f'\nPerson: {person_name}')
        print(f'Images: {len(images)}')
        print(f'Model: {person_name}_lbph.xml')
        print(f'\nYour model is ready for deployment!')
        print('='*70)
        print()
        
        input('Press ENTER to return to menu...')
    
    def add_training_pictures(self):
        """Add additional training pictures for existing person."""
        self.show_header('Add Additional Training Pictures')
        
        trained = self.list_trained_people()
        datasets = self.list_training_datasets()
        people = sorted(set(list(trained) + list(datasets.keys())))
        
        if not people:
            print('No people available. Create a new person first.')
            input('Press ENTER...')
            return
        
        print('Select person to add more pictures to:')
        for i, person in enumerate(people, 1):
            count = datasets.get(person, 0)
            print(f'  [{i}] {person.capitalize()} ({count} images)')
        print(f'  [0] Back')
        print()
        
        choice = input('Enter choice: ').strip()
        if choice == '0':
            return
        
        try:
            person_name = people[int(choice) - 1]
            
            # Show current data and ask for confirmation
            person_dir = self.base_dir / person_name
            existing_count = len(list(person_dir.glob('*.jpg')))
            
            if existing_count > 0:
                print()
                print(f'Current dataset for "{person_name.capitalize()}":')
                print(f'  • {existing_count} images')
                print()
                print('New images will be ADDED to this existing dataset.')
                confirm = input('Continue adding more pictures? (yes/no): ').strip().lower()
                if confirm != 'yes':
                    print('❌ Cancelled.')
                    return
                print()
            
            self.run_interactive_training(person_name)
        except (ValueError, IndexError):
            print('❌ Invalid choice.')
            input('Press ENTER...')
    
    def train_existing_person(self):
        """Train model for person with images."""
        self.show_header('Train Model')
        
        datasets = self.list_training_datasets()
        
        if not datasets:
            print('No training datasets found.')
            print('Capture images first.')
            input('Press ENTER...')
            return
        
        people = sorted(datasets.keys())
        print('Select person to train:')
        for i, person in enumerate(people, 1):
            count = datasets[person]
            print(f'  [{i}] {person.capitalize()} ({count} images)')
        print(f'  [0] Back')
        print()
        
        choice = input('Enter choice: ').strip()
        if choice == '0':
            return
        
        try:
            person_name = people[int(choice) - 1]
            self.run_module('_train_module', person_name)
        except (ValueError, IndexError):
            print('❌ Invalid choice.')
            input('Press ENTER...')
    
    def test_trained_model(self):
        """Test trained model."""
        self.show_header('Test Model')
        
        trained = self.list_trained_people()
        
        if not trained:
            print('No trained models found.')
            print('Train a model first.')
            input('Press ENTER...')
            return
        
        print('Select person to test:')
        for i, person in enumerate(trained, 1):
            print(f'  [{i}] {person.capitalize()}')
        print(f'  [0] Back')
        print()
        
        choice = input('Enter choice: ').strip()
        if choice == '0':
            return
        
        try:
            person_name = trained[int(choice) - 1]
            self.run_module('_test_module', person_name)
        except (ValueError, IndexError):
            print('❌ Invalid choice.')
            input('Press ENTER...')
    
    def realtime_recognition_test(self):
        """Real-time face recognition test with instant feedback."""
        self.show_header('Real-Time Recognition Test')
        
        trained = self.list_trained_people()
        
        if not trained:
            print('No trained models found.')
            print('Train a model first.')
            input('Press ENTER...')
            return
        
        print('Select person to test:')
        for i, person in enumerate(trained, 1):
            print(f'  [{i}] {person.capitalize()}')
        print(f'  [0] Back')
        print()
        
        choice = input('Enter choice: ').strip()
        if choice == '0':
            return
        
        try:
            person_name = trained[int(choice) - 1]
            
            # Run real-time recognition test
            script_path = self.script_dir / 'realtime_recognition_test_headless.py'
            
            if not script_path.exists():
                print(f'❌ Error: {script_path.name} not found')
                input('Press ENTER...')
                return
            
            import subprocess
            env = os.environ.copy()
            
            # Use depthai environment
            depthai_env = Path.home() / 'depthai_env'
            if depthai_env.exists():
                env['PATH'] = str(depthai_env / 'bin') + ':' + env['PATH']
                env['VIRTUAL_ENV'] = str(depthai_env)
            
            env['OPENBLAS_CORETYPE'] = 'ARMV8'
            
            # Run with 30 second duration and threshold 70 (best we found)
            subprocess.run(
                ['python3', str(script_path), person_name, str(self.base_dir), '30', '70'],
                env=env,
                check=False
            )
            
            input('Press ENTER to return to menu...')
        except (ValueError, IndexError):
            print('❌ Invalid choice.')
            input('Press ENTER...')
    
    def show_all_people(self):
        """Display all people and their status."""
        self.show_header('All People and Models')
        
        trained = self.list_trained_people()
        datasets = self.list_training_datasets()
        all_people = sorted(set(list(trained) + list(datasets.keys())))
        
        if not all_people:
            print('No people or models yet.\n')
            input('Press ENTER...')
            return
        
        print(f'Total: {len(all_people)} people\n')
        
        for person in all_people:
            has_model = person in trained
            image_count = datasets.get(person, 0)
            
            status = ''
            if has_model:
                model_file = self.models_dir / f'{person}_lbph.xml'
                size = model_file.stat().st_size / 1024
                status = f' ✓ Model ({size:.1f} KB)'
            
            print(f'{person.capitalize()}')
            print(f'  Images: {image_count}{status}')
        
        print()
        input('Press ENTER...')
    
    def delete_person(self):
        """Delete person's data and model."""
        self.show_header('Delete Person')
        
        trained = self.list_trained_people()
        datasets = self.list_training_datasets()
        all_people = sorted(set(list(trained) + list(datasets.keys())))
        
        if not all_people:
            print('No people to delete.')
            input('Press ENTER...')
            return
        
        print('⚠️  WARNING: This will delete ALL data and models')
        print()
        print('Select person:')
        for i, person in enumerate(all_people, 1):
            print(f'  [{i}] {person.capitalize()}')
        print(f'  [0] Cancel')
        print()
        
        choice = input('Enter choice: ').strip()
        if choice == '0':
            return
        
        try:
            person_name = all_people[int(choice) - 1]
        except (ValueError, IndexError):
            print('❌ Invalid choice.')
            input('Press ENTER...')
            return
        
        print(f'\n⚠️  Really delete "{person_name}"? (type YES to confirm)')
        confirm = input('> ').strip()
        
        if confirm.upper() != 'YES':
            print('Cancelled.')
            input('Press ENTER...')
            return
        
        # Delete images
        person_dir = self.base_dir / person_name
        if person_dir.exists():
            import shutil
            shutil.rmtree(person_dir)
            print(f'✓ Deleted images')
        
        # Delete model
        model_file = self.models_dir / f'{person_name}_lbph.xml'
        if model_file.exists():
            model_file.unlink()
            print(f'✓ Deleted model')
        
        # Delete from person registry
        try:
            person = self.person_registry.get_person(person_name)
            if person:
                self.person_registry.delete_person(person['id'])
                print(f'✓ Removed from person registry')
        except Exception as e:
            print(f'⚠️  Warning: Could not remove from person registry: {e}')
        
        print(f'\n✅ Deleted: {person_name}')
        input('Press ENTER...')
    
    def launch_person_manager(self):
        """Launch the person manager CLI."""
        self.show_header('Person Manager')
        
        print('Launching person_manager.py...')
        print()
        
        person_manager_path = self.script_dir / 'person_manager.py'
        
        if not person_manager_path.exists():
            print(f'❌ Error: person_manager.py not found at {person_manager_path}')
            input('Press ENTER to return...')
            return
        
        # Run person manager
        try:
            subprocess.run(['python3', str(person_manager_path)])
        except KeyboardInterrupt:
            print('\n\n👋 Returned from person manager.')
        except Exception as e:
            print(f'\n❌ Error: {e}')
            input('Press ENTER to return...')
    
    # ========== GESTURE RECOGNITION METHODS ==========
    
    def run_gesture_module(self, module_name, person_name):
        """
        Run a gesture training module.
        
        Args:
            module_name: '_gesture_capture_module', '_gesture_train_module', or '_gesture_test_module'
            person_name: Name of person being trained
        
        Returns:
            True if successful, False otherwise
        """
        module_path = self.script_dir / f'{module_name}.py'
        
        if not module_path.exists():
            print(f'❌ Error: {module_name}.py not found at {module_path}')
            return False
        
        # Set up environment
        env = os.environ.copy()
        
        # Try to use the depthai virtual environment
        depthai_env = Path.home() / 'depthai_env'
        if depthai_env.exists():
            env['PATH'] = str(depthai_env / 'bin') + ':' + env['PATH']
            env['VIRTUAL_ENV'] = str(depthai_env)
        
        # Set ARM optimization if needed
        env['OPENBLAS_CORETYPE'] = 'ARMV8'
        
        try:
            result = subprocess.run(
                ['python3', str(module_path), person_name, str(self.gesture_base_dir)],
                env=env,
                check=False
            )
            return result.returncode == 0
        except Exception as e:
            print(f'❌ Error running module: {e}')
            return False
    
    def train_person_gestures(self):
        """Complete gesture training workflow for target person."""
        self.show_header('Train Gestures for Target Person')
        
        print('Complete workflow: capture gestures → train → test')
        print()
        
        # Get target person name
        person_name = self.get_person_name('Enter target person name: ')
        
        # Check if person exists in face recognition (recommendation only)
        trained_faces = self.list_trained_people()
        if person_name not in trained_faces:
            print(f'\n⚠️  Note: "{person_name.capitalize()}" not found in face recognition.')
            print('   Gesture training can proceed independently.')
            print('   However, it\'s recommended to train face recognition first.')
            print()
            proceed = input('Continue anyway? (y/n): ').strip().lower()
            if proceed != 'y':
                print('❌ Cancelled.')
                input('Press ENTER to return...')
                return
        
        # Check if gesture data already exists
        gesture_datasets = self.list_gesture_datasets()
        gesture_trained = self.list_gesture_trained_people()
        
        if person_name in gesture_datasets or person_name in gesture_trained:
            print()
            print(f'⚠️  Gesture data already exists for "{person_name.capitalize()}"!')
            if person_name in gesture_trained:
                model_file = self.gesture_models_dir / f'{person_name}_gesture_classifier.pkl'
                size = model_file.stat().st_size / 1024
                print(f'   - Has trained model: {size:.1f} KB')
            if person_name in gesture_datasets:
                counts = gesture_datasets[person_name]
                for gesture, count in counts.items():
                    print(f'   - {gesture}: {count} images')
            print()
            confirm = input('Overwrite existing data? (yes/no): ').strip().lower()
            if confirm != 'yes':
                print('❌ Cancelled.')
                input('Press ENTER to return...')
                return
        
        print(f'\n✓ Set up for: {person_name}')
        print()
        
        # Step 1: Capture gestures
        print('STEP 1: Capture Gesture Images')
        print('-' * 70)
        print('You will capture three gestures:')
        print('  • Index finger up (pointing upward) - starts Fast Mode')
        print('  • Fist (all fingers closed) - stops conversation')
        print('  • Open hand (palm facing camera) - starts Intelligent Mode')
        print()
        proceed = input('Ready to capture? (y/n): ').strip().lower()
        if proceed == 'y':
            self.run_gesture_module('_gesture_capture_module', person_name)
        
        # Check if images captured
        gesture_data = self.list_gesture_datasets()
        if person_name not in gesture_data:
            print('\n❌ No gesture images captured. Aborting.')
            input('Press ENTER to return...')
            return
        
        # Step 2: Train classifier
        print('\nSTEP 2: Train Gesture Classifier')
        print('-' * 70)
        # Show image counts
        counts = gesture_data[person_name]
        for gesture, count in sorted(counts.items()):
            print(f'  {gesture}: {count} images')
        print()
        proceed = input('Ready to train? (y/n): ').strip().lower()
        if proceed == 'y':
            self.run_gesture_module('_gesture_train_module', person_name)
        
        # Check if model exists
        model_file = self.gesture_models_dir / f'{person_name}_gesture_classifier.pkl'
        if not model_file.exists():
            print('\n❌ Model training failed. Aborting.')
            input('Press ENTER to return...')
            return
        
        # Step 3: Test classifier
        print('\nSTEP 3: Test Gesture Classifier')
        print('-' * 70)
        proceed = input('Ready to test? (y/n): ').strip().lower()
        if proceed == 'y':
            self.run_gesture_module('_gesture_test_module', person_name)
        
        # Register/update person in person registry
        try:
            person = self.person_registry.get_person(person_name)
            if not person:
                person_id = self.person_registry.register_person(person_name)
            else:
                person_id = person['id']
            self.person_registry.update_gesture_model(person_id, str(model_file))
            print(f'\n✓ Registered in person registry')
        except Exception as e:
            print(f'\n⚠️  Warning: Could not register in person registry: {e}')
        
        print('\n' + '='*70)
        print('✅ GESTURE TRAINING COMPLETE')
        print('='*70)
        print(f'\nPerson: {person_name}')
        for gesture, count in sorted(counts.items()):
            print(f'  {gesture}: {count} images')
        print(f'Model: {person_name}_gesture_classifier.pkl')
        print(f'\nYour gesture model is ready for deployment!')
        print('='*70)
        print()
        
        input('Press ENTER to return to menu...')
    
    def add_gesture_pictures(self):
        """Add additional gesture pictures for existing person."""
        self.show_header('Add Additional Gesture Pictures')
        
        gesture_trained = self.list_gesture_trained_people()
        gesture_datasets = self.list_gesture_datasets()
        people = sorted(set(list(gesture_trained) + list(gesture_datasets.keys())))
        
        if not people:
            print('No people with gesture data available.')
            print('Train gestures for a person first.')
            input('Press ENTER...')
            return
        
        print('Select person to add more gesture pictures to:')
        for i, person in enumerate(people, 1):
            counts = gesture_datasets.get(person, {})
            if counts:
                gesture_summary = ', '.join([f'{g}: {c}' for g, c in sorted(counts.items())])
                print(f'  [{i}] {person.capitalize()} ({gesture_summary})')
            else:
                print(f'  [{i}] {person.capitalize()} (no images yet)')
        print(f'  [0] Back')
        print()
        
        choice = input('Enter choice: ').strip()
        if choice == '0':
            return
        
        try:
            person_name = people[int(choice) - 1]
            
            # Show current data
            person_dir = self.gesture_base_dir / person_name
            existing_counts = gesture_datasets.get(person_name, {})
            
            if existing_counts:
                print()
                print(f'Current gesture dataset for "{person_name.capitalize()}":')
                for gesture, count in sorted(existing_counts.items()):
                    print(f'  • {gesture}: {count} images')
                print()
                print('New images will be ADDED to this existing dataset.')
                confirm = input('Continue adding more pictures? (yes/no): ').strip().lower()
                if confirm != 'yes':
                    print('❌ Cancelled.')
                    return
                print()
            
            # Run capture module (it will capture all gestures again)
            self.run_gesture_module('_gesture_capture_module', person_name)
        except (ValueError, IndexError):
            print('❌ Invalid choice.')
            input('Press ENTER...')
    
    def train_existing_gestures(self):
        """Train gesture model from existing images."""
        self.show_header('Train Gesture Model')
        
        gesture_datasets = self.list_gesture_datasets()
        
        if not gesture_datasets:
            print('No gesture training datasets found.')
            print('Capture gesture images first.')
            input('Press ENTER...')
            return
        
        people = sorted(gesture_datasets.keys())
        print('Select person to train gestures for:')
        for i, person in enumerate(people, 1):
            counts = gesture_datasets[person]
            gesture_summary = ', '.join([f'{g}: {c}' for g, c in sorted(counts.items())])
            print(f'  [{i}] {person.capitalize()} ({gesture_summary})')
        print(f'  [0] Back')
        print()
        
        choice = input('Enter choice: ').strip()
        if choice == '0':
            return
        
        try:
            person_name = people[int(choice) - 1]
            self.run_gesture_module('_gesture_train_module', person_name)
        except (ValueError, IndexError):
            print('❌ Invalid choice.')
            input('Press ENTER...')
    
    def test_gesture_classifier(self):
        """Test gesture classifier in real-time."""
        self.show_header('Test Gesture Classifier')
        
        gesture_trained = self.list_gesture_trained_people()
        
        if not gesture_trained:
            print('No trained gesture models found.')
            print('Train a gesture model first.')
            input('Press ENTER...')
            return
        
        print('Select person to test:')
        for i, person in enumerate(gesture_trained, 1):
            print(f'  [{i}] {person.capitalize()}')
        print(f'  [0] Back')
        print()
        
        choice = input('Enter choice: ').strip()
        if choice == '0':
            return
        
        try:
            person_name = gesture_trained[int(choice) - 1]
            self.run_gesture_module('_gesture_test_module', person_name)
        except (ValueError, IndexError):
            print('❌ Invalid choice.')
            input('Press ENTER...')
    
    def show_all_gesture_info(self):
        """Display all gesture datasets and models."""
        self.show_header('All Gesture Datasets and Models')
        
        gesture_trained = self.list_gesture_trained_people()
        gesture_datasets = self.list_gesture_datasets()
        all_people = sorted(set(list(gesture_trained) + list(gesture_datasets.keys())))
        
        if not all_people:
            print('No gesture data or models yet.\n')
            input('Press ENTER...')
            return
        
        print(f'Total: {len(all_people)} people with gesture data\n')
        
        for person in all_people:
            has_model = person in gesture_trained
            gesture_counts = gesture_datasets.get(person, {})
            
            status = ''
            if has_model:
                model_file = self.gesture_models_dir / f'{person}_gesture_classifier.pkl'
                size = model_file.stat().st_size / 1024
                status = f' ✓ Model ({size:.1f} KB)'
            
            print(f'{person.capitalize()}{status}')
            if gesture_counts:
                for gesture, count in sorted(gesture_counts.items()):
                    print(f'  {gesture}: {count} images')
            else:
                print(f'  No images')
            print()
        
        input('Press ENTER...')
    
    def delete_person_gestures(self):
        """Delete person's gesture data and model."""
        self.show_header('Delete Person Gestures')
        
        gesture_trained = self.list_gesture_trained_people()
        gesture_datasets = self.list_gesture_datasets()
        all_people = sorted(set(list(gesture_trained) + list(gesture_datasets.keys())))
        
        if not all_people:
            print('No gesture data to delete.')
            input('Press ENTER...')
            return
        
        print('⚠️  WARNING: This will delete ALL gesture data and models')
        print()
        print('Select person:')
        for i, person in enumerate(all_people, 1):
            print(f'  [{i}] {person.capitalize()}')
        print(f'  [0] Cancel')
        print()
        
        choice = input('Enter choice: ').strip()
        if choice == '0':
            return
        
        try:
            person_name = all_people[int(choice) - 1]
        except (ValueError, IndexError):
            print('❌ Invalid choice.')
            input('Press ENTER...')
            return
        
        print(f'\n⚠️  Really delete gesture data for "{person_name}"? (type YES to confirm)')
        confirm = input('> ').strip()
        
        if confirm.upper() != 'YES':
            print('Cancelled.')
            input('Press ENTER...')
            return
        
        # Delete images
        person_dir = self.gesture_base_dir / person_name
        if person_dir.exists():
            import shutil
            shutil.rmtree(person_dir)
            print(f'✓ Deleted gesture images')
        
        # Delete model
        model_file = self.gesture_models_dir / f'{person_name}_gesture_classifier.pkl'
        if model_file.exists():
            model_file.unlink()
            print(f'✓ Deleted gesture model')
        
        print(f'\n✅ Deleted gesture data for: {person_name}')
        input('Press ENTER...')
    
    def run(self):
        """Start the training manager."""
        try:
            self.main_menu()
        except KeyboardInterrupt:
            print('\n\n👋 Interrupted by user.')
            sys.exit(0)
        except Exception as e:
            print(f'\n❌ Error: {e}')
            sys.exit(1)


if __name__ == '__main__':
    manager = TrainingManager()
    manager.run()
