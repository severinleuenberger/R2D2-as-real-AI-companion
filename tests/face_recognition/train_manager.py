#!/usr/bin/env python3
"""
Face Training Manager - Modular, Interactive Face Recognition System

Purpose:
  Central hub for training and testing face recognition models.
  Supports multiple people, flexible workflows, and clear instructions.
  Calls modular scripts: _capture_module.py, _train_module.py, _test_module.py

Features:
  • Train multiple people (not just Severin)
  • Interactive menu system with stage-by-stage guidance
  • Clear, bold instructions with ENTER prompts
  • Modular architecture with external scripts
  • Model management (list, delete, info)
  • Progress tracking

Usage:
  source ~/depthai_env/bin/activate
  export OPENBLAS_CORETYPE=ARMV8
  python3 train_manager.py

Author: R2D2 Perception Pipeline
Date: December 6, 2025
"""

import subprocess
import sys
import os
from pathlib import Path
from datetime import datetime


class TrainingManager:
    """Central hub for face recognition training and testing."""
    
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
            self.show_header('Face Recognition Training Manager')
            
            trained = self.list_trained_people()
            datasets = self.list_training_datasets()
            
            print('📦 TRAINED MODELS:')
            if trained:
                for person in trained:
                    model_file = self.models_dir / f'{person}_lbph.xml'
                    size = model_file.stat().st_size / 1024
                    print(f'   ✓ {person.capitalize()} ({size:.1f} KB)')
            else:
                print('   (none yet)')
            
            print()
            print('📷 TRAINING DATASETS:')
            if datasets:
                for person, count in sorted(datasets.items()):
                    print(f'   • {person.capitalize()} ({count} images)')
            else:
                print('   (none yet)')
            
            print()
            print('='*70)
            print('MENU OPTIONS:')
            print('  [1] Train new person (capture + train + test)')
            print('  [2] Add additional training pictures')
            print('  [3] Train model from existing images')
            print('  [4] Test trained model (accuracy at distances)')
            print('  [5] Real-time recognition test (instant feedback)')
            print('  [6] List all people and models')
            print('  [7] Delete person (images + model)')
            print('  [0] Exit')
            print('='*70)
            print()
            
            choice = input('Enter choice (0-7): ').strip()
            
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
            elif choice == '0':
                self.show_header('Exit')
                print('Thank you for using Face Recognition Manager!\n')
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
        
        print(f'\n✅ Deleted: {person_name}')
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
