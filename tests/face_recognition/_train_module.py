#!/usr/bin/env python3
"""
Modular Face Training Script

Purpose:
  Trains LBPH face recognizer on captured images.
  Called by train_manager.py with person name and data directory.
  Saves trained model for later use.

Usage (via manager):
  python3 train_manager.py  (select "Train recognizer")

Usage (direct):
  python3 _train_module.py <person_name> <data_dir>

Author: R2D2 Perception Pipeline
Date: December 6, 2025
"""

import cv2
import os
import sys
from pathlib import Path


class TrainingModule:
    """Trains LBPH face recognizer on captured images."""
    
    def __init__(self, person_name, data_dir):
        """
        Initialize training module.
        
        Args:
            person_name: Name of person being trained
            data_dir: Base data directory path
        """
        self.person_name = person_name
        self.data_dir = Path(data_dir)
        self.person_dir = self.data_dir / person_name
        self.models_dir = self.data_dir / 'models'
        self.model_path = self.models_dir / f'{person_name}_lbph.xml'
        
        self.models_dir.mkdir(parents=True, exist_ok=True)
        
        # Initialize LBPH recognizer
        try:
            self.recognizer = cv2.face.LBPHFaceRecognizer_create()
        except AttributeError:
            raise RuntimeError(
                'OpenCV contrib module not installed.\n'
                'Install: pip install opencv-contrib-python'
            )
    
    def load_training_images(self):
        """
        Load all training images from person directory.
        
        Returns:
            (images, labels) - numpy arrays for training
        """
        print(f'\n[Loading] Training images from: {self.person_dir}')
        
        images = []
        labels = []
        
        if not self.person_dir.exists():
            raise FileNotFoundError(f'Person directory not found: {self.person_dir}')
        
        image_files = sorted(list(self.person_dir.glob('*.jpg')))
        
        if not image_files:
            raise ValueError(f'No training images found in: {self.person_dir}')
        
        print(f'  Found {len(image_files)} image files')
        print(f'  Loading images...', end='', flush=True)
        
        for idx, img_path in enumerate(image_files):
            try:
                img = cv2.imread(str(img_path), cv2.IMREAD_GRAYSCALE)
                
                if img is None:
                    print(f'\n  ⚠️  Skipped (read failed): {img_path.name}')
                    continue
                
                if img.shape != (100, 100):
                    print(f'\n  ⚠️  Skipped (wrong size): {img_path.name}')
                    continue
                
                images.append(img)
                labels.append(0)  # Person ID = 0 (can extend for multi-person)
                
                if (idx + 1) % 10 == 0:
                    print(f'.', end='', flush=True)
            
            except Exception as e:
                print(f'\n  ⚠️  Error loading {img_path.name}: {e}')
                continue
        
        print(' Done!')
        
        if len(images) == 0:
            raise ValueError('No valid training images loaded')
        
        print(f'  ✓ Successfully loaded: {len(images)} images')
        
        return images, labels
    
    def train(self):
        """Train LBPH recognizer on loaded images."""
        print(f'\n[Training] LBPH Face Recognizer')
        print(f'  Person: {self.person_name}')
        
        images, labels = self.load_training_images()
        
        print(f'\n[Processing] Training LBPH model...')
        print(f'  Images: {len(images)}')
        print(f'  Labels: {len(labels)}')
        
        try:
            import numpy as np
            self.recognizer.train(
                [np.asarray(img) for img in images],
                np.asarray(labels)
            )
            print(f'  ✓ Training complete')
        except Exception as e:
            raise RuntimeError(f'LBPH training failed: {e}')
    
    def save_model(self):
        """Save trained model to file."""
        print(f'\n[Saving] Model to: {self.model_path}')
        
        try:
            self.recognizer.write(str(self.model_path))
            file_size = self.model_path.stat().st_size
            print(f'  ✓ Model saved successfully')
            print(f'  ✓ File size: {file_size / 1024:.1f} KB')
        except Exception as e:
            raise RuntimeError(f'Failed to save model: {e}')
    
    def run(self):
        """Run the complete training sequence."""
        print('\n' + '='*70)
        print(f'TRAINING FACE RECOGNIZER FOR: {self.person_name.upper()}')
        print('='*70)
        
        try:
            print('\nThis will:')
            print('  1. Load all captured images')
            print('  2. Train LBPH face recognizer')
            print('  3. Save trained model')
            print()
            input('Press ENTER to continue...')
            
            self.train()
            self.save_model()
            
            print('\n' + '='*70)
            print('TRAINING COMPLETE')
            print('='*70)
            print(f'\n✓ Model trained and saved: {self.person_name}_lbph.xml')
            print(f'\nNext step: Test recognizer with trained model.')
            print('='*70 + '\n')
            
            return True
        
        except Exception as e:
            print(f'\n❌ Training failed: {e}')
            return False


if __name__ == '__main__':
    if len(sys.argv) != 3:
        print('Usage: python3 _train_module.py <person_name> <data_dir>')
        sys.exit(1)
    
    try:
        person_name = sys.argv[1]
        data_dir = sys.argv[2]
        
        training = TrainingModule(person_name, data_dir)
        success = training.run()
        sys.exit(0 if success else 1)
    except Exception as e:
        print(f'\n❌ Error: {e}')
        sys.exit(1)
