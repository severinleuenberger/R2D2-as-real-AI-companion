#!/usr/bin/env python3
"""
Face Recognizer Training Script for R2D2

Purpose:
  Trains an OpenCV LBPH (Local Binary Pattern Histograms) face recognizer
  on captured training images. Saves trained model for later recognition.

Usage:
  python3 2_train_recognizer.py

Input:
  Expects face images in ~/dev/r2d2/data/face_recognition/severin/
  (captured by 1_capture_training_data.py)

Output:
  Saves trained LBPH model to ~/dev/r2d2/data/face_recognition/models/severin_lbph.xml

Why LBPH?
  - CPU-efficient (no GPU needed)
  - Fast training and recognition (milliseconds on ARM)
  - Good accuracy for frontal face recognition
  - Model file is small (~50 KB)
  - Works well on downscaled/low-res face crops

Author: R2D2 Perception Pipeline
Date: December 6, 2025
"""

import cv2
import os
import numpy as np
from pathlib import Path


class FaceRecognizerTrainer:
    """Trains LBPH face recognizer on captured face images."""
    
    def __init__(self):
        """Initialize paths and LBPH recognizer."""
        self.training_dir = os.path.expanduser('~/dev/r2d2/data/face_recognition/severin')
        self.model_dir = os.path.expanduser('~/dev/r2d2/data/face_recognition/models')
        self.model_path = os.path.join(self.model_dir, 'severin_lbph.xml')
        
        # Create model directory if it doesn't exist
        os.makedirs(self.model_dir, exist_ok=True)
        
        # Initialize LBPH recognizer with tuned parameters for ARM
        # radius=1: Local Binary Pattern radius (smaller = faster)
        # neighbors=8: Number of pixels to consider (smaller = faster)
        # grid_x/grid_y=8: Grid for histogram computation
        self.recognizer = cv2.face.LBPHFaceRecognizer_create(
            radius=1,
            neighbors=8,
            grid_x=8,
            grid_y=8
        )
        
        print('\n' + '='*60)
        print('LBPH FACE RECOGNIZER TRAINER')
        print('='*60)
        print(f'Training data directory: {self.training_dir}')
        print(f'Model output: {self.model_path}')
        print('='*60 + '\n')
    
    def load_training_images(self):
        """
        Load all training images from the training directory.
        
        Returns:
            tuple: (faces, labels, image_count)
              faces: List of 100x100 grayscale images (numpy arrays)
              labels: List of integer labels (0 for Severin)
              image_count: Number of images loaded
        """
        faces = []
        labels = []
        image_count = 0
        
        print('[Loading] Scanning training directory for images...')
        
        # Get all JPG files in training directory
        image_files = sorted(Path(self.training_dir).glob('*.jpg'))
        
        if not image_files:
            raise RuntimeError(f'No JPG files found in {self.training_dir}')
        
        print(f'Found {len(image_files)} image files.')
        
        for image_path in image_files:
            try:
                # Load image in grayscale (LBPH expects grayscale)
                face_image = cv2.imread(str(image_path), cv2.IMREAD_GRAYSCALE)
                
                if face_image is None:
                    print(f'  ⚠ Warning: Failed to load {image_path.name}')
                    continue
                
                # Verify image is 100x100 (as captured)
                if face_image.shape != (100, 100):
                    print(f'  ⚠ Warning: {image_path.name} is {face_image.shape}, expected (100,100)')
                    continue
                
                # Add to training set
                faces.append(face_image)
                labels.append(0)  # 0 = Severin (only one person for now)
                image_count += 1
                
            except Exception as e:
                print(f'  ❌ Error loading {image_path.name}: {e}')
                continue
        
        if image_count == 0:
            raise RuntimeError('No valid images were loaded for training.')
        
        print(f'✓ Loaded {image_count} training images')
        return np.array(faces), np.array(labels), image_count
    
    def train(self, faces, labels):
        """
        Train the LBPH recognizer.
        
        Args:
            faces: numpy array of training images
            labels: numpy array of labels
        """
        print(f'\n[Training] Training LBPH recognizer on {len(faces)} images...')
        
        # Train the recognizer
        # Note: This is CPU-only, no GPU acceleration
        self.recognizer.train(faces, labels)
        
        print('✓ Training complete')
    
    def save_model(self):
        """Save trained model to file."""
        print(f'\n[Saving] Writing model to {self.model_path}...')
        
        self.recognizer.write(self.model_path)
        
        # Verify file was created
        if os.path.exists(self.model_path):
            file_size = os.path.getsize(self.model_path)
            print(f'✓ Model saved successfully ({file_size} bytes)')
        else:
            raise RuntimeError(f'Failed to save model to {self.model_path}')
    
    def run(self):
        """Run the complete training pipeline."""
        try:
            # Load training images
            faces, labels, image_count = self.load_training_images()
            
            # Train recognizer
            self.train(faces, labels)
            
            # Save model
            self.save_model()
            
            # Summary
            print('\n' + '='*60)
            print('TRAINING COMPLETE')
            print('='*60)
            print(f'Images trained on: {image_count}')
            print(f'Model location: {self.model_path}')
            print(f'Model size: {os.path.getsize(self.model_path)} bytes')
            print('\nNext step: Run 3_test_recognizer_demo.py to test recognition.')
            print('='*60 + '\n')
            
            return True
            
        except Exception as e:
            print(f'\n❌ Error: {e}')
            return False


if __name__ == '__main__':
    trainer = FaceRecognizerTrainer()
    success = trainer.run()
    exit(0 if success else 1)
