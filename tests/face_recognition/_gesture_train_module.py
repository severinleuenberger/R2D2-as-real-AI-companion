#!/usr/bin/env python3
"""
Modular Gesture Training Script (Person-Specific)

Purpose:
  Trains gesture classifier on captured gesture images.
  Called by train_manager.py with person name and data directory.
  Uses MediaPipe Hands to extract landmarks and trains SVM classifier.

Usage (via manager):
  python3 train_manager.py  (select "Train gesture model")

Usage (direct):
  python3 _gesture_train_module.py <person_name> <gesture_recognition_data_dir>

Author: R2D2 Perception Pipeline
Date: December 17, 2025
"""

import cv2
import os
import sys
import numpy as np
from pathlib import Path
import pickle

try:
    import mediapipe as mp
except ImportError:
    print('\n❌ Error: MediaPipe not installed.')
    print('Install with: pip install mediapipe')
    sys.exit(1)

try:
    from sklearn.svm import SVC
    from sklearn.preprocessing import StandardScaler
except ImportError:
    print('\n❌ Error: scikit-learn not installed.')
    print('Install with: pip install scikit-learn')
    sys.exit(1)


class GestureTrainingModule:
    """Trains gesture classifier on captured images."""
    
    def __init__(self, person_name, data_dir):
        """
        Initialize gesture training module.
        
        Args:
            person_name: Name of person to train gestures for
            data_dir: Base gesture recognition data directory
        """
        self.person_name = person_name
        self.data_dir = Path(data_dir)
        self.person_dir = self.data_dir / person_name
        self.models_dir = self.data_dir / 'models'
        self.model_path = self.models_dir / f'{person_name}_gesture_classifier.pkl'
        
        self.models_dir.mkdir(parents=True, exist_ok=True)
        
        # Gesture classes
        self.gestures = ['index_finger_up', 'fist']
        self.gesture_to_label = {
            'index_finger_up': 0,
            'fist': 1
        }
        
        # Initialize MediaPipe Hands
        self.mp_hands = mp.solutions.hands
        self.hands = self.mp_hands.Hands(
            static_image_mode=True,
            max_num_hands=1,
            min_detection_confidence=0.5
        )
        
        # Initialize classifier and scaler
        self.classifier = SVC(kernel='rbf', probability=True, random_state=42)
        self.scaler = StandardScaler()
    
    def extract_hand_landmarks(self, image):
        """
        Extract hand landmarks from image using MediaPipe.
        
        Args:
            image: OpenCV image (BGR)
            
        Returns:
            numpy array of 63 features (21 landmarks × 3 coords) or None
        """
        # Convert to RGB for MediaPipe
        image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        
        # Process with MediaPipe
        results = self.hands.process(image_rgb)
        
        if not results.multi_hand_landmarks:
            return None
        
        # Extract first detected hand
        hand_landmarks = results.multi_hand_landmarks[0]
        
        # Convert to numpy array (21 landmarks × 3 coords = 63 features)
        landmarks = []
        for landmark in hand_landmarks.landmark:
            landmarks.extend([landmark.x, landmark.y, landmark.z])
        
        return np.array(landmarks)
    
    def normalize_landmarks(self, landmarks):
        """
        Normalize landmarks to be scale and position invariant.
        
        Args:
            landmarks: numpy array of 63 features
            
        Returns:
            normalized numpy array
        """
        # Reshape to (21, 3)
        landmarks_reshaped = landmarks.reshape(21, 3)
        
        # Get wrist position (landmark 0)
        wrist = landmarks_reshaped[0]
        
        # Translate to origin (wrist at 0,0,0)
        landmarks_centered = landmarks_reshaped - wrist
        
        # Calculate hand size (distance from wrist to middle finger MCP)
        middle_mcp = landmarks_centered[9]  # Middle finger MCP
        hand_size = np.linalg.norm(middle_mcp)
        
        # Scale by hand size (avoid division by zero)
        if hand_size > 0.001:
            landmarks_normalized = landmarks_centered / hand_size
        else:
            landmarks_normalized = landmarks_centered
        
        # Flatten back to 63 features
        return landmarks_normalized.flatten()
    
    def load_training_images(self):
        """
        Load all training images and extract landmarks.
        
        Returns:
            (features, labels) - numpy arrays for training
        """
        print(f'\n[Loading] Training images from: {self.person_dir}')
        
        features = []
        labels = []
        
        if not self.person_dir.exists():
            raise FileNotFoundError(f'Person directory not found: {self.person_dir}')
        
        # Load images for each gesture
        for gesture_name in self.gestures:
            gesture_dir = self.person_dir / gesture_name
            
            if not gesture_dir.exists():
                print(f'  ⚠️  Warning: {gesture_name} directory not found, skipping')
                continue
            
            image_files = sorted(list(gesture_dir.glob('*.jpg')))
            
            if not image_files:
                print(f'  ⚠️  Warning: No images found for {gesture_name}, skipping')
                continue
            
            print(f'\n  Processing {gesture_name}:')
            print(f'    Found {len(image_files)} image files')
            print(f'    Extracting landmarks...', end='', flush=True)
            
            valid_count = 0
            for idx, img_path in enumerate(image_files):
                try:
                    # Load image
                    img = cv2.imread(str(img_path))
                    
                    if img is None:
                        continue
                    
                    # Extract landmarks
                    landmarks = self.extract_hand_landmarks(img)
                    
                    if landmarks is None:
                        continue
                    
                    # Normalize landmarks
                    landmarks_normalized = self.normalize_landmarks(landmarks)
                    
                    features.append(landmarks_normalized)
                    labels.append(self.gesture_to_label[gesture_name])
                    valid_count += 1
                    
                    if (idx + 1) % 10 == 0:
                        print(f'.', end='', flush=True)
                
                except Exception as e:
                    continue
            
            print(f' Done!')
            print(f'    ✓ Successfully loaded: {valid_count} images')
        
        if len(features) == 0:
            raise ValueError('No valid training images loaded')
        
        print(f'\n  ✓ Total samples loaded: {len(features)}')
        print(f'  ✓ Gesture distribution:')
        for gesture_name, label in self.gesture_to_label.items():
            count = sum(1 for l in labels if l == label)
            print(f'      {gesture_name}: {count} samples')
        
        return np.array(features), np.array(labels)
    
    def train(self):
        """Train gesture classifier on loaded features."""
        print(f'\n[Training] Gesture Classifier')
        print(f'  Person: {self.person_name}')
        
        features, labels = self.load_training_images()
        
        print(f'\n[Processing] Training SVM classifier...')
        print(f'  Features shape: {features.shape}')
        print(f'  Labels shape: {labels.shape}')
        
        try:
            # Normalize features
            features_scaled = self.scaler.fit_transform(features)
            
            # Train classifier
            self.classifier.fit(features_scaled, labels)
            
            # Calculate training accuracy
            train_accuracy = self.classifier.score(features_scaled, labels)
            
            print(f'  ✓ Training complete')
            print(f'  ✓ Training accuracy: {train_accuracy*100:.1f}%')
            
        except Exception as e:
            raise RuntimeError(f'Classifier training failed: {e}')
    
    def save_model(self):
        """Save trained model to file."""
        print(f'\n[Saving] Model to: {self.model_path}')
        
        try:
            # Package model, scaler, and metadata
            model_data = {
                'classifier': self.classifier,
                'scaler': self.scaler,
                'person_name': self.person_name,
                'gestures': self.gestures,
                'gesture_to_label': self.gesture_to_label,
                'label_to_gesture': {v: k for k, v in self.gesture_to_label.items()},
                'training_date': str(np.datetime64('now'))
            }
            
            # Save as pickle
            with open(self.model_path, 'wb') as f:
                pickle.dump(model_data, f)
            
            file_size = self.model_path.stat().st_size
            print(f'  ✓ Model saved successfully')
            print(f'  ✓ File size: {file_size / 1024:.1f} KB')
            
        except Exception as e:
            raise RuntimeError(f'Failed to save model: {e}')
    
    def run(self):
        """Run the complete training sequence."""
        print('\n' + '='*70)
        print(f'TRAINING GESTURE CLASSIFIER FOR: {self.person_name.upper()}')
        print('='*70)
        
        try:
            print('\nThis will:')
            print('  1. Load all captured gesture images')
            print('  2. Extract hand landmarks using MediaPipe')
            print('  3. Normalize features (scale/position invariant)')
            print('  4. Train SVM classifier')
            print('  5. Save trained model')
            print()
            
            # Auto-continue if not in interactive terminal
            if sys.stdin.isatty():
                input('Press ENTER to continue...')
            else:
                print('Auto-continuing...')
                import time
                time.sleep(1)
            
            self.train()
            self.save_model()
            
            print('\n' + '='*70)
            print('TRAINING COMPLETE')
            print('='*70)
            print(f'\n✓ Model trained and saved: {self.person_name}_gesture_classifier.pkl')
            print(f'\nNext step: Test gesture classifier with real-time recognition.')
            print('='*70 + '\n')
            
            return True
        
        except Exception as e:
            print(f'\n❌ Training failed: {e}')
            import traceback
            traceback.print_exc()
            return False
        finally:
            self.hands.close()


if __name__ == '__main__':
    if len(sys.argv) != 3:
        print('Usage: python3 _gesture_train_module.py <person_name> <gesture_recognition_data_dir>')
        sys.exit(1)
    
    try:
        person_name = sys.argv[1]
        data_dir = sys.argv[2]
        
        training = GestureTrainingModule(person_name, data_dir)
        success = training.run()
        sys.exit(0 if success else 1)
    except Exception as e:
        print(f'\n❌ Error: {e}')
        import traceback
        traceback.print_exc()
        sys.exit(1)


