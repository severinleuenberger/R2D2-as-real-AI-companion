#!/usr/bin/env python3
"""
Reduce Training Images - Quality Selection Tool

Purpose:
  Intelligently reduces training dataset from 900+ images down to 50-100
  high-quality, diverse images for optimal LBPH face recognition training.

Strategy:
  1. Assess quality (blur, lighting, contrast)
  2. Calculate diversity (avoid duplicates)
  3. Keep best images from each stage
  4. Archive extras (not delete - backup first!)

Usage:
  python3 reduce_training_images.py <person_name>

Author: R2D2 Training System
Date: December 27, 2025
"""

import cv2
import numpy as np
import os
import sys
import shutil
from pathlib import Path
from datetime import datetime


class ImageReducer:
    """Intelligently reduces training dataset to optimal size."""
    
    def __init__(self, person_name, base_dir=None):
        """Initialize image reducer."""
        if base_dir is None:
            base_dir = Path.home() / 'dev' / 'r2d2' / 'data' / 'face_recognition'
        else:
            base_dir = Path(base_dir)
        
        self.person_name = person_name
        self.data_dir = base_dir / person_name
        self.archive_dir = base_dir / f'{person_name}_archive_{datetime.now().strftime("%Y%m%d_%H%M%S")}'
        
        if not self.data_dir.exists():
            raise ValueError(f'Person directory not found: {self.data_dir}')
        
        # Target: 50-100 high-quality images
        self.target_images = 75
        self.min_quality_score = 0.4  # Lower threshold for existing images
    
    def assess_quality(self, image_path):
        """
        Assess quality of a face image.
        
        Returns:
            tuple: (quality_score, details)
        """
        img = cv2.imread(str(image_path), cv2.IMREAD_GRAYSCALE)
        if img is None:
            return 0.0, {}
        
        # 1. Blur detection (Laplacian variance)
        laplacian_var = cv2.Laplacian(img, cv2.CV_64F).var()
        blur_score = min(laplacian_var / 100.0, 1.0)
        
        # 2. Lighting (mean brightness)
        mean_brightness = np.mean(img)
        brightness_diff = abs(mean_brightness - 128) / 128.0
        lighting_score = max(0.0, 1.0 - brightness_diff)
        
        # 3. Contrast (std dev)
        contrast = np.std(img) / 128.0
        contrast_score = min(contrast, 1.0)
        
        # Combined score
        quality_score = (
            blur_score * 0.5 +
            lighting_score * 0.3 +
            contrast_score * 0.2
        )
        
        details = {
            'blur': blur_score,
            'lighting': lighting_score,
            'contrast': contrast_score,
            'laplacian_var': laplacian_var,
            'image': img
        }
        
        return quality_score, details
    
    def calculate_diversity_score(self, img, existing_images):
        """
        Calculate how diverse this image is compared to existing selected images.
        
        Args:
            img: Grayscale image
            existing_images: List of already selected images
            
        Returns:
            float: Diversity score (0.0 = duplicate, 1.0 = very diverse)
        """
        if len(existing_images) == 0:
            return 1.0
        
        # Compare with existing images using histogram
        hist1 = cv2.calcHist([img], [0], None, [256], [0, 256])
        
        similarities = []
        for existing_img in existing_images:
            hist2 = cv2.calcHist([existing_img], [0], None, [256], [0, 256])
            similarity = cv2.compareHist(hist1, hist2, cv2.HISTCMP_CORREL)
            similarities.append(similarity)
        
        # Average similarity to existing images
        avg_similarity = np.mean(similarities)
        
        # Convert to diversity score (inverse of similarity)
        diversity_score = 1.0 - avg_similarity
        
        return diversity_score
    
    def select_best_images(self, target_count=75):
        """
        Select best images based on quality and diversity.
        
        Args:
            target_count: Number of images to keep
            
        Returns:
            tuple: (selected_paths, archive_paths)
        """
        print(f'\n[Analysis] Scanning images...')
        
        # Get all image files
        image_files = sorted(self.data_dir.glob('*.jpg'))
        print(f'  Found {len(image_files)} images')
        
        if len(image_files) <= target_count:
            print(f'  Already at or below target ({target_count}). No reduction needed.')
            return image_files, []
        
        # Assess quality of all images
        print(f'\n[Quality Assessment] Analyzing {len(image_files)} images...')
        image_scores = []
        
        for i, img_path in enumerate(image_files):
            quality_score, details = self.assess_quality(img_path)
            image_scores.append({
                'path': img_path,
                'quality': quality_score,
                'details': details
            })
            
            if (i + 1) % 100 == 0:
                print(f'  Processed {i+1}/{len(image_files)} images...')
        
        # Sort by quality
        image_scores.sort(key=lambda x: x['quality'], reverse=True)
        
        # Filter out very low quality images first
        filtered_scores = [x for x in image_scores if x['quality'] >= self.min_quality_score]
        print(f'\n[Filtering] {len(filtered_scores)} images pass quality threshold (>= {self.min_quality_score})')
        
        if len(filtered_scores) <= target_count:
            print(f'  After filtering, we have {len(filtered_scores)} images (target: {target_count})')
            selected = [x['path'] for x in filtered_scores]
            archive = [x['path'] for x in image_scores if x['path'] not in selected]
            return selected, archive
        
        # Now select diverse images
        print(f'\n[Diversity Selection] Selecting {target_count} diverse images...')
        selected_images = []
        selected_paths = []
        
        for i, img_info in enumerate(filtered_scores):
            if len(selected_paths) >= target_count:
                break
            
            img = img_info['details']['image']
            
            # Calculate diversity score
            diversity_score = self.calculate_diversity_score(img, selected_images)
            
            # Combined score: quality (70%) + diversity (30%)
            combined_score = img_info['quality'] * 0.7 + diversity_score * 0.3
            
            # Accept if diverse enough or if we need more images
            if diversity_score > 0.3 or len(selected_paths) < target_count * 0.8:
                selected_images.append(img)
                selected_paths.append(img_info['path'])
                
                if (len(selected_paths)) % 10 == 0:
                    print(f'  Selected {len(selected_paths)}/{target_count} images...')
        
        # Archive the rest
        archive_paths = [img['path'] for img in image_scores if img['path'] not in selected_paths]
        
        print(f'\n[Results]')
        print(f'  Selected: {len(selected_paths)} high-quality diverse images')
        print(f'  Archive:  {len(archive_paths)} images')
        
        return selected_paths, archive_paths
    
    def run(self, dry_run=False):
        """
        Run the image reduction process.
        
        Args:
            dry_run: If True, only show what would be done
        """
        print('='*70)
        print(f'IMAGE REDUCER: {self.person_name.upper()}')
        print('='*70)
        print(f'\nData directory: {self.data_dir}')
        print(f'Target images: {self.target_images}')
        print(f'Archive directory: {self.archive_dir}')
        
        if dry_run:
            print('\n⚠️  DRY RUN MODE - No files will be moved')
        
        print('\n' + '='*70)
        
        # Select best images
        selected_paths, archive_paths = self.select_best_images(self.target_images)
        
        if len(archive_paths) == 0:
            print('\n✅ No reduction needed! Image count is optimal.')
            return
        
        # Show summary
        print('\n' + '='*70)
        print('SUMMARY')
        print('='*70)
        print(f'  Keep:    {len(selected_paths)} images (in {self.data_dir})')
        print(f'  Archive: {len(archive_paths)} images (to {self.archive_dir})')
        print()
        
        if dry_run:
            print('DRY RUN - No changes made.')
            return
        
        # Confirm before proceeding
        print('This will:')
        print(f'  1. Create archive directory: {self.archive_dir}')
        print(f'  2. Move {len(archive_paths)} images to archive')
        print(f'  3. Keep {len(selected_paths)} best images in {self.data_dir}')
        print()
        response = input('Proceed? (yes/no): ').strip().lower()
        
        if response != 'yes':
            print('❌ Cancelled.')
            return
        
        # Create archive directory
        self.archive_dir.mkdir(parents=True, exist_ok=True)
        print(f'\n✓ Created archive directory')
        
        # Move images to archive
        print(f'\n[Moving] Archiving {len(archive_paths)} images...')
        for i, img_path in enumerate(archive_paths):
            dest_path = self.archive_dir / img_path.name
            shutil.move(str(img_path), str(dest_path))
            
            if (i + 1) % 100 == 0:
                print(f'  Moved {i+1}/{len(archive_paths)} images...')
        
        print(f'\n✅ COMPLETE')
        print('='*70)
        print(f'  Kept:     {len(selected_paths)} images in {self.data_dir}')
        print(f'  Archived: {len(archive_paths)} images in {self.archive_dir}')
        print('\nYou can now train the model with optimal dataset size.')
        print('='*70)


if __name__ == '__main__':
    if len(sys.argv) < 2:
        print('Usage: python3 reduce_training_images.py <person_name> [--dry-run]')
        sys.exit(1)
    
    person_name = sys.argv[1]
    dry_run = '--dry-run' in sys.argv
    
    try:
        reducer = ImageReducer(person_name)
        reducer.run(dry_run=dry_run)
    except Exception as e:
        print(f'\n❌ Error: {e}')
        import traceback
        traceback.print_exc()
        sys.exit(1)

