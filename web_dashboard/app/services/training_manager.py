"""Training task management for face recognition"""
import subprocess
import asyncio
import os
from pathlib import Path
from typing import Dict, Optional, List
from datetime import datetime
import uuid
from app.config import TRAINING_SCRIPT_DIR, TRAINING_BASE_DIR


class TrainingTaskManager:
    """Manages face recognition training tasks"""
    
    def __init__(self):
        self.tasks: Dict[str, Dict] = {}
        self.script_dir = TRAINING_SCRIPT_DIR
        self.base_dir = TRAINING_BASE_DIR
    
    def create_task(self, task_type: str, person_name: str, option: int = None) -> str:
        """
        Create a new training task.
        
        Args:
            task_type: Type of task (capture, add_pictures, retrain, etc.)
            person_name: Name of person
            option: Menu option number (for reference)
        
        Returns:
            Task ID
        """
        task_id = str(uuid.uuid4())
        self.tasks[task_id] = {
            'task_id': task_id,
            'type': task_type,
            'person_name': person_name,
            'option': option,
            'status': 'running',
            'progress': 0,
            'current_step': 'Initializing...',
            'logs': [],
            'started_at': datetime.now().isoformat(),
            'completed_at': None,
            'error': None
        }
        return task_id
    
    async def run_capture(self, task_id: str, person_name: str) -> None:
        """Run training image capture"""
        script = self.script_dir / '1_capture_training_data.py'
        if not script.exists():
            self.tasks[task_id]['status'] = 'failed'
            self.tasks[task_id]['error'] = f"Script not found: {script}"
            return
        
        env = os.environ.copy()
        env['OPENBLAS_CORETYPE'] = 'ARMV8'
        
        # Try to use depthai_env
        depthai_env = Path.home() / 'depthai_env'
        if depthai_env.exists():
            env['PATH'] = str(depthai_env / 'bin') + ':' + env.get('PATH', '')
            env['VIRTUAL_ENV'] = str(depthai_env)
        
        try:
            self.tasks[task_id]['current_step'] = 'Starting capture...'
            process = await asyncio.create_subprocess_exec(
                'python3', str(script), person_name, str(self.base_dir),
                stdout=asyncio.subprocess.PIPE,
                stderr=asyncio.subprocess.STDOUT,
                env=env
            )
            
            # Stream logs
            while True:
                line = await process.stdout.readline()
                if not line:
                    break
                log_line = line.decode().strip()
                self.tasks[task_id]['logs'].append(log_line)
                # Update progress based on log content
                if 'Task' in log_line or 'Step' in log_line:
                    self.tasks[task_id]['current_step'] = log_line
            
            await process.wait()
            
            if process.returncode == 0:
                self.tasks[task_id]['status'] = 'completed'
            else:
                self.tasks[task_id]['status'] = 'failed'
                self.tasks[task_id]['error'] = f"Process exited with code {process.returncode}"
            
            self.tasks[task_id]['completed_at'] = datetime.now().isoformat()
            
        except Exception as e:
            self.tasks[task_id]['status'] = 'failed'
            self.tasks[task_id]['error'] = str(e)
            self.tasks[task_id]['completed_at'] = datetime.now().isoformat()
    
    async def run_train(self, task_id: str, person_name: str) -> None:
        """Run model training"""
        script = self.script_dir / '2_train_recognizer.py'
        if not script.exists():
            self.tasks[task_id]['status'] = 'failed'
            self.tasks[task_id]['error'] = f"Script not found: {script}"
            return
        
        env = os.environ.copy()
        env['OPENBLAS_CORETYPE'] = 'ARMV8'
        
        try:
            self.tasks[task_id]['current_step'] = 'Training model...'
            process = await asyncio.create_subprocess_exec(
                'python3', str(script), person_name, str(self.base_dir),
                stdout=asyncio.subprocess.PIPE,
                stderr=asyncio.subprocess.STDOUT,
                env=env
            )
            
            while True:
                line = await process.stdout.readline()
                if not line:
                    break
                log_line = line.decode().strip()
                self.tasks[task_id]['logs'].append(log_line)
                self.tasks[task_id]['current_step'] = log_line
            
            await process.wait()
            self.tasks[task_id]['status'] = 'completed' if process.returncode == 0 else 'failed'
            self.tasks[task_id]['completed_at'] = datetime.now().isoformat()
            
        except Exception as e:
            self.tasks[task_id]['status'] = 'failed'
            self.tasks[task_id]['error'] = str(e)
            self.tasks[task_id]['completed_at'] = datetime.now().isoformat()
    
    def get_task_status(self, task_id: str) -> Optional[Dict]:
        """Get training task status"""
        return self.tasks.get(task_id)
    
    def list_trained_people(self) -> List[Dict]:
        """List all trained people and their models"""
        models_dir = self.base_dir / 'models'
        trained = []
        
        if models_dir.exists():
            for model_file in sorted(models_dir.glob('*_lbph.xml')):
                person_name = model_file.stem.replace('_lbph', '')
                size_kb = model_file.stat().st_size / 1024
                
                # Count images
                person_dir = self.base_dir / person_name
                image_count = 0
                if person_dir.exists():
                    image_count = len(list(person_dir.glob('*.jpg')))
                
                trained.append({
                    'name': person_name,
                    'model_size_kb': round(size_kb, 1),
                    'image_count': image_count,
                    'model_path': str(model_file)
                })
        
        return trained
    
    def list_datasets(self) -> List[Dict]:
        """List all training datasets"""
        datasets = []
        
        if self.base_dir.exists():
            for person_dir in self.base_dir.iterdir():
                if person_dir.is_dir() and person_dir.name != 'models':
                    image_count = len(list(person_dir.glob('*.jpg')))
                    if image_count > 0:
                        datasets.append({
                            'name': person_dir.name,
                            'image_count': image_count
                        })
        
        return datasets
    
    def delete_person(self, person_name: str) -> Dict[str, any]:
        """
        Delete a person's training data and model.
        
        Args:
            person_name: Name of person to delete
        
        Returns:
            Success status
        """
        # Security check: Validate path traversal characters
        if '..' in person_name or '/' in person_name or '\\' in person_name:
            return {"success": False, "error": "Invalid person name"}
            
        try:
            # Delete model
            model_file = (self.base_dir / 'models' / f'{person_name}_lbph.xml').resolve()
            
            # Security check: Ensure path is within base directory
            if not str(model_file).startswith(str(self.base_dir.resolve())):
                return {"success": False, "error": "Invalid path"}

            if model_file.exists():
                model_file.unlink()
            
            # Delete images directory
            person_dir = (self.base_dir / person_name).resolve()
            
            # Security check: Ensure path is within base directory
            if not str(person_dir).startswith(str(self.base_dir.resolve())):
                 # Don't error here if model was deleted, just don't delete dir
                 pass 
            elif person_dir.exists():
                shutil.rmtree(person_dir)
            
            return {"success": True, "message": f"Deleted {person_name}"}
        except Exception as e:
            return {"success": False, "error": str(e)}

