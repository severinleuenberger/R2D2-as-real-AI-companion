"""Training control API endpoints"""
from fastapi import APIRouter, HTTPException, BackgroundTasks
from pydantic import BaseModel, Field
from app.services.training_manager import TrainingTaskManager
from typing import Optional

router = APIRouter(prefix="/api/training", tags=["training"])
task_manager = TrainingTaskManager()


class TrainingRequest(BaseModel):
    person_name: str = Field(..., description="Name of person to train")
    option: Optional[int] = Field(None, description="Menu option number (for reference)")


@router.post("/capture")
async def start_capture(request: TrainingRequest, background_tasks: BackgroundTasks):
    """Start training image capture (Option 1)"""
    task_id = task_manager.create_task("capture", request.person_name, option=1)
    background_tasks.add_task(task_manager.run_capture, task_id, request.person_name)
    return {"task_id": task_id, "status": "running", "message": "Capture started"}


@router.post("/add_pictures")
async def add_pictures(request: TrainingRequest, background_tasks: BackgroundTasks):
    """Add more training pictures (Option 2)"""
    task_id = task_manager.create_task("add_pictures", request.person_name, option=2)
    background_tasks.add_task(task_manager.run_capture, task_id, request.person_name)
    return {"task_id": task_id, "status": "running", "message": "Adding pictures started"}


@router.post("/retrain")
async def retrain(request: TrainingRequest, background_tasks: BackgroundTasks):
    """Retrain model from existing images (Option 3)"""
    task_id = task_manager.create_task("retrain", request.person_name, option=3)
    background_tasks.add_task(task_manager.run_train, task_id, request.person_name)
    return {"task_id": task_id, "status": "running", "message": "Retraining started"}


@router.post("/test_accuracy")
async def test_accuracy(request: TrainingRequest):
    """Test accuracy at different distances (Option 4)"""
    # This would run synchronously as it's a test
    return {"message": "Test accuracy not yet implemented", "person_name": request.person_name}


@router.post("/realtime_test")
async def realtime_test(request: TrainingRequest):
    """Real-time recognition test (Option 5)"""
    # This would run synchronously as it's a test
    return {"message": "Real-time test not yet implemented", "person_name": request.person_name}


@router.get("/list")
async def list_people():
    """List all trained people and models (Option 6)"""
    trained = task_manager.list_trained_people()
    datasets = task_manager.list_datasets()
    return {
        "trained_people": trained,
        "datasets": datasets
    }


@router.delete("/{person_name}")
async def delete_person(person_name: str):
    """Delete person (images + model) (Option 7)"""
    result = task_manager.delete_person(person_name)
    if not result.get("success"):
        raise HTTPException(status_code=500, detail=result.get("error", "Unknown error"))
    return result


@router.get("/status/{task_id}")
async def get_training_status(task_id: str):
    """Get training task status"""
    status = task_manager.get_task_status(task_id)
    if not status:
        raise HTTPException(status_code=404, detail="Task not found")
    return status


