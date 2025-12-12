"""Audio control API endpoints"""
from fastapi import APIRouter, HTTPException
from pydantic import BaseModel, Field
from app.services.ros2_control import ROS2ParameterControl

router = APIRouter(prefix="/api/audio", tags=["audio"])
ros2_control = ROS2ParameterControl()


class VolumeRequest(BaseModel):
    volume: float = Field(ge=0.0, le=1.0, description="Volume level (0.0 to 1.0)")


@router.get("/volume")
async def get_volume():
    """Get current audio volume"""
    volume = ros2_control.get_audio_volume()
    if volume is None:
        raise HTTPException(status_code=500, detail="Failed to get volume (node may not be running)")
    return {"volume": volume}


@router.post("/volume")
async def set_volume(request: VolumeRequest):
    """Set audio volume"""
    success = ros2_control.set_audio_volume(request.volume)
    if not success:
        raise HTTPException(status_code=500, detail="Failed to set volume")
    
    # Verify it was set
    new_volume = ros2_control.get_audio_volume()
    return {"success": True, "volume": new_volume}


@router.get("/parameters")
async def get_audio_parameters():
    """Get all audio notification parameters"""
    params = ros2_control.get_audio_parameters()
    if not params:
        raise HTTPException(status_code=500, detail="Failed to get parameters (node may not be running)")
    return params


