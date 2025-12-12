"""Status and monitoring API endpoints"""
from fastapi import APIRouter
import subprocess

router = APIRouter(prefix="/api/status", tags=["status"])


@router.get("/nodes")
async def get_ros_nodes():
    """Get list of running ROS 2 nodes"""
    result = subprocess.run(
        ['ros2', 'node', 'list'],
        capture_output=True, text=True
    )
    nodes = [line.strip() for line in result.stdout.strip().split('\n') if line.strip()]
    return {"nodes": nodes}


@router.get("/topics")
async def get_ros_topics():
    """Get list of ROS 2 topics"""
    result = subprocess.run(
        ['ros2', 'topic', 'list'],
        capture_output=True, text=True
    )
    topics = [line.strip() for line in result.stdout.strip().split('\n') if line.strip()]
    return {"topics": topics}


