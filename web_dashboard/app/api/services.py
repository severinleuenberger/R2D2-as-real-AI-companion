"""Service control API endpoints"""
from fastapi import APIRouter, HTTPException
from app.services.service_manager import ServiceManager

router = APIRouter(prefix="/api/services", tags=["services"])
service_manager = ServiceManager()


@router.get("/status")
async def get_all_services_status():
    """Get status of all R2D2 services"""
    return service_manager.get_all_services_status()


@router.get("/{service_name}/status")
async def get_service_status(service_name: str):
    """Get status of a specific service"""
    status = service_manager.get_status(service_name)
    if "error" in status:
        raise HTTPException(status_code=404, detail=status["error"])
    return status


@router.post("/{service_name}/start")
async def start_service(service_name: str):
    """Start a systemd service"""
    result = service_manager.start_service(service_name)
    if not result.get("success"):
        raise HTTPException(status_code=500, detail=result.get("error", "Unknown error"))
    return result


@router.post("/{service_name}/stop")
async def stop_service(service_name: str):
    """Stop a systemd service"""
    result = service_manager.stop_service(service_name)
    if not result.get("success"):
        raise HTTPException(status_code=500, detail=result.get("error", "Unknown error"))
    return result


@router.post("/{service_name}/restart")
async def restart_service(service_name: str):
    """Restart a systemd service"""
    result = service_manager.restart_service(service_name)
    if not result.get("success"):
        raise HTTPException(status_code=500, detail=result.get("error", "Unknown error"))
    return result


