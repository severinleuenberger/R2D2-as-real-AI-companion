"""Service control API endpoints"""
from fastapi import APIRouter, HTTPException
from app.services.service_manager import ServiceManager

router = APIRouter(prefix="/api/services", tags=["services"])
service_manager = ServiceManager()


@router.get("/status")
async def get_all_services_status():
    """Get status of all R2D2 services"""
    return service_manager.get_all_services_status()


# IMPORTANT: Specific routes must come BEFORE parameterized routes
# Otherwise FastAPI will match /recognition/start to /{service_name}/start

@router.post("/recognition/start")
async def start_recognition_mode():
    """Start recognition mode (stops stream, starts camera-perception + audio)"""
    import json
    import traceback
    from datetime import datetime
    
    # #region agent log
    try:
        with open('/home/severin/.cursor/debug.log', 'a') as f:
            f.write(json.dumps({"sessionId":"debug-session","runId":"recognition-start","hypothesisId":"A","location":"services.py:59","message":"Recognition start endpoint called","data":{},"timestamp":int(datetime.now().timestamp()*1000)}) + "\n")
    except: pass
    # #endregion
    
    try:
        result = service_manager.start_recognition_mode()
        
        # #region agent log
        try:
            with open('/home/severin/.cursor/debug.log', 'a') as f:
                f.write(json.dumps({"sessionId":"debug-session","runId":"recognition-start","hypothesisId":"A","location":"services.py:67","message":"Service manager result","data":{"success":result.get("success"),"error":result.get("error"),"message":result.get("message")},"timestamp":int(datetime.now().timestamp()*1000)}) + "\n")
        except: pass
        # #endregion
        
        if not result.get("success"):
            error_msg = result.get("error", "Unknown error")
            # #region agent log
            try:
                with open('/home/severin/.cursor/debug.log', 'a') as f:
                    f.write(json.dumps({"sessionId":"debug-session","runId":"recognition-start","hypothesisId":"A","location":"services.py:73","message":"Raising HTTPException","data":{"error":error_msg,"result_keys":list(result.keys())},"timestamp":int(datetime.now().timestamp()*1000)}) + "\n")
            except: pass
            # #endregion
            raise HTTPException(status_code=500, detail=error_msg)
        return result
    except HTTPException:
        raise
    except Exception as e:
        # #region agent log
        try:
            with open('/home/severin/.cursor/debug.log', 'a') as f:
                f.write(json.dumps({"sessionId":"debug-session","runId":"recognition-start","hypothesisId":"B","location":"services.py:81","message":"Exception in recognition start","data":{"error":str(e),"type":type(e).__name__,"traceback":traceback.format_exc()},"timestamp":int(datetime.now().timestamp()*1000)}) + "\n")
        except: pass
        # #endregion
        raise HTTPException(status_code=500, detail=f"Internal error: {str(e)}")


@router.post("/recognition/stop")
async def stop_recognition_mode():
    """Stop recognition mode (stops camera-perception + audio)"""
    result = service_manager.stop_recognition_mode()
    if not result.get("success"):
        raise HTTPException(status_code=500, detail=result.get("error", "Unknown error"))
    return result


@router.get("/recognition/command/{action}")
async def get_recognition_command(action: str):
    """Get command-line instruction for recognition mode"""
    if action not in ["start", "stop"]:
        raise HTTPException(status_code=400, detail="Action must be 'start' or 'stop'")
    command = service_manager.get_recognition_mode_command(action)
    return {"command": command}


@router.post("/stream/start")
async def start_stream_mode():
    """Start stream mode (stops recognition, starts stream)"""
    result = service_manager.start_stream_mode()
    if not result.get("success"):
        raise HTTPException(status_code=500, detail=result.get("error", "Unknown error"))
    return result


@router.post("/stream/stop")
async def stop_stream_mode():
    """Stop stream mode (stops stream)"""
    result = service_manager.stop_stream_mode()
    if not result.get("success"):
        raise HTTPException(status_code=500, detail=result.get("error", "Unknown error"))
    return result


@router.get("/stream/command/{action}")
async def get_stream_command(action: str):
    """Get command-line instruction for stream mode"""
    if action not in ["start", "stop"]:
        raise HTTPException(status_code=400, detail="Action must be 'start' or 'stop'")
    command = service_manager.get_stream_mode_command(action)
    return {"command": command}


# Generic parameterized routes - must come AFTER specific routes
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


@router.get("/{service_name}/command/{action}")
async def get_service_command(service_name: str, action: str):
    """Get command-line instruction for a service action"""
    command = service_manager.get_service_command(service_name, action)
    return {"command": command}

