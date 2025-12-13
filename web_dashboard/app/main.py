"""Main FastAPI application for R2D2 Web Dashboard"""
import json
import sys
from datetime import datetime

# #region agent log
try:
    with open('/home/severin/.cursor/debug.log', 'a') as f:
        f.write(json.dumps({"sessionId":"debug-session","runId":"startup","hypothesisId":"B","location":"main.py:7","message":"Starting imports","data":{"python":sys.executable},"timestamp":int(datetime.now().timestamp()*1000)}) + "\n")
except: pass
# #endregion

from fastapi import FastAPI
from fastapi.staticfiles import StaticFiles
from fastapi.responses import FileResponse
from pathlib import Path

# #region agent log
try:
    with open('/home/severin/.cursor/debug.log', 'a') as f:
        f.write(json.dumps({"sessionId":"debug-session","runId":"startup","hypothesisId":"B","location":"main.py:18","message":"FastAPI imported","data":{},"timestamp":int(datetime.now().timestamp()*1000)}) + "\n")
except: pass
# #endregion

from app.config import BASE_DIR, HOST, PORT

# #region agent log
try:
    with open('/home/severin/.cursor/debug.log', 'a') as f:
        f.write(json.dumps({"sessionId":"debug-session","runId":"startup","hypothesisId":"B","location":"main.py:23","message":"Config imported","data":{"host":HOST,"port":PORT,"base_dir":str(BASE_DIR)},"timestamp":int(datetime.now().timestamp()*1000)}) + "\n")
except: pass
# #endregion

try:
    from app.api import services, audio, training, status
    # #region agent log
    try:
        with open('/home/severin/.cursor/debug.log', 'a') as f:
            f.write(json.dumps({"sessionId":"debug-session","runId":"startup","hypothesisId":"B","location":"main.py:30","message":"API modules imported","data":{},"timestamp":int(datetime.now().timestamp()*1000)}) + "\n")
    except: pass
    # #endregion
except Exception as e:
    # #region agent log
    try:
        with open('/home/severin/.cursor/debug.log', 'a') as f:
            f.write(json.dumps({"sessionId":"debug-session","runId":"startup","hypothesisId":"B","location":"main.py:33","message":"API import failed","data":{"error":str(e),"type":type(e).__name__},"timestamp":int(datetime.now().timestamp()*1000)}) + "\n")
    except: pass
    # #endregion
    raise

# Create FastAPI app
# #region agent log
try:
    with open('/home/severin/.cursor/debug.log', 'a') as f:
        f.write(json.dumps({"sessionId":"debug-session","runId":"startup","hypothesisId":"B","location":"main.py:40","message":"Creating FastAPI app","data":{},"timestamp":int(datetime.now().timestamp()*1000)}) + "\n")
except: pass
# #endregion

app = FastAPI(
    title="R2D2 Web Dashboard",
    description="Monitoring and control dashboard for R2D2 system",
    version="1.0.0"
)

# #region agent log
try:
    with open('/home/severin/.cursor/debug.log', 'a') as f:
        f.write(json.dumps({"sessionId":"debug-session","runId":"startup","hypothesisId":"B","location":"main.py:48","message":"FastAPI app created","data":{},"timestamp":int(datetime.now().timestamp()*1000)}) + "\n")
except: pass
# #endregion

# Include API routers
try:
    app.include_router(services.router)
    app.include_router(audio.router)
    app.include_router(training.router)
    app.include_router(status.router)
    # #region agent log
    try:
        with open('/home/severin/.cursor/debug.log', 'a') as f:
            f.write(json.dumps({"sessionId":"debug-session","runId":"startup","hypothesisId":"B","location":"main.py:55","message":"Routers included","data":{},"timestamp":int(datetime.now().timestamp()*1000)}) + "\n")
    except: pass
    # #endregion
except Exception as e:
    # #region agent log
    try:
        with open('/home/severin/.cursor/debug.log', 'a') as f:
            f.write(json.dumps({"sessionId":"debug-session","runId":"startup","hypothesisId":"B","location":"main.py:58","message":"Router inclusion failed","data":{"error":str(e),"type":type(e).__name__},"timestamp":int(datetime.now().timestamp()*1000)}) + "\n")
    except: pass
    # #endregion
    raise

# Serve static files
static_dir = BASE_DIR / "app" / "static"
if static_dir.exists():
    app.mount("/static", StaticFiles(directory=str(static_dir)), name="static")

# Serve main dashboard page
templates_dir = BASE_DIR / "app" / "templates"


@app.get("/")
async def index():
    """Serve main dashboard page"""
    index_file = templates_dir / "index.html"
    if index_file.exists():
        return FileResponse(str(index_file))
    else:
        return {"message": "Dashboard not found. Please create index.html"}


@app.get("/health")
async def health():
    """Health check endpoint"""
    return {"status": "ok", "service": "r2d2-web-dashboard"}


if __name__ == "__main__":
    # #region agent log
    try:
        with open('/home/severin/.cursor/debug.log', 'a') as f:
            f.write(json.dumps({"sessionId":"debug-session","runId":"startup","hypothesisId":"B","location":"main.py:75","message":"About to start uvicorn","data":{"host":HOST,"port":PORT},"timestamp":int(datetime.now().timestamp()*1000)}) + "\n")
    except: pass
    # #endregion
    
    import uvicorn
    try:
        # #region agent log
        try:
            with open('/home/severin/.cursor/debug.log', 'a') as f:
                f.write(json.dumps({"sessionId":"debug-session","runId":"startup","hypothesisId":"B","location":"main.py:82","message":"Uvicorn imported, starting server","data":{},"timestamp":int(datetime.now().timestamp()*1000)}) + "\n")
        except: pass
        # #endregion
        uvicorn.run(app, host=HOST, port=PORT)
    except Exception as e:
        # #region agent log
        try:
            with open('/home/severin/.cursor/debug.log', 'a') as f:
                f.write(json.dumps({"sessionId":"debug-session","runId":"startup","hypothesisId":"B","location":"main.py:87","message":"Uvicorn start failed","data":{"error":str(e),"type":type(e).__name__},"timestamp":int(datetime.now().timestamp()*1000)}) + "\n")
        except: pass
        # #endregion
        raise


