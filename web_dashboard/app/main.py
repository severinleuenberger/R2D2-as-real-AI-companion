"""Main FastAPI application for R2D2 Web Dashboard"""
from fastapi import FastAPI
from fastapi.staticfiles import StaticFiles
from fastapi.responses import FileResponse
from pathlib import Path
from app.config import BASE_DIR, HOST, PORT
from app.api import services, audio, training, status, system, database

# Create FastAPI app
app = FastAPI(
    title="R2D2 Web Dashboard",
    description="Monitoring and control dashboard for R2D2 system",
    version="1.0.0"
)

# Include API routers
app.include_router(services.router)
app.include_router(audio.router)
app.include_router(training.router)
app.include_router(status.router)
app.include_router(system.router)
app.include_router(database.router)

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
    import uvicorn
    uvicorn.run(app, host=HOST, port=PORT)
