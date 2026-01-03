"""
Secure Database Access API Endpoints

Provides authenticated download endpoints for R2D2 SQLite databases.

Security:
- API key required (stored in ~/.r2d2/database_api_key)
- All access logged to audit log
- Only accessible via Tailscale VPN

Databases:
- persons.db: Person registry, learning progress
- conversations.db: Speech sessions and transcripts
"""

from fastapi import APIRouter, HTTPException, Header, Depends
from fastapi.responses import FileResponse
from pathlib import Path
from datetime import datetime
import sqlite3
import logging
from logging.handlers import RotatingFileHandler
import os

router = APIRouter(prefix="/api/database", tags=["database"])

# =============================================================================
# Audit Logging Setup
# =============================================================================

# Create logs directory
LOG_DIR = Path.home() / 'dev' / 'r2d2' / 'logs'
LOG_DIR.mkdir(parents=True, exist_ok=True)

# Configure audit logger with rotation
audit_logger = logging.getLogger('database_audit')
audit_logger.setLevel(logging.INFO)

# Prevent duplicate handlers
if not audit_logger.handlers:
    handler = RotatingFileHandler(
        LOG_DIR / 'database_access.log',
        maxBytes=1_000_000,  # 1 MB
        backupCount=5
    )
    handler.setFormatter(logging.Formatter(
        '%(asctime)s - %(levelname)s - %(message)s'
    ))
    audit_logger.addHandler(handler)

# =============================================================================
# API Key Authentication
# =============================================================================

def _get_api_key() -> str | None:
    """Load API key from secure location (~/.r2d2/database_api_key)"""
    key_path = Path.home() / '.r2d2' / 'database_api_key'
    if key_path.exists():
        return key_path.read_text().strip()
    return None


async def verify_api_key(x_api_key: str = Header(..., alias="X-API-Key")) -> bool:
    """
    Verify API key from request header.
    
    Requires header: X-API-Key: <your-api-key>
    
    Raises:
        HTTPException 401: If API key is missing or invalid
    """
    expected = _get_api_key()
    
    if not expected:
        audit_logger.error("API key file not found at ~/.r2d2/database_api_key")
        raise HTTPException(
            status_code=500, 
            detail="API key not configured on server"
        )
    
    if x_api_key != expected:
        audit_logger.warning(f"Invalid API key attempt at {datetime.now().isoformat()}")
        raise HTTPException(
            status_code=401, 
            detail="Invalid API key"
        )
    
    return True

# =============================================================================
# Database Paths
# =============================================================================

DATABASE_PATHS = {
    "persons": Path.home() / 'dev' / 'r2d2' / 'data' / 'persons.db',
    "conversations": Path.home() / 'dev' / 'r2d2' / 'r2d2_speech' / 'data' / 'conversations.db',
}

DATABASE_DESCRIPTIONS = {
    "persons": "Person registry with face/gesture models and learning progress",
    "conversations": "Speech conversation sessions and transcripts"
}

# =============================================================================
# Helper Functions
# =============================================================================

def _get_db_stats(db_path: Path) -> dict:
    """Get statistics for a database file"""
    if not db_path.exists():
        return {"exists": False}
    
    stats = {
        "exists": True,
        "size_bytes": db_path.stat().st_size,
        "size_human": _human_size(db_path.stat().st_size),
        "last_modified": datetime.fromtimestamp(
            db_path.stat().st_mtime
        ).isoformat(),
        "tables": []
    }
    
    try:
        conn = sqlite3.connect(str(db_path))
        cursor = conn.cursor()
        
        # Get table names and row counts
        cursor.execute("SELECT name FROM sqlite_master WHERE type='table'")
        tables = cursor.fetchall()
        
        for (table_name,) in tables:
            if table_name.startswith('sqlite_'):
                continue
            cursor.execute(f"SELECT COUNT(*) FROM {table_name}")
            count = cursor.fetchone()[0]
            stats["tables"].append({
                "name": table_name,
                "row_count": count
            })
        
        conn.close()
    except Exception as e:
        stats["error"] = str(e)
    
    return stats


def _human_size(size_bytes: int) -> str:
    """Convert bytes to human readable string"""
    for unit in ['B', 'KB', 'MB', 'GB']:
        if size_bytes < 1024:
            return f"{size_bytes:.1f} {unit}"
        size_bytes /= 1024
    return f"{size_bytes:.1f} TB"

# =============================================================================
# API Endpoints
# =============================================================================

@router.get("/info")
async def get_database_info(authorized: bool = Depends(verify_api_key)):
    """
    Get information about available databases.
    
    Returns file sizes, table counts, and last modified times.
    Requires API key authentication.
    """
    audit_logger.info(f"Database info requested at {datetime.now().isoformat()}")
    
    result = {}
    for name, path in DATABASE_PATHS.items():
        result[name] = {
            "description": DATABASE_DESCRIPTIONS.get(name, ""),
            "path": str(path),
            **_get_db_stats(path)
        }
    
    return result


@router.get("/download/{db_name}")
async def download_database(
    db_name: str,
    authorized: bool = Depends(verify_api_key)
):
    """
    Download a database file.
    
    Args:
        db_name: Database name ('persons' or 'conversations')
    
    Returns:
        Database file as attachment
        
    Requires API key authentication.
    """
    if db_name not in DATABASE_PATHS:
        raise HTTPException(
            status_code=404, 
            detail=f"Unknown database: {db_name}. Available: {list(DATABASE_PATHS.keys())}"
        )
    
    db_path = DATABASE_PATHS[db_name]
    
    if not db_path.exists():
        audit_logger.warning(f"Database not found: {db_name} at {db_path}")
        raise HTTPException(
            status_code=404, 
            detail=f"Database file not found: {db_path}"
        )
    
    # Log successful download
    audit_logger.info(
        f"DATABASE DOWNLOAD: {db_name}.db "
        f"({_human_size(db_path.stat().st_size)}) "
        f"at {datetime.now().isoformat()}"
    )
    
    return FileResponse(
        path=str(db_path),
        filename=f"{db_name}.db",
        media_type="application/x-sqlite3"
    )


@router.get("/schema/{db_name}")
async def get_database_schema(
    db_name: str,
    authorized: bool = Depends(verify_api_key)
):
    """
    Get schema (table definitions) for a database.
    
    Returns CREATE TABLE statements for all tables.
    Requires API key authentication.
    """
    if db_name not in DATABASE_PATHS:
        raise HTTPException(
            status_code=404,
            detail=f"Unknown database: {db_name}"
        )
    
    db_path = DATABASE_PATHS[db_name]
    
    if not db_path.exists():
        raise HTTPException(
            status_code=404,
            detail=f"Database file not found"
        )
    
    audit_logger.info(f"Schema requested for {db_name} at {datetime.now().isoformat()}")
    
    try:
        conn = sqlite3.connect(str(db_path))
        cursor = conn.cursor()
        
        # Get all table schemas
        cursor.execute("""
            SELECT name, sql FROM sqlite_master 
            WHERE type='table' AND name NOT LIKE 'sqlite_%'
        """)
        
        tables = {}
        for name, sql in cursor.fetchall():
            tables[name] = sql
        
        conn.close()
        
        return {
            "database": db_name,
            "tables": tables
        }
        
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))


@router.get("/query-examples")
async def get_query_examples():
    """
    Get example SQL queries for each database.
    
    Public endpoint (no auth required) - just returns example queries.
    """
    return {
        "persons_db": {
            "description": "Person registry database",
            "examples": [
                {
                    "name": "List all persons",
                    "sql": "SELECT display_name, face_model_path, gesture_model_path FROM persons;"
                },
                {
                    "name": "Learning progress by category",
                    "sql": """SELECT category, COUNT(*) as topics, 
       ROUND(AVG(understanding_level), 1) as avg_level
FROM learning_topics 
GROUP BY category 
ORDER BY topics DESC;"""
                },
                {
                    "name": "Topics needing review (level < 3)",
                    "sql": """SELECT topic, category, understanding_level, last_reviewed
FROM learning_topics 
WHERE understanding_level < 3 
ORDER BY understanding_level;"""
                },
                {
                    "name": "Recent learning sessions",
                    "sql": """SELECT started_at, ended_at, summary 
FROM learning_sessions 
ORDER BY started_at DESC 
LIMIT 10;"""
                }
            ]
        },
        "conversations_db": {
            "description": "Conversation history database",
            "examples": [
                {
                    "name": "Recent conversations with transcripts",
                    "sql": """SELECT s.started_at, m.role, m.text
FROM sessions s 
JOIN messages m ON s.session_id = m.session_id
ORDER BY s.started_at DESC, m.created_at ASC
LIMIT 100;"""
                },
                {
                    "name": "Conversation statistics by day",
                    "sql": """SELECT DATE(started_at) as day, COUNT(*) as sessions
FROM sessions 
GROUP BY DATE(started_at)
ORDER BY day DESC;"""
                },
                {
                    "name": "Message count by role",
                    "sql": """SELECT role, COUNT(*) as count 
FROM messages 
GROUP BY role;"""
                },
                {
                    "name": "Longest conversations",
                    "sql": """SELECT s.session_id, s.started_at, COUNT(m.id) as message_count
FROM sessions s 
JOIN messages m ON s.session_id = m.session_id
GROUP BY s.session_id
ORDER BY message_count DESC
LIMIT 10;"""
                }
            ]
        }
    }


