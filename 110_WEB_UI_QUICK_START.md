# Web UI Quick Start Guide
**Date:** December 12, 2025  
**Purpose:** Quick reference for R2D2 Web UI implementation

---

## Overview

This is a quick reference guide. For complete details, see **[110_WEB_UI_ARCHITECTURE_AND_INTEGRATION.md](110_WEB_UI_ARCHITECTURE_AND_INTEGRATION.md)**.

---

## Architecture Summary

```
Internet → Tailscale VPN → Jetson (100.95.133.26:5000)
                              ↓
                    FastAPI Web Server
                    ├─ Google OAuth
                    ├─ WebSocket Chat
                    ├─ ROS 2 Bridge (Face Recognition)
                    └─ SQLite Database
```

---

## Key Components

| Component | Technology | Purpose |
|-----------|-----------|---------|
| **Web Server** | FastAPI | HTTP/WebSocket server |
| **Authentication** | Google OAuth 2.0 | User login |
| **Chat** | WebSocket + LLM API | Real-time messaging |
| **Face Recognition** | ROS 2 Bridge | Sync recognition status |
| **Database** | SQLite | Chat history storage |
| **Remote Access** | Tailscale | Secure access from anywhere |

---

## Implementation Phases

### Phase 1: Foundation (Week 1)
- ✅ Create project structure
- ✅ Set up Python environment
- ✅ Configure environment variables
- ✅ Initialize database schema

### Phase 2: Authentication (Week 1-2)
- ✅ Google OAuth 2.0 setup
- ✅ User session management
- ✅ Profile linking (face → Google account)

### Phase 3: ROS 2 Integration (Week 2)
- ✅ Subscribe to face recognition topics
- ✅ Real-time status updates
- ✅ Database logging

### Phase 4: Chat System (Week 2)
- ✅ LLM API integration
- ✅ WebSocket chat handler
- ✅ Message storage

### Phase 5: Frontend (Week 2-3)
- ✅ HTML/CSS/JavaScript UI
- ✅ WebSocket client
- ✅ Real-time updates

### Phase 6: Deployment (Week 3)
- ✅ Systemd service
- ✅ Testing & validation
- ✅ Production deployment

---

## Quick Setup Commands

```bash
# 1. Create project
cd ~/dev/r2d2
mkdir -p r2d2_web/{app/{auth,api,websocket,database,ros2_bridge,llm,static/{css,js}},templates}

# 2. Set up environment
cd r2d2_web
python3 -m venv r2d2_web_env
source r2d2_web_env/bin/activate
pip install -r requirements.txt

# 3. Configure environment
cp .env.example .env
nano .env  # Add Google OAuth credentials and API keys

# 4. Initialize database
python -c "from app.database.db import init_db; init_db()"

# 5. Test locally
uvicorn app.main:app --host 0.0.0.0 --port 5000

# 6. Create systemd service
sudo cp r2d2-web-ui.service /etc/systemd/system/
sudo systemctl enable r2d2-web-ui
sudo systemctl start r2d2-web-ui
```

---

## Access Points

- **Web UI:** http://100.95.133.26:5000 (via Tailscale)
- **API Docs:** http://100.95.133.26:5000/docs (FastAPI Swagger)
- **Database:** `~/dev/r2d2/data/r2d2_web.db`

**Note:** For the current web dashboard (port 8080), see [`111_WEB_DASHBOARD_DOCUMENTATION.md`](111_WEB_DASHBOARD_DOCUMENTATION.md). The dashboard defaults to **Recognition Status mode** after reboot (camera-perception + audio-notification services start automatically).

---

## Database Schema (Quick Reference)

```sql
users                    -- Google OAuth users
chat_sessions            -- Chat conversation sessions
chat_messages            -- Individual messages
face_recognition_events  -- Face recognition sync events
oauth_sessions           -- OAuth token storage
```

---

## ROS 2 Topics Used

- `/r2d2/perception/person_id` - Person identification
- `/r2d2/audio/person_status` - Recognition state (RED/BLUE/GREEN)
- `/r2d2/perception/face_confidence` - Confidence score

---

## Security Checklist

- [ ] `.env` file not committed to git
- [ ] OAuth tokens encrypted in database
- [ ] HTTPS enabled (Let's Encrypt)
- [ ] Rate limiting on API endpoints
- [ ] Input validation on all user inputs
- [ ] CORS restricted to Tailscale network

---

## Troubleshooting

**Web UI not accessible:**
```bash
sudo systemctl status r2d2-web-ui
tailscale status
curl http://localhost:5000
```

**Face recognition not syncing:**
```bash
ros2 topic echo /r2d2/perception/person_id
ros2 topic list
journalctl -u r2d2-web-ui -f
```

**Database issues:**
```bash
sqlite3 ~/dev/r2d2/data/r2d2_web.db ".tables"
sqlite3 ~/dev/r2d2/data/r2d2_web.db ".schema"
```

---

## Next Steps

1. Read full architecture: `110_WEB_UI_ARCHITECTURE_AND_INTEGRATION.md`
2. Set up Google Cloud project for OAuth
3. Begin Phase 1 implementation
4. Test each phase before moving to next

---

**For complete documentation, see:** `110_WEB_UI_ARCHITECTURE_AND_INTEGRATION.md`

