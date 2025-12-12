# Web UI Architecture & Integration Plan
**Date:** December 12, 2025  
**Platform:** NVIDIA Jetson AGX Orin 64GB + ROS 2 Humble  
**Purpose:** Secure web-based UI accessible from anywhere with Google authentication, chat, and face recognition sync

---

## Executive Summary

This document outlines the architecture and step-by-step integration plan for a web-based user interface that:
- ✅ **Accessible from anywhere** via Tailscale VPN (already configured)
- ✅ **Google OAuth authentication** for secure login
- ✅ **Real-time chat** with LLM integration (using shared API keys)
- ✅ **Face recognition sync** - links "severin" profile to Google account
- ✅ **SQLite database** for persistent chat storage on Jetson
- ✅ **ROS 2 integration** - subscribes to face recognition topics
- ✅ **Profile synchronization** - face detection status synced to Google account

**Estimated Implementation Time:** 2-3 weeks  
**Technology Stack:** Flask/FastAPI, Google OAuth 2.0, SQLite, ROS 2 bridge, WebSockets

---

## 1. System Architecture Overview

### 1.1 High-Level Architecture

```
┌─────────────────────────────────────────────────────────────────────┐
│                    WEB UI SYSTEM ARCHITECTURE                      │
└─────────────────────────────────────────────────────────────────────┘

INTERNET (Anywhere)
    ↓
Tailscale VPN (Already Configured)
    ↓ (100.95.133.26)
┌─────────────────────────────────────────────────────────────────────┐
│                    JETSON AGX ORIN                                 │
│                                                                     │
│  ┌─────────────────────────────────────────────────────────────┐  │
│  │  Web Server (Flask/FastAPI)                                 │  │
│  │  Port: 5000 (HTTP) / 5001 (HTTPS)                          │  │
│  │  ├─ Google OAuth 2.0 Authentication                        │  │
│  │  ├─ REST API Endpoints                                      │  │
│  │  ├─ WebSocket Server (Real-time chat)                      │  │
│  │  └─ Static File Serving (HTML/CSS/JS)                      │  │
│  └─────────────────────────────────────────────────────────────┘  │
│           │                    │                    │              │
│           ↓                    ↓                    ↓              │
│  ┌─────────────────┐  ┌─────────────────┐  ┌─────────────────┐  │
│  │  SQLite DB      │  │  ROS 2 Bridge   │  │  LLM Service    │  │
│  │  (Chat History) │  │  (Face Recog)    │  │  (API Keys)      │  │
│  └─────────────────┘  └─────────────────┘  └─────────────────┘  │
│           │                    │                    │              │
│           └────────────────────┴────────────────────┘              │
│                              │                                      │
│                              ↓                                      │
│  ┌─────────────────────────────────────────────────────────────┐  │
│  │  ROS 2 System (Existing)                                    │  │
│  │  ├─ /r2d2/perception/person_id (String)                      │  │
│  │  ├─ /r2d2/audio/person_status (JSON)                        │  │
│  │  └─ /r2d2/perception/face_confidence (Float32)              │  │
│  └─────────────────────────────────────────────────────────────┘  │
│                                                                     │
└─────────────────────────────────────────────────────────────────────┘
```

### 1.2 Component Breakdown

| Component | Technology | Purpose | Status |
|-----------|-----------|---------|--------|
| **Web Server** | Flask/FastAPI | HTTP/WebSocket server | ⏳ To implement |
| **Frontend** | HTML/CSS/JavaScript | User interface | ⏳ To implement |
| **OAuth** | Google OAuth 2.0 | Authentication | ⏳ To implement |
| **Database** | SQLite | Chat storage | ⏳ To implement |
| **ROS Bridge** | rclpy | Face recognition integration | ⏳ To implement |
| **LLM Service** | OpenAI/Grok API | Chat responses | ⏳ To implement |
| **VPN** | Tailscale | Remote access | ✅ Already configured |

---

## 2. Detailed Architecture Design

### 2.1 Web Server Architecture

**Framework Choice: FastAPI** (Recommended over Flask)
- ✅ Modern async/await support (better for WebSockets)
- ✅ Automatic API documentation (Swagger UI)
- ✅ Type hints and validation
- ✅ Better performance for concurrent connections
- ✅ Native WebSocket support

**Server Structure:**
```
~/dev/r2d2/r2d2_web/
├── app/
│   ├── __init__.py
│   ├── main.py                 # FastAPI app entry point
│   ├── config.py               # Configuration (OAuth, DB, etc.)
│   ├── auth/
│   │   ├── __init__.py
│   │   ├── google_oauth.py     # Google OAuth 2.0 handler
│   │   └── session.py           # Session management
│   ├── api/
│   │   ├── __init__.py
│   │   ├── chat.py              # Chat API endpoints
│   │   ├── profile.py           # Profile management
│   │   └── status.py             # Face recognition status
│   ├── websocket/
│   │   ├── __init__.py
│   │   └── chat_handler.py      # WebSocket chat handler
│   ├── database/
│   │   ├── __init__.py
│   │   ├── models.py             # SQLAlchemy models
│   │   ├── schema.py             # Database schema
│   │   └── db.py                 # Database connection
│   ├── ros2_bridge/
│   │   ├── __init__.py
│   │   ├── face_recognition.py   # ROS 2 subscriber for face recog
│   │   └── status_monitor.py     # Monitor person_status topic
│   ├── llm/
│   │   ├── __init__.py
│   │   ├── client.py              # LLM API client (OpenAI/Grok)
│   │   └── chat_processor.py     # Chat message processing
│   └── static/
│       ├── css/
│       ├── js/
│       └── images/
├── templates/
│   ├── index.html                # Main chat interface
│   ├── login.html                # Google OAuth login
│   └── profile.html              # Profile management
├── requirements.txt
├── .env                          # Environment variables (API keys)
└── run.py                        # Application entry point
```

### 2.2 Database Schema

**SQLite Database: `~/dev/r2d2/data/r2d2_web.db`**

```sql
-- Users table (linked to Google accounts)
CREATE TABLE users (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    google_id TEXT UNIQUE NOT NULL,          -- Google user ID
    email TEXT UNIQUE NOT NULL,               -- Google email
    name TEXT,                                -- Display name
    picture_url TEXT,                         -- Profile picture
    face_profile_name TEXT,                   -- Linked face profile (e.g., "severin")
    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    updated_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
);

-- Chat sessions
CREATE TABLE chat_sessions (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    user_id INTEGER NOT NULL,
    title TEXT,                               -- Auto-generated or user-set
    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    updated_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    FOREIGN KEY (user_id) REFERENCES users(id) ON DELETE CASCADE
);

-- Chat messages
CREATE TABLE chat_messages (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    session_id INTEGER NOT NULL,
    role TEXT NOT NULL,                       -- 'user' or 'assistant'
    content TEXT NOT NULL,                    -- Message text
    face_recognized BOOLEAN DEFAULT 0,        -- Was face recognized when sent?
    recognition_confidence REAL,               -- Confidence score if recognized
    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    FOREIGN KEY (session_id) REFERENCES chat_sessions(id) ON DELETE CASCADE
);

-- Face recognition events (synced from ROS 2)
CREATE TABLE face_recognition_events (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    user_id INTEGER,                          -- NULL if unknown person
    face_profile_name TEXT,                   -- "severin" or "unknown"
    confidence REAL,                           -- Recognition confidence
    status TEXT,                               -- "red", "blue", "green"
    recognized_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    FOREIGN KEY (user_id) REFERENCES users(id) ON DELETE SET NULL
);

-- OAuth sessions (for Google authentication)
CREATE TABLE oauth_sessions (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    session_id TEXT UNIQUE NOT NULL,          -- Flask session ID
    user_id INTEGER NOT NULL,
    access_token TEXT,                         -- Encrypted OAuth token
    refresh_token TEXT,                        -- Encrypted refresh token
    expires_at TIMESTAMP,
    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    FOREIGN KEY (user_id) REFERENCES users(id) ON DELETE CASCADE
);

-- Indexes for performance
CREATE INDEX idx_chat_sessions_user ON chat_sessions(user_id);
CREATE INDEX idx_chat_messages_session ON chat_messages(session_id);
CREATE INDEX idx_face_events_user ON face_recognition_events(user_id);
CREATE INDEX idx_face_events_profile ON face_recognition_events(face_profile_name);
```

### 2.3 Google OAuth 2.0 Flow

```
1. User clicks "Login with Google" on web UI
   ↓
2. Web server redirects to Google OAuth consent screen
   ↓
3. User authorizes application
   ↓
4. Google redirects back with authorization code
   ↓
5. Web server exchanges code for access token
   ↓
6. Web server fetches user profile (email, name, picture)
   ↓
7. Web server creates/updates user in SQLite database
   ↓
8. Web server creates session and links to user
   ↓
9. User is logged in and can access chat interface
```

**OAuth Scopes Required:**
- `openid` - Basic OpenID Connect
- `profile` - User profile information
- `email` - User email address

### 2.4 Face Recognition Integration

**ROS 2 Topics to Subscribe:**
- `/r2d2/perception/person_id` (std_msgs/String) - "severin" or "unknown"
- `/r2d2/audio/person_status` (std_msgs/String) - JSON status (RED/BLUE/GREEN)
- `/r2d2/perception/face_confidence` (std_msgs/Float32) - Confidence score

**Integration Flow:**
```
ROS 2 Topic: /r2d2/perception/person_id
    ↓
ROS 2 Bridge (rclpy subscriber)
    ↓
Web Server (FastAPI background task)
    ↓
1. Check if person_id == "severin"
2. Look up user with face_profile_name == "severin"
3. Update user's recognition status in database
4. Emit WebSocket event to connected clients
5. Log event to face_recognition_events table
```

**Profile Linking:**
- When user logs in with Google OAuth, they can link their account to a face profile
- If face_profile_name matches (e.g., "severin"), system automatically links
- Face recognition events are then associated with the user's Google account

### 2.5 Chat System Architecture

**Chat Flow:**
```
User sends message via WebSocket
    ↓
Web Server receives message
    ↓
1. Check if user's face is currently recognized
2. Store message in database (with recognition status)
3. Send message to LLM API (OpenAI/Grok)
4. Receive LLM response
5. Store response in database
6. Send response back to user via WebSocket
```

**LLM Integration:**
- Use shared API keys (stored in `.env` file)
- Support multiple providers: OpenAI, Grok (X.AI), Anthropic
- Include context: user profile, recent chat history, face recognition status
- Rate limiting and error handling

**WebSocket Protocol:**
```json
// Client → Server
{
  "type": "message",
  "content": "Hello R2D2!",
  "session_id": 123
}

// Server → Client
{
  "type": "message",
  "role": "assistant",
  "content": "Hello! I recognize you, Severin!",
  "face_recognized": true,
  "confidence": 0.95
}

// Server → Client (Face recognition update)
{
  "type": "face_recognition",
  "status": "red",
  "person": "severin",
  "confidence": 0.92
}
```

---

## 3. Step-by-Step Integration Plan

### Phase 1: Foundation Setup (Week 1)

#### Step 1.1: Create Project Structure
```bash
cd ~/dev/r2d2
mkdir -p r2d2_web/{app/{auth,api,websocket,database,ros2_bridge,llm,static/{css,js,images}},templates}
cd r2d2_web
```

#### Step 1.2: Set Up Python Environment
```bash
# Create virtual environment
python3 -m venv r2d2_web_env
source r2d2_web_env/bin/activate

# Install dependencies
pip install fastapi uvicorn[standard] python-multipart
pip install google-auth google-auth-oauthlib google-auth-httplib2
pip install sqlalchemy aiosqlite
pip install rclpy std_msgs sensor_msgs
pip install openai anthropic httpx websockets
pip install python-dotenv python-jose[cryptography] passlib[bcrypt]
```

#### Step 1.3: Create requirements.txt
```bash
cat > requirements.txt << 'EOF'
fastapi==0.104.1
uvicorn[standard]==0.24.0
python-multipart==0.0.6
google-auth==2.23.4
google-auth-oauthlib==1.1.0
google-auth-httplib2==0.1.1
sqlalchemy==2.0.23
aiosqlite==0.19.0
rclpy==3.3.4
std_msgs==4.2.3
sensor_msgs==4.2.3
openai==1.3.5
anthropic==0.7.7
httpx==0.25.1
websockets==12.0
python-dotenv==1.0.0
python-jose[cryptography]==3.3.0
passlib[bcrypt]==1.7.4
EOF
```

#### Step 1.4: Set Up Environment Variables
```bash
cat > .env << 'EOF'
# Google OAuth
GOOGLE_CLIENT_ID=your_client_id_here
GOOGLE_CLIENT_SECRET=your_client_secret_here
GOOGLE_REDIRECT_URI=http://100.95.133.26:5000/auth/callback

# LLM API Keys
OPENAI_API_KEY=your_openai_key_here
GROK_API_KEY=your_grok_key_here
ANTHROPIC_API_KEY=your_anthropic_key_here

# Database
DATABASE_PATH=/home/severin/dev/r2d2/data/r2d2_web.db

# Server
HOST=0.0.0.0
PORT=5000
SECRET_KEY=your_secret_key_here_generate_random

# ROS 2
ROS_DOMAIN_ID=0
EOF
```

**⚠️ Security:** Never commit `.env` file to git! Add to `.gitignore`.

### Phase 2: Database Implementation (Week 1)

#### Step 2.1: Create Database Models
```python
# app/database/models.py
from sqlalchemy import Column, Integer, String, Text, Boolean, Float, ForeignKey, DateTime
from sqlalchemy.ext.declarative import declarative_base
from sqlalchemy.orm import relationship
from datetime import datetime

Base = declarative_base()

class User(Base):
    __tablename__ = 'users'
    
    id = Column(Integer, primary_key=True)
    google_id = Column(String, unique=True, nullable=False)
    email = Column(String, unique=True, nullable=False)
    name = Column(String)
    picture_url = Column(String)
    face_profile_name = Column(String)  # "severin" or NULL
    created_at = Column(DateTime, default=datetime.utcnow)
    updated_at = Column(DateTime, default=datetime.utcnow, onupdate=datetime.utcnow)
    
    # Relationships
    chat_sessions = relationship("ChatSession", back_populates="user")
    face_events = relationship("FaceRecognitionEvent", back_populates="user")

class ChatSession(Base):
    __tablename__ = 'chat_sessions'
    
    id = Column(Integer, primary_key=True)
    user_id = Column(Integer, ForeignKey('users.id'), nullable=False)
    title = Column(String)
    created_at = Column(DateTime, default=datetime.utcnow)
    updated_at = Column(DateTime, default=datetime.utcnow, onupdate=datetime.utcnow)
    
    # Relationships
    user = relationship("User", back_populates="chat_sessions")
    messages = relationship("ChatMessage", back_populates="session")

class ChatMessage(Base):
    __tablename__ = 'chat_messages'
    
    id = Column(Integer, primary_key=True)
    session_id = Column(Integer, ForeignKey('chat_sessions.id'), nullable=False)
    role = Column(String, nullable=False)  # 'user' or 'assistant'
    content = Column(Text, nullable=False)
    face_recognized = Column(Boolean, default=False)
    recognition_confidence = Column(Float)
    created_at = Column(DateTime, default=datetime.utcnow)
    
    # Relationships
    session = relationship("ChatSession", back_populates="messages")

class FaceRecognitionEvent(Base):
    __tablename__ = 'face_recognition_events'
    
    id = Column(Integer, primary_key=True)
    user_id = Column(Integer, ForeignKey('users.id'), nullable=True)
    face_profile_name = Column(String)  # "severin" or "unknown"
    confidence = Column(Float)
    status = Column(String)  # "red", "blue", "green"
    recognized_at = Column(DateTime, default=datetime.utcnow)
    
    # Relationships
    user = relationship("User", back_populates="face_events")
```

#### Step 2.2: Database Initialization
```python
# app/database/db.py
from sqlalchemy import create_engine
from sqlalchemy.orm import sessionmaker
from app.database.models import Base
import os
from dotenv import load_dotenv

load_dotenv()

DATABASE_PATH = os.getenv('DATABASE_PATH', '/home/severin/dev/r2d2/data/r2d2_web.db')

# Ensure directory exists
os.makedirs(os.path.dirname(DATABASE_PATH), exist_ok=True)

engine = create_engine(f'sqlite:///{DATABASE_PATH}', echo=False)
SessionLocal = sessionmaker(autocommit=False, autoflush=False, bind=engine)

def init_db():
    """Create all database tables"""
    Base.metadata.create_all(bind=engine)

def get_db():
    """Dependency for FastAPI to get database session"""
    db = SessionLocal()
    try:
        yield db
    finally:
        db.close()
```

### Phase 3: Google OAuth Implementation (Week 1-2)

#### Step 3.1: Set Up Google Cloud Project
1. Go to https://console.cloud.google.com/
2. Create new project: "R2D2 Web UI"
3. Enable Google+ API
4. Create OAuth 2.0 credentials:
   - Application type: Web application
   - Authorized redirect URIs: `http://100.95.133.26:5000/auth/callback`
   - Copy Client ID and Client Secret to `.env`

#### Step 3.2: Implement OAuth Handler
```python
# app/auth/google_oauth.py
from google.oauth2 import id_token
from google.auth.transport import requests
from google_auth_oauthlib.flow import Flow
import os
from dotenv import load_dotenv

load_dotenv()

CLIENT_ID = os.getenv('GOOGLE_CLIENT_ID')
CLIENT_SECRET = os.getenv('GOOGLE_CLIENT_SECRET')
REDIRECT_URI = os.getenv('GOOGLE_REDIRECT_URI')

SCOPES = ['openid', 'profile', 'email']

def get_oauth_flow():
    """Create OAuth flow for Google authentication"""
    return Flow.from_client_config(
        {
            "web": {
                "client_id": CLIENT_ID,
                "client_secret": CLIENT_SECRET,
                "auth_uri": "https://accounts.google.com/o/oauth2/auth",
                "token_uri": "https://oauth2.googleapis.com/token",
                "redirect_uris": [REDIRECT_URI]
            }
        },
        scopes=SCOPES,
        redirect_uri=REDIRECT_URI
    )

def verify_token(token):
    """Verify Google ID token and return user info"""
    try:
        idinfo = id_token.verify_oauth2_token(
            token, requests.Request(), CLIENT_ID
        )
        return {
            'google_id': idinfo['sub'],
            'email': idinfo['email'],
            'name': idinfo.get('name'),
            'picture': idinfo.get('picture')
        }
    except ValueError:
        return None
```

### Phase 4: ROS 2 Bridge Implementation (Week 2)

#### Step 4.1: Create ROS 2 Subscriber
```python
# app/ros2_bridge/face_recognition.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32
import json
from typing import Optional, Callable

class FaceRecognitionBridge(Node):
    """ROS 2 bridge for face recognition topics"""
    
    def __init__(self, callback: Optional[Callable] = None):
        super().__init__('web_ui_face_recognition_bridge')
        
        self.callback = callback
        self.current_person = None
        self.current_confidence = None
        self.current_status = None
        
        # Subscribe to face recognition topics
        self.person_id_sub = self.create_subscription(
            String,
            '/r2d2/perception/person_id',
            self.person_id_callback,
            10
        )
        
        self.confidence_sub = self.create_subscription(
            Float32,
            '/r2d2/perception/face_confidence',
            self.confidence_callback,
            10
        )
        
        self.status_sub = self.create_subscription(
            String,
            '/r2d2/audio/person_status',
            self.status_callback,
            10
        )
        
        self.get_logger().info('Face Recognition Bridge initialized')
    
    def person_id_callback(self, msg: String):
        """Handle person_id topic updates"""
        self.current_person = msg.data
        self._notify_callback()
    
    def confidence_callback(self, msg: Float32):
        """Handle confidence topic updates"""
        self.current_confidence = msg.data
        self._notify_callback()
    
    def status_callback(self, msg: String):
        """Handle person_status topic updates"""
        try:
            status_data = json.loads(msg.data)
            self.current_status = status_data.get('status')  # "red", "blue", "green"
            self._notify_callback()
        except json.JSONDecodeError:
            self.get_logger().warn('Failed to parse person_status JSON')
    
    def _notify_callback(self):
        """Notify callback with current state"""
        if self.callback:
            self.callback({
                'person': self.current_person,
                'confidence': self.current_confidence,
                'status': self.current_status
            })
    
    def get_current_state(self):
        """Get current face recognition state"""
        return {
            'person': self.current_person,
            'confidence': self.current_confidence,
            'status': self.current_status
        }
```

#### Step 4.2: Integrate with FastAPI
```python
# In app/main.py
from app.ros2_bridge.face_recognition import FaceRecognitionBridge
import threading

# Initialize ROS 2 in background thread
def init_ros2_bridge():
    rclpy.init()
    bridge = FaceRecognitionBridge(callback=on_face_recognition_update)
    
    def spin_ros2():
        rclpy.spin(bridge)
    
    thread = threading.Thread(target=spin_ros2, daemon=True)
    thread.start()
    return bridge

# Callback when face recognition updates
def on_face_recognition_update(state):
    # Store in database
    # Emit WebSocket event to connected clients
    pass
```

### Phase 5: Chat System Implementation (Week 2)

#### Step 5.1: LLM Client
```python
# app/llm/client.py
import openai
import os
from dotenv import load_dotenv

load_dotenv()

class LLMClient:
    def __init__(self):
        self.openai_key = os.getenv('OPENAI_API_KEY')
        self.grok_key = os.getenv('GROK_API_KEY')
        self.provider = 'openai'  # or 'grok'
    
    async def chat(self, messages: list, user_context: dict = None):
        """Send chat messages to LLM"""
        if self.provider == 'openai':
            return await self._chat_openai(messages, user_context)
        elif self.provider == 'grok':
            return await self._chat_grok(messages, user_context)
    
    async def _chat_openai(self, messages: list, user_context: dict):
        client = openai.AsyncOpenAI(api_key=self.openai_key)
        
        # Add context if face recognized
        system_message = "You are R2D2, a helpful AI companion."
        if user_context and user_context.get('face_recognized'):
            system_message += f" You recognize {user_context.get('name')}."
        
        response = await client.chat.completions.create(
            model="gpt-4",
            messages=[{"role": "system", "content": system_message}] + messages
        )
        
        return response.choices[0].message.content
```

#### Step 5.2: WebSocket Chat Handler
```python
# app/websocket/chat_handler.py
from fastapi import WebSocket, WebSocketDisconnect
from app.database.db import get_db
from app.llm.client import LLMClient
import json

class ChatManager:
    def __init__(self):
        self.active_connections: dict = {}  # user_id -> WebSocket
        self.llm_client = LLMClient()
    
    async def connect(self, websocket: WebSocket, user_id: int):
        await websocket.accept()
        self.active_connections[user_id] = websocket
    
    def disconnect(self, user_id: int):
        if user_id in self.active_connections:
            del self.active_connections[user_id]
    
    async def send_message(self, user_id: int, message: dict):
        if user_id in self.active_connections:
            await self.active_connections[user_id].send_json(message)
    
    async def handle_message(self, websocket: WebSocket, user_id: int, data: dict):
        """Handle incoming chat message"""
        # Get current face recognition status
        face_status = get_current_face_status(user_id)
        
        # Store user message in database
        # ... database code ...
        
        # Send to LLM
        response = await self.llm_client.chat(
            messages=[{"role": "user", "content": data['content']}],
            user_context={'face_recognized': face_status['recognized']}
        )
        
        # Store LLM response in database
        # ... database code ...
        
        # Send response to user
        await websocket.send_json({
            "type": "message",
            "role": "assistant",
            "content": response,
            "face_recognized": face_status['recognized']
        })
```

### Phase 6: Frontend Implementation (Week 2-3)

#### Step 6.1: Create HTML Templates
```html
<!-- templates/index.html -->
<!DOCTYPE html>
<html>
<head>
    <title>R2D2 Chat</title>
    <link rel="stylesheet" href="/static/css/style.css">
</head>
<body>
    <div class="chat-container">
        <div class="header">
            <h1>R2D2 Chat</h1>
            <div class="face-status" id="faceStatus">
                <span id="statusText">Status: Unknown</span>
            </div>
        </div>
        <div class="chat-messages" id="chatMessages"></div>
        <div class="chat-input">
            <input type="text" id="messageInput" placeholder="Type a message...">
            <button id="sendButton">Send</button>
        </div>
    </div>
    <script src="/static/js/chat.js"></script>
</body>
</html>
```

#### Step 6.2: JavaScript WebSocket Client
```javascript
// static/js/chat.js
const ws = new WebSocket('ws://100.95.133.26:5000/ws/chat');

ws.onmessage = (event) => {
    const data = JSON.parse(event.data);
    
    if (data.type === 'message') {
        addMessage(data.role, data.content, data.face_recognized);
    } else if (data.type === 'face_recognition') {
        updateFaceStatus(data.status, data.person, data.confidence);
    }
};

function addMessage(role, content, faceRecognized) {
    const messagesDiv = document.getElementById('chatMessages');
    const messageDiv = document.createElement('div');
    messageDiv.className = `message ${role}`;
    messageDiv.innerHTML = `
        <div class="content">${content}</div>
        ${faceRecognized ? '<span class="face-badge">Recognized</span>' : ''}
    `;
    messagesDiv.appendChild(messageDiv);
}

function updateFaceStatus(status, person, confidence) {
    const statusText = document.getElementById('statusText');
    statusText.textContent = `Status: ${status.toUpperCase()} - ${person} (${confidence})`;
}
```

### Phase 7: Integration & Testing (Week 3)

#### Step 7.1: Create Main Application
```python
# app/main.py
from fastapi import FastAPI, WebSocket, Depends
from fastapi.staticfiles import StaticFiles
from fastapi.templating import Jinja2Templates
from app.database.db import init_db, get_db
from app.ros2_bridge.face_recognition import init_ros2_bridge
import rclpy

app = FastAPI(title="R2D2 Web UI")

# Mount static files
app.mount("/static", StaticFiles(directory="app/static"), name="static")

# Templates
templates = Jinja2Templates(directory="templates")

# Initialize database
init_db()

# Initialize ROS 2 bridge (in background)
ros2_bridge = None

@app.on_event("startup")
async def startup():
    global ros2_bridge
    ros2_bridge = init_ros2_bridge()

@app.on_event("shutdown")
async def shutdown():
    rclpy.shutdown()

@app.get("/")
async def index():
    return templates.TemplateResponse("index.html", {"request": {}})

@app.websocket("/ws/chat")
async def websocket_endpoint(websocket: WebSocket):
    # Handle WebSocket connection
    pass
```

#### Step 7.2: Create Systemd Service
```bash
# /etc/systemd/system/r2d2-web-ui.service
[Unit]
Description=R2D2 Web UI Service
After=network.target

[Service]
Type=simple
User=severin
WorkingDirectory=/home/severin/dev/r2d2/r2d2_web
Environment="PATH=/home/severin/dev/r2d2/r2d2_web/r2d2_web_env/bin"
ExecStart=/home/severin/dev/r2d2/r2d2_web/r2d2_web_env/bin/uvicorn app.main:app --host 0.0.0.0 --port 5000
Restart=always
RestartSec=10

[Install]
WantedBy=multi-user.target
```

---

## 4. Security Considerations

### 4.1 Authentication Security
- ✅ Use HTTPS in production (Let's Encrypt certificate)
- ✅ Store OAuth tokens encrypted in database
- ✅ Implement session timeout (30 minutes)
- ✅ Validate all user inputs
- ✅ Use secure cookies (HttpOnly, Secure, SameSite)

### 4.2 API Security
- ✅ Rate limiting on chat endpoints
- ✅ Input sanitization for LLM prompts
- ✅ API key rotation capability
- ✅ CORS configuration (restrict to Tailscale network)

### 4.3 Database Security
- ✅ SQL injection prevention (SQLAlchemy ORM)
- ✅ Encrypt sensitive data (OAuth tokens)
- ✅ Regular backups of SQLite database
- ✅ Access control (only authenticated users)

---

## 5. Impact on Existing Architecture

### 5.1 ROS 2 System (No Changes Required)
- ✅ Web UI subscribes to existing topics
- ✅ No modifications to existing nodes
- ✅ Non-intrusive integration

### 5.2 New Components Added
- ✅ `r2d2_web` package (separate from ROS 2 workspace)
- ✅ SQLite database for chat storage
- ✅ ROS 2 bridge node (lightweight subscriber)

### 5.3 Resource Usage
- **CPU:** +5-10% (web server + ROS bridge)
- **Memory:** +200-300 MB (FastAPI + SQLite)
- **Network:** Minimal (Tailscale already configured)

### 5.4 Integration Points
```
Existing ROS 2 Topics (Read-Only):
├─ /r2d2/perception/person_id → Web UI (face recognition status)
├─ /r2d2/audio/person_status → Web UI (state machine status)
└─ /r2d2/perception/face_confidence → Web UI (confidence score)

New Web UI Components:
├─ HTTP Server (Port 5000) → Accessible via Tailscale
├─ WebSocket Server → Real-time chat
└─ SQLite Database → Chat history storage
```

---

## 6. Deployment Checklist

### 6.1 Pre-Deployment
- [ ] Google Cloud project created
- [ ] OAuth credentials configured
- [ ] API keys added to `.env`
- [ ] Database schema created
- [ ] ROS 2 topics verified (face recognition running)

### 6.2 Deployment Steps
- [ ] Install dependencies: `pip install -r requirements.txt`
- [ ] Initialize database: `python -c "from app.database.db import init_db; init_db()"`
- [ ] Test OAuth flow locally
- [ ] Test ROS 2 bridge connection
- [ ] Test WebSocket chat
- [ ] Create systemd service
- [ ] Enable and start service: `sudo systemctl enable r2d2-web-ui`

### 6.3 Post-Deployment
- [ ] Verify web UI accessible via Tailscale IP
- [ ] Test Google login
- [ ] Test chat functionality
- [ ] Verify face recognition sync
- [ ] Monitor logs: `journalctl -u r2d2-web-ui -f`

---

## 7. Future Enhancements

### 7.1 Short Term
- [ ] HTTPS with Let's Encrypt
- [ ] Chat history search
- [ ] Multiple chat sessions
- [ ] Profile picture upload

### 7.2 Medium Term
- [ ] Voice chat (WebRTC)
- [ ] Mobile app (React Native)
- [ ] Multi-user support
- [ ] Chat export (PDF/JSON)

### 7.3 Long Term
- [ ] Real-time video streaming
- [ ] Advanced analytics dashboard
- [ ] Integration with Phase 2 speech system
- [ ] Cloud backup of chat history

---

## 8. Troubleshooting Guide

### Problem: Web UI not accessible
**Solution:**
- Check Tailscale status: `tailscale status`
- Verify service running: `sudo systemctl status r2d2-web-ui`
- Check firewall: `sudo ufw allow 5000/tcp`
- Test locally: `curl http://localhost:5000`

### Problem: Google OAuth not working
**Solution:**
- Verify redirect URI matches Google Cloud Console
- Check `.env` file has correct credentials
- Review OAuth logs in application logs

### Problem: Face recognition not syncing
**Solution:**
- Verify ROS 2 topics publishing: `ros2 topic list`
- Check ROS 2 bridge logs
- Ensure face recognition enabled: `ros2 launch r2d2_bringup r2d2_camera_perception.launch.py enable_face_recognition:=true`

### Problem: Chat messages not saving
**Solution:**
- Check database permissions: `ls -la ~/dev/r2d2/data/r2d2_web.db`
- Verify database schema: `sqlite3 ~/dev/r2d2/data/r2d2_web.db ".schema"`
- Check application logs for database errors

---

## 9. Quick Reference

### Essential Commands
```bash
# Start web UI
sudo systemctl start r2d2-web-ui

# Stop web UI
sudo systemctl stop r2d2-web-ui

# View logs
journalctl -u r2d2-web-ui -f

# Access web UI
# From browser: http://100.95.133.26:5000 (via Tailscale)

# Database management
sqlite3 ~/dev/r2d2/data/r2d2_web.db
```

### File Locations
| File | Location |
|------|----------|
| **Web App** | `~/dev/r2d2/r2d2_web/` |
| **Database** | `~/dev/r2d2/data/r2d2_web.db` |
| **Service** | `/etc/systemd/system/r2d2-web-ui.service` |
| **Logs** | `journalctl -u r2d2-web-ui` |

---

## 10. Success Criteria

✅ **Web UI accessible from anywhere** via Tailscale  
✅ **Google OAuth login working**  
✅ **Chat functionality operational** with LLM integration  
✅ **Face recognition syncing** to Google account  
✅ **Chat history stored** in SQLite database  
✅ **Real-time updates** via WebSocket  
✅ **ROS 2 integration** non-intrusive  

---

**Created:** December 12, 2025  
**Status:** ⏳ Planning Phase  
**Next Steps:** Begin Phase 1 implementation

**Related Documents:**
- `001_ARCHITECTURE_OVERVIEW.md` - System architecture
- `012_VPN_SETUP_TAILSCALE.md` - Tailscale VPN setup
- `040_FACE_RECOGNITION_COMPLETE.md` - Perception pipeline & Face recognition system

