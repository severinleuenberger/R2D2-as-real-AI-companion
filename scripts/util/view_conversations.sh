#!/bin/bash
# R2D2 Conversation Viewer
# Quick script to view conversation history from the database

DB_PATH="/home/severin/dev/r2d2/r2d2_speech/data/conversations.db"

echo "╔════════════════════════════════════════════════════════════════╗"
echo "║         R2-D2 Conversation History Viewer                      ║"
echo "╚════════════════════════════════════════════════════════════════╝"
echo ""

if [ ! -f "$DB_PATH" ]; then
    echo "❌ Database not found at: $DB_PATH"
    exit 1
fi

# Check if arguments provided
if [ $# -eq 0 ]; then
    echo "Usage: $0 [command]"
    echo ""
    echo "Commands:"
    echo "  recent      - Show 10 most recent messages"
    echo "  today       - Show today's messages"
    echo "  sessions    - List all sessions"
    echo "  stats       - Show database statistics"
    echo "  all         - Show all messages"
    echo "  sql         - Open interactive sqlite3 shell"
    echo ""
    echo "Example: $0 recent"
    exit 0
fi

case "$1" in
    recent)
        sqlite3 "$DB_PATH" << 'EOF'
.mode column
.headers on
.width 10 60 20
SELECT role, text, created_at 
FROM messages 
ORDER BY id DESC 
LIMIT 10;
EOF
        ;;
    
    today)
        sqlite3 "$DB_PATH" << 'EOF'
.mode column
.headers on
.width 10 60 20
SELECT role, text, created_at 
FROM messages 
WHERE date(created_at) = date('now')
ORDER BY created_at ASC;
EOF
        ;;
    
    sessions)
        sqlite3 "$DB_PATH" << 'EOF'
.mode column
.headers on
SELECT session_id, started_at, ended_at 
FROM sessions 
ORDER BY started_at DESC;
EOF
        ;;
    
    stats)
        sqlite3 "$DB_PATH" << 'EOF'
.mode column
.headers on
SELECT 'Total Messages' as metric, COUNT(*) as count FROM messages
UNION ALL
SELECT 'User Messages', COUNT(*) FROM messages WHERE role = 'user'
UNION ALL
SELECT 'Assistant Messages', COUNT(*) FROM messages WHERE role = 'assistant'
UNION ALL
SELECT 'Total Sessions', COUNT(*) FROM sessions;
EOF
        ;;
    
    all)
        sqlite3 "$DB_PATH" << 'EOF'
.mode column
.headers on
.width 10 60 20
SELECT role, text, created_at 
FROM messages 
ORDER BY created_at ASC;
EOF
        ;;
    
    sql)
        echo "Opening interactive SQLite shell..."
        echo "Type .quit to exit"
        echo ""
        sqlite3 "$DB_PATH"
        ;;
    
    *)
        echo "❌ Unknown command: $1"
        echo "Run without arguments to see available commands"
        exit 1
        ;;
esac


