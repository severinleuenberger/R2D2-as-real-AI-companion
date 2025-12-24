#!/bin/bash
# Script to fix Tailscale SSH connectivity issues
# This script will:
# 1. Check Tailscale status
# 2. Restart Tailscale service if needed
# 3. Check DNS configuration
# 4. Verify SSH is accessible

LOG_PATH="/home/severin/.cursor/debug.log"
SESSION_ID="debug-session"
RUN_ID="fix-run"

log() {
    local hypothesis_id=$1
    local location=$2
    local message=$3
    local data=$4
    local timestamp=$(date +%s%3N)
    local entry=$(cat <<EOF
{"id":"log_${timestamp}_${hypothesis_id}","timestamp":${timestamp},"location":"${location}","message":"${message}","data":${data},"sessionId":"${SESSION_ID}","runId":"${RUN_ID}","hypothesisId":"${hypothesis_id}"}
EOF
)
    echo "$entry" >> "$LOG_PATH"
}

echo "=== Tailscale SSH Fix Script ==="

# #region agent log
log "FIX" "fix_tailscale_ssh.sh:start" "Fix script started" "{}"
# #endregion

# Check current status
echo "1. Checking Tailscale status..."
TS_STATUS=$(tailscale status 2>&1)
TS_IP=$(tailscale ip -4 2>&1)

# #region agent log
log "FIX" "fix_tailscale_ssh.sh:status" "Tailscale status before fix" "{\"status\":\"$TS_STATUS\",\"ip\":\"$TS_IP\"}"
# #endregion

echo "   Tailscale IP: $TS_IP"
echo "   Status:"
echo "$TS_STATUS" | head -5

# Check if Windows device is visible
if echo "$TS_STATUS" | grep -q "itxcl883.*offline"; then
    echo ""
    echo "2. ⚠️  WARNING: Windows device (itxcl883) appears offline in Tailscale"
    echo "   This is likely why SSH connections are failing."
    echo "   Solution: Restart Tailscale on your Windows machine"
    echo ""
fi

# Try to fix connectivity by restarting Tailscale
echo "3. Attempting to restart Tailscale service..."
# Try without password first (if user has passwordless sudo for systemctl)
if sudo -n systemctl restart tailscaled 2>&1; then
    echo "   ✓ Tailscale service restarted"
    sleep 2
    
    # #region agent log
    log "FIX" "fix_tailscale_ssh.sh:restart" "Tailscale service restarted" "{\"success\":true}"
    # #endregion
    
    # Check status after restart
    NEW_STATUS=$(tailscale status 2>&1)
    NEW_IP=$(tailscale ip -4 2>&1)
    
    # #region agent log
    log "FIX" "fix_tailscale_ssh.sh:status_after" "Tailscale status after restart" "{\"status\":\"$NEW_STATUS\",\"ip\":\"$NEW_IP\"}"
    # #endregion
    
    echo "   New status:"
    echo "$NEW_STATUS" | head -5
else
    echo "   ✗ Failed to restart (may need password)"
    # #region agent log
    log "FIX" "fix_tailscale_ssh.sh:restart" "Tailscale restart failed" "{\"success\":false}"
    # #endregion
fi

# Check SSH
echo ""
echo "4. Checking SSH service..."
if netstat -tlnp 2>/dev/null | grep -q ":22 "; then
    echo "   ✓ SSH is listening on port 22"
    # #region agent log
    log "FIX" "fix_tailscale_ssh.sh:ssh_check" "SSH port check" "{\"listening\":true}"
    # #endregion
else
    echo "   ✗ SSH is NOT listening on port 22"
    # #region agent log
    log "FIX" "fix_tailscale_ssh.sh:ssh_check" "SSH port check" "{\"listening\":false}"
    # #endregion
fi

echo ""
echo "=== Summary ==="
echo "Jetson Tailscale IP: $TS_IP"
echo ""
echo "Next steps:"
echo "1. On your Windows machine, restart Tailscale:"
echo "   - Right-click Tailscale icon → Exit"
echo "   - Open Tailscale from Start menu"
echo "2. Verify Windows device appears online in:"
echo "   https://login.tailscale.com/admin/machines"
echo "3. Try SSH again: ssh jetson-tailscale"

# #region agent log
log "FIX" "fix_tailscale_ssh.sh:end" "Fix script completed" "{}"
# #endregion

