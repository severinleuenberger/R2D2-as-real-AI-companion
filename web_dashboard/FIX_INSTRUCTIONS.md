# Fix Instructions for Web Dashboard Issues

## Current Issues
1. ❌ Passwordless sudo NOT configured - service control buttons fail
2. ❌ Status doesn't update automatically (only on manual refresh)
3. ❌ Both stream and recognition running (should be mutually exclusive)

## Step-by-Step Fix

### Step 1: Configure Passwordless Sudo (REQUIRED - DO THIS FIRST)

**Run this command in your terminal:**
```bash
bash ~/dev/r2d2/web_dashboard/fix_passwordless_sudo.sh
```

You will be prompted for your sudo password. This is required for the web dashboard to control services.

**Verify it worked:**
```bash
sudo -n systemctl status r2d2-audio-notification.service > /dev/null 2>&1 && echo "✅ Passwordless sudo works!" || echo "❌ Still not working"
```

### Step 2: Clear Browser Cache (IMPORTANT)

The browser may be caching the old JavaScript file. You need to do a **hard refresh**:

- **Chrome/Edge:** Press `Ctrl+Shift+R` or `Ctrl+F5`
- **Firefox:** Press `Ctrl+Shift+R` or `Ctrl+F5`
- **Or:** Open Developer Tools (F12) → Right-click refresh button → "Empty Cache and Hard Reload"

### Step 3: Restart Web Dashboard Service (Optional but Recommended)

To ensure the latest JavaScript is being served:

```bash
sudo systemctl restart r2d2-web-dashboard.service
```

Wait 5 seconds, then refresh your browser.

### Step 4: Test the Fixes

After completing steps 1-3:

1. **Hard refresh** the web dashboard page (`Ctrl+Shift+R`)
2. Wait 5 seconds for initialization
3. Check that:
   - Camera stream shows as "not active" (should be stopped by default)
   - Recognition status updates automatically (watch for changes)
   - Mode shows "Recognition Active" (not "Streaming Active")
4. Test buttons:
   - Click "Stop Recognition" - should work without password error
   - Click "Start Recognition" - should work without password error

### Step 5: Check Browser Console (If Still Not Working)

If status still doesn't update:

1. Open Developer Tools (F12)
2. Go to Console tab
3. Look for:
   - "subscribeToTopics called" - should appear
   - "person_status received" - should appear when status changes
   - Any error messages in red

4. Go to Network tab
5. Filter for "ingest" 
6. Look for POST requests - these are debug logs being sent

## Expected Behavior After Fix

✅ Status updates automatically via ROS topics (every ~100ms)
✅ Camera stream stopped by default
✅ Recognition active by default  
✅ All buttons work without password errors
✅ Starting one mode stops the other (mutual exclusivity)

## If Still Having Issues

Check the debug logs:
```bash
cat ~/.cursor/debug.log | tail -50
```

Look for entries with hypothesisId A, B, C, D, E, F to see what's happening.

