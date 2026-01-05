// R2D2 Diagnostics Page - Safe Implementation
// Read-only by default with protection levels for critical services

// ============================================================================
// Global State
// ============================================================================

let readOnlyMode = true;
let ros = null;
let activeTopicMonitor = null;
let statusTopics = {};
let fistStartTime = null;
let lastGestureTime = null;
let lastPersonStatus = null;
let silenceStartTime = null;
let lastVADState = null;

// Service protection configuration
const SERVICE_PROTECTION = {
    'r2d2-camera-perception': { 
        level: 'critical', 
        icon: '🛡️',
        warning: 'Face recognition, gesture detection, and LED feedback will stop!' 
    },
    'r2d2-audio-notification': { 
        level: 'critical', 
        icon: '🛡️',
        warning: 'LED control and audio alerts will stop!' 
    },
    'r2d2-gesture-intent': { 
        level: 'critical', 
        icon: '🛡️',
        warning: 'Gesture-to-speech control will stop!' 
    },
    'r2d2-speech-node': { 
        level: 'high', 
        icon: '⚡',
        warning: 'Fast Mode conversations will be disabled!' 
    },
    'r2d2-rest-speech-node': { 
        level: 'high', 
        icon: '⚡',
        warning: 'R2-D2 Mode conversations will be disabled!' 
    },
    'r2d2-volume-control': { 
        level: 'high', 
        icon: '⚡',
        warning: 'Audio volume control will stop!' 
    },
    'r2d2-heartbeat': { level: 'low', icon: '', warning: null },
    'r2d2-powerbutton': { level: 'low', icon: '', warning: null },
    'r2d2-wake-api': { level: 'low', icon: '', warning: null },
    'r2d2-rosbridge': { level: 'low', icon: '', warning: null },
    'r2d2-camera-stream': { level: 'low', icon: '', warning: null },
    'r2d2-web-dashboard': { level: 'low', icon: '', warning: null }
};

// ============================================================================
// Initialization
// ============================================================================

document.addEventListener('DOMContentLoaded', function() {
    console.log('Initializing R2D2 Diagnostics Dashboard...');
    
    // Initialize in safe read-only mode
    initSafeMode();
    
    // Connect to rosbridge
    connectToRosbridge();
    
    // Load service status
    loadServiceStatus();
    
    // Load LED status
    updateLEDStatus();
    
    // Start periodic updates
    setInterval(loadServiceStatus, 5000);  // Update services every 5s
    setInterval(updateLEDStatus, 2000);     // Update LED every 2s
    setInterval(updateTimers, 500);         // Update timers twice per second
});

// ============================================================================
// Safety Layer
// ============================================================================

function initSafeMode() {
    readOnlyMode = true;
    document.querySelectorAll('.service-control-btn').forEach(btn => {
        btn.style.display = 'none';
    });
    const indicator = document.getElementById('mode-indicator');
    if (indicator) {
        indicator.textContent = '🔒 READ-ONLY MODE';
        indicator.className = 'mode-safe';
    }
    const btn = document.getElementById('toggle-control-btn');
    if (btn) {
        btn.textContent = '🔓 Enable Control';
    }
}

function toggleControlMode() {
    if (readOnlyMode) {
        const confirmed = confirm(
            '⚠️ Enable Service Control?\n\n' +
            'This allows starting/stopping services which may affect robot operation.\n\n' +
            'Critical services (🛡️) will require additional confirmation.\n\n' +
            'Enable control mode?'
        );
        if (confirmed) {
            readOnlyMode = false;
            document.querySelectorAll('.service-control-btn').forEach(btn => {
                btn.style.display = 'inline-block';
            });
            document.getElementById('mode-indicator').textContent = '🔓 CONTROL MODE';
            document.getElementById('mode-indicator').className = 'mode-control';
            document.getElementById('toggle-control-btn').textContent = '🔒 Lock Controls';
        }
    } else {
        initSafeMode();
    }
}

async function controlService(serviceName, action) {
    if (readOnlyMode) {
        alert('Please enable Control Mode first');
        return;
    }
    
    const protection = SERVICE_PROTECTION[serviceName];
    if (!protection) {
        console.warn('Unknown service:', serviceName);
        return;
    }
    
    // Critical services: double confirmation
    if (protection.level === 'critical') {
        if (!confirm(`⚠️ CRITICAL SERVICE WARNING ⚠️\n\n${action}ing ${serviceName}\n\n${protection.warning}\n\nThe robot's core UX functionality will be affected!\n\nAre you absolutely sure?`)) {
            return;
        }
        if (!confirm(`Final confirmation: ${action} ${serviceName}?`)) {
            return;
        }
    }
    // High protection: single confirmation
    else if (protection.level === 'high') {
        if (!confirm(`⚡ Warning: ${action}ing ${serviceName}\n\n${protection.warning}\n\nContinue?`)) {
            return;
        }
    }
    // Low protection: simple confirmation
    else {
        if (!confirm(`${action} ${serviceName}?`)) {
            return;
        }
    }
    
    // Execute action
    try {
        const response = await fetch(`/api/diagnostics/services/${serviceName}/${action}`, {
            method: 'POST'
        });
        const result = await response.json();
        
        if (result.success) {
            showNotification(`✓ ${serviceName} ${action}ed successfully`, 'success');
            setTimeout(loadServiceStatus, 2000);  // Reload after 2s
        } else {
            showNotification(`✗ Failed: ${result.error}`, 'error');
        }
    } catch (error) {
        showNotification(`✗ Error: ${error.message}`, 'error');
    }
}

// ============================================================================
// Rosbridge Connection
// ============================================================================

function connectToRosbridge() {
    console.log('Connecting to rosbridge...');
    
    // Use current hostname to connect to rosbridge (works for localhost and remote access)
    const rosbridgeUrl = `ws://${window.location.hostname}:9090`;
    console.log('rosbridge URL:', rosbridgeUrl);
    
    ros = new ROSLIB.Ros({
        url: rosbridgeUrl
    });

    ros.on('connection', function() {
        console.log('✓ Connected to rosbridge');
        subscribeToStatusTopics();
    });

    ros.on('error', function(error) {
        console.error('✗ rosbridge error:', error);
        showNotification('rosbridge not connected. Start rosbridge service for live monitoring.', 'warning');
    });

    ros.on('close', function() {
        console.log('rosbridge connection closed');
        setTimeout(connectToRosbridge, 5000);  // Retry after 5s
    });
}

function subscribeToStatusTopics() {
    // Subscribe to person_status (main status indicators)
    statusTopics.personStatus = new ROSLIB.Topic({
        ros: ros,
        name: '/r2d2/audio/person_status',
        messageType: 'std_msgs/String'
    });
    statusTopics.personStatus.subscribe(function(message) {
        handlePersonStatus(message.data);
    });

    // Subscribe to face_confidence
    statusTopics.faceConfidence = new ROSLIB.Topic({
        ros: ros,
        name: '/r2d2/perception/face_confidence',
        messageType: 'std_msgs/Float32'
    });
    statusTopics.faceConfidence.subscribe(function(message) {
        updateConfidence(message.data);
    });

    // Subscribe to face_count
    statusTopics.faceCount = new ROSLIB.Topic({
        ros: ros,
        name: '/r2d2/perception/face_count',
        messageType: 'std_msgs/Int32'
    });
    statusTopics.faceCount.subscribe(function(message) {
        document.getElementById('faces-value').textContent = message.data;
    });

    // Subscribe to gesture_event
    statusTopics.gesture = new ROSLIB.Topic({
        ros: ros,
        name: '/r2d2/perception/gesture_event',
        messageType: 'std_msgs/String'
    });
    statusTopics.gesture.subscribe(function(message) {
        handleGesture(message.data);
    });

    // Subscribe to session_status (Fast Mode)
    statusTopics.sessionStatus = new ROSLIB.Topic({
        ros: ros,
        name: '/r2d2/speech/session_status',
        messageType: 'std_msgs/String'
    });
    statusTopics.sessionStatus.subscribe(function(message) {
        handleSessionStatus(message.data, 'fast');
    });

    // Subscribe to rest_session_status (R2-D2 Mode)
    statusTopics.restSessionStatus = new ROSLIB.Topic({
        ros: ros,
        name: '/r2d2/speech/intelligent/session_status',
        messageType: 'std_msgs/String'
    });
    statusTopics.restSessionStatus.subscribe(function(message) {
        handleSessionStatus(message.data, 'r2d2');
    });

    // Subscribe to voice_activity
    statusTopics.voiceActivity = new ROSLIB.Topic({
        ros: ros,
        name: '/r2d2/speech/voice_activity',
        messageType: 'std_msgs/String'
    });
    statusTopics.voiceActivity.subscribe(function(message) {
        handleVoiceActivity(message.data);
    });
}

// ============================================================================
// Status Indicator Handlers
// ============================================================================

function handlePersonStatus(data) {
    try {
        const status = JSON.parse(data);
        lastPersonStatus = status;
        
        // Update status badge
        const statusValue = document.getElementById('status-value');
        const statusUpper = status.status.toUpperCase();
        statusValue.textContent = statusUpper;
        statusValue.className = `status-badge status-${status.status}`;
        
        // Update person
        document.getElementById('person-value').textContent = status.person_identity || '--';
        
        // Update phase
        updatePhase();
        
    } catch (e) {
        console.error('Error parsing person_status:', e);
    }
}

function updateConfidence(value) {
    const percent = Math.round(value);
    document.getElementById('confidence-text').textContent = `${percent}%`;
    
    const fill = document.getElementById('confidence-fill');
    fill.style.width = `${percent}%`;
    
    // Color code: green >80, yellow >50, red <50
    if (percent >= 80) {
        fill.className = 'confidence-fill confidence-high';
    } else if (percent >= 50) {
        fill.className = 'confidence-fill confidence-medium';
    } else {
        fill.className = 'confidence-fill confidence-low';
    }
}

function handleGesture(gesture) {
    lastGestureTime = Date.now();
    
    const gestureMap = {
        'index_finger_up': '☝️',
        'fist': '✊',
        'open_hand': '🖐️'
    };
    
    document.getElementById('gesture-value').textContent = gestureMap[gesture] || gesture;
    
    // Track fist for two-stage stop
    if (gesture === 'fist') {
        if (!fistStartTime) {
            fistStartTime = Date.now();
        }
    } else {
        fistStartTime = null;
    }
}

function handleSessionStatus(data, mode) {
    try {
        const status = JSON.parse(data);
        const active = (mode === 'fast' && status.status === 'connected') ||
                      (mode === 'r2d2' && status.session_active === true);
        
        if (active) {
            const modeText = mode === 'fast' ? '🎙️ Fast Mode' : '🖐️ R2-D2 Mode';
            document.getElementById('speech-mode-value').textContent = modeText;
        } else {
            // Check if other mode is active
            const otherMode = mode === 'fast' ? 'r2d2' : 'fast';
            // Only set to OFF if we know for sure both are inactive
            if (document.getElementById('speech-mode-value').textContent.includes(mode === 'fast' ? 'Fast' : 'R2-D2')) {
                document.getElementById('speech-mode-value').textContent = '🔇 OFF';
            }
        }
        
        updatePhase();
    } catch (e) {
        console.error('Error parsing session_status:', e);
    }
}

function handleVoiceActivity(data) {
    try {
        const vad = JSON.parse(data);
        lastVADState = vad.status;
        
        document.getElementById('vad-value').textContent = vad.status === 'speaking' ? 'Speaking' : 'Silent';
        
        // Track silence start time
        if (vad.status === 'silent' && !silenceStartTime) {
            silenceStartTime = Date.now();
        } else if (vad.status === 'speaking') {
            silenceStartTime = null;
        }
    } catch (e) {
        console.error('Error parsing voice_activity:', e);
    }
}

function updatePhase() {
    if (!lastPersonStatus) {
        document.getElementById('phase-value').textContent = '--';
        return;
    }
    
    const status = lastPersonStatus.status.toUpperCase();
    const speechMode = document.getElementById('speech-mode-value').textContent;
    
    let phase = '';
    if (status === 'BLUE') {
        phase = 'Phase 1: Waiting';
    } else if (status === 'GREEN') {
        phase = 'Phase 3: Unknown';
    } else if (status === 'RED') {
        if (speechMode.includes('Fast') || speechMode.includes('R2-D2')) {
            phase = 'Phase 5-7: Active';
        } else {
            phase = 'Phase 4: Ready';
        }
    }
    
    document.getElementById('phase-value').textContent = phase;
}

function updateTimers() {
    const now = Date.now();
    
    // Clear gesture after 2s
    if (lastGestureTime && (now - lastGestureTime) > 2000) {
        document.getElementById('gesture-value').textContent = '--';
        lastGestureTime = null;
    }
    
    // Update fist stop progress
    if (fistStartTime) {
        const elapsed = (now - fistStartTime) / 1000;
        if (elapsed >= 3.0) {
            document.getElementById('fist-stop-value').textContent = 'Stage 2 (STOP)';
        } else if (elapsed >= 1.5) {
            document.getElementById('fist-stop-value').textContent = 'Stage 1 (Warning)';
        } else {
            document.getElementById('fist-stop-value').textContent = `${elapsed.toFixed(1)}s...`;
        }
    } else {
        document.getElementById('fist-stop-value').textContent = '--';
    }
    
    // Update silence timer
    if (silenceStartTime) {
        const elapsed = Math.floor((now - silenceStartTime) / 1000);
        document.getElementById('silence-timer').textContent = `${elapsed}/60s`;
    } else {
        document.getElementById('silence-timer').textContent = '--/60s';
    }
    
    // Cooldowns and watchdog would need backend data - placeholder for now
    document.getElementById('cooldowns-value').textContent = 'Start ✓ Stop ✓';
    document.getElementById('watchdog-value').textContent = '--/35s';
}

async function updateLEDStatus() {
    try {
        const response = await fetch('/api/diagnostics/gpio/17');
        const data = await response.json();
        
        if (data.state === 'ON') {
            document.getElementById('led-value').textContent = '💡 ON';
        } else if (data.state === 'OFF') {
            document.getElementById('led-value').textContent = '○ OFF';
        } else {
            document.getElementById('led-value').textContent = '--';
        }
    } catch (error) {
        console.error('Error reading LED status:', error);
    }
}

// ============================================================================
// Service Status
// ============================================================================

async function loadServiceStatus() {
    try {
        const response = await fetch('/api/diagnostics/services');
        const services = await response.json();
        
        renderServiceGrid(services);
    } catch (error) {
        console.error('Error loading service status:', error);
    }
}

function renderServiceGrid(services) {
    const grid = document.getElementById('service-grid');
    grid.innerHTML = '';
    
    const serviceList = [
        'r2d2-camera-perception',
        'r2d2-audio-notification',
        'r2d2-gesture-intent',
        'r2d2-volume-control',
        'r2d2-speech-node',
        'r2d2-rest-speech-node',
        'r2d2-heartbeat',
        'r2d2-powerbutton',
        'r2d2-wake-api',
        'r2d2-rosbridge',
        'r2d2-camera-stream',
        'r2d2-web-dashboard'
    ];
    
    serviceList.forEach(serviceName => {
        const serviceKey = serviceName.replace('r2d2-', '').replace('.service', '');
        const status = services[serviceKey] || { active: false };
        const protection = SERVICE_PROTECTION[serviceName];
        
        const item = document.createElement('div');
        item.className = `service-item service-${protection.level}`;
        
        const icon = protection.icon;
        const displayName = serviceName.replace('r2d2-', '').replace('.service', '');
        const statusBadge = status.active ? '[●]' : '[○]';
        const statusText = status.active ? 'Running' : 'Stopped';
        const statusClass = status.active ? 'status-running' : 'status-stopped';
        
        item.innerHTML = `
            <div class="service-name">${icon} ${displayName}</div>
            <div class="service-status ${statusClass}">${statusBadge} ${statusText}</div>
            <div class="service-controls" style="display: ${readOnlyMode ? 'none' : 'block'}">
                ${!status.active ? `<button class="btn-service-control service-control-btn" onclick="controlService('${serviceName}', 'start')">Start</button>` : ''}
                ${status.active ? `<button class="btn-service-control service-control-btn" onclick="controlService('${serviceName}', 'stop')">Stop</button>` : ''}
                ${status.active ? `<button class="btn-service-control service-control-btn" onclick="controlService('${serviceName}', 'restart')">Restart</button>` : ''}
            </div>
        `;
        
        grid.appendChild(item);
    });
}

// ============================================================================
// Topic Monitoring
// ============================================================================

function startTopicMonitor(topicName, messageType) {
    if (activeTopicMonitor) {
        stopTopicMonitor();
    }
    
    if (!ros || !ros.isConnected) {
        alert('rosbridge not connected. Please start rosbridge service first.');
        return;
    }
    
    activeTopicMonitor = new ROSLIB.Topic({
        ros: ros,
        name: topicName,
        messageType: messageType
    });
    
    const output = document.getElementById('topic-output');
    output.innerHTML = '';
    
    activeTopicMonitor.subscribe(function(message) {
        const timestamp = new Date().toLocaleTimeString();
        const msgText = typeof message.data !== 'undefined' ? message.data : JSON.stringify(message);
        
        const line = document.createElement('div');
        line.className = 'topic-line';
        line.textContent = `[${timestamp}] ${msgText}`;
        output.appendChild(line);
        
        // Auto-scroll
        output.scrollTop = output.scrollHeight;
        
        // Limit to last 100 lines
        while (output.children.length > 100) {
            output.removeChild(output.firstChild);
        }
    });
    
    document.getElementById('topic-output-container').style.display = 'block';
    document.getElementById('active-topic-name').textContent = topicName;
}

function stopTopicMonitor() {
    if (activeTopicMonitor) {
        activeTopicMonitor.unsubscribe();
        activeTopicMonitor = null;
    }
    document.getElementById('topic-output-container').style.display = 'none';
    document.getElementById('topic-output').innerHTML = '';
}

// ============================================================================
// Diagnostic Tests
// ============================================================================

async function runTest(testName) {
    const output = document.getElementById('test-output');
    output.innerHTML = `<div class="test-running">Running ${testName} test...</div>`;
    
    try {
        const response = await fetch(`/api/diagnostics/test/${testName}`, {
            method: 'POST'
        });
        const result = await response.json();
        
        displayTestResult(result);
    } catch (error) {
        output.innerHTML = `<div class="test-error">Error: ${error.message}</div>`;
    }
}

function displayTestResult(result) {
    const output = document.getElementById('test-output');
    output.innerHTML = '';
    
    // Format based on result structure
    if (result.output) {
        // Multi-line output
        const lines = result.output.split('\n');
        lines.forEach(line => {
            const div = document.createElement('div');
            div.className = 'test-line';
            if (line.includes('PASS') || line.includes('✓') || line.includes('✅')) {
                div.classList.add('test-pass');
            } else if (line.includes('FAIL') || line.includes('✗') || line.includes('❌')) {
                div.classList.add('test-fail');
            } else if (line.includes('WARN') || line.includes('⚠️')) {
                div.classList.add('test-warn');
            }
            div.textContent = line;
            output.appendChild(div);
        });
    } else {
        // Single result
        const div = document.createElement('div');
        div.className = 'test-line';
        const status = result.status || 'UNKNOWN';
        div.textContent = `Status: ${status} - ${result.message || JSON.stringify(result)}`;
        if (status === 'PASS') div.classList.add('test-pass');
        else if (status === 'FAIL') div.classList.add('test-fail');
        else if (status === 'WARN') div.classList.add('test-warn');
        output.appendChild(div);
    }
}

// ============================================================================
// Utilities
// ============================================================================

function showNotification(message, type = 'info') {
    console.log(`[${type.toUpperCase()}] ${message}`);
    // Could add toast notifications here
    alert(message);
}

