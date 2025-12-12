// R2D2 Web Dashboard JavaScript

// Configuration
// rosbridge URL: Use current hostname (works with Tailscale VPN)
const ROSBRIDGE_URL = `ws://${window.location.hostname}:9090`;
const API_BASE_URL = '/api';

// Global state
let ros = null;
let personStatusTopic = null;
let personIdTopic = null;
let faceCountTopic = null;
let heartbeatTopic = null;
let currentStatus = null;
let currentTaskId = null;
let audioServiceRunning = false;
let lastHeartbeatTime = null;
let heartbeatCheckInterval = null;
let cameraStreamServiceRunning = false;

// Initialize on page load
document.addEventListener('DOMContentLoaded', () => {
    initializeROS();
    loadServices(); // This will also update status display based on service state
    loadVolume();
    loadPeopleList();
    startServiceStatusPolling();
    checkSystemStatus();
    // Initialize status display with white (service not running) state
    updateStatusDisplayBasedOnService(false);
    // Start heartbeat monitoring
    startHeartbeatMonitoring();
    // Initialize camera stream display
    updateCameraStreamDisplay();
});

// Check overall system status
async function checkSystemStatus() {
    try {
        // Check if web server is responding
        const healthResponse = await fetch('/health');
        const health = await healthResponse.json();
        
        // Check services
        const servicesResponse = await fetch('/api/services/status');
        const services = await servicesResponse.json();
        
        // Count running services
        let runningCount = 0;
        let totalCount = 0;
        for (const [name, info] of Object.entries(services)) {
            totalCount++;
            if (info.status === 'active') {
                runningCount++;
            }
        }
        
        // Update status with service info if disconnected
        const statusDetails = document.getElementById('ros-status-details');
        if (statusDetails && (!ros || (ros && ros.isConnected === false))) {
            // Only update if disconnected, to avoid overwriting connection message
            if (runningCount > 0) {
                statusDetails.textContent = `${runningCount}/${totalCount} services running (Web API: ✓) | Install rosbridge for real-time updates`;
            } else {
                statusDetails.textContent = 'Web API: ✓ | Install & start rosbridge for real-time updates';
            }
        }
    } catch (error) {
        console.error('Failed to check system status:', error);
    }
}

// ROS Connection
function initializeROS() {
    ros = new ROSLIB.Ros({
        url: ROSBRIDGE_URL
    });
    ros.isConnected = false;

    ros.on('connection', () => {
        console.log('Connected to rosbridge');
        ros.isConnected = true;
        updateConnectionStatus(true);
        subscribeToTopics();
    });

    ros.on('error', (error) => {
        console.error('ROS error:', error);
        ros.isConnected = false;
        updateConnectionStatus(false);
    });

    ros.on('close', () => {
        console.log('ROS connection closed');
        ros.isConnected = false;
        updateConnectionStatus(false);
        // Try to reconnect after 3 seconds
        setTimeout(initializeROS, 3000);
    });
}

function updateConnectionStatus(connected) {
    const statusIndicator = document.getElementById('ros-status');
    const statusText = document.getElementById('ros-status-text');
    const statusDetails = document.getElementById('ros-status-details');
    
    if (connected) {
        statusIndicator.className = 'status-indicator connected';
        statusText.textContent = '✓ Real-time Connected';
        statusDetails.textContent = 'Receiving live updates';
        statusDetails.className = 'status-details-text connected';
    } else {
        statusIndicator.className = 'status-indicator disconnected';
        statusText.textContent = '✗ Real-time Disconnected';
        statusDetails.textContent = 'Start rosbridge: ./start_rosbridge.sh (or: ros2 run rosbridge_server rosbridge_websocket)';
        statusDetails.className = 'status-details-text disconnected';
    }
}

function subscribeToTopics() {
    // Subscribe to person_status
    personStatusTopic = new ROSLIB.Topic({
        ros: ros,
        name: '/r2d2/audio/person_status',
        messageType: 'std_msgs/String'
    });

    personStatusTopic.subscribe((message) => {
        try {
            const status = JSON.parse(message.data);
            updateStatusDisplay(status);
            addStreamMessage('status', `Status: ${status.status.toUpperCase()} - ${status.person_identity}`);
        } catch (e) {
            console.error('Failed to parse status:', e);
        }
    });

    // Subscribe to person_id
    personIdTopic = new ROSLIB.Topic({
        ros: ros,
        name: '/r2d2/perception/person_id',
        messageType: 'std_msgs/String'
    });

    personIdTopic.subscribe((message) => {
        const personId = message.data;
        document.getElementById('person-id').textContent = personId;
        if (personId !== 'unknown' && personId !== 'no_person') {
            addStreamMessage('recognition', `✅ Person recognized: ${personId}`);
        } else if (personId === 'unknown') {
            addStreamMessage('unknown', `👤 Unknown person detected`);
        }
    });

    // Subscribe to face_count
    faceCountTopic = new ROSLIB.Topic({
        ros: ros,
        name: '/r2d2/perception/face_count',
        messageType: 'std_msgs/Int32'
    });

    faceCountTopic.subscribe((message) => {
        document.getElementById('face-count').textContent = message.data;
    });

    // Subscribe to heartbeat
    heartbeatTopic = new ROSLIB.Topic({
        ros: ros,
        name: '/r2d2/heartbeat',
        messageType: 'std_msgs/String'
    });

    heartbeatTopic.subscribe((message) => {
        try {
            const heartbeatData = JSON.parse(message.data);
            updateHealthDisplay(heartbeatData);
            lastHeartbeatTime = Date.now();
        } catch (e) {
            console.error('Failed to parse heartbeat:', e);
        }
    });
}

// Status Display
function updateStatusDisplay(status) {
    // Only update if audio service is running
    if (!audioServiceRunning) {
        return;
    }
    
    currentStatus = status;
    
    const statusColor = document.getElementById('status-color');
    const statusText = document.getElementById('status-text');
    const personId = document.getElementById('person-id');
    const confidence = document.getElementById('confidence');
    const duration = document.getElementById('duration');
    
    // Update color - only if service is running
    statusColor.className = 'status-color';
    if (status.status === 'red') {
        statusColor.classList.add('red');
        statusText.textContent = '🔴 RED - Target Person Recognized';
    } else if (status.status === 'blue') {
        statusColor.classList.add('blue');
        statusText.textContent = '🔵 BLUE - No Person (Idle)';
    } else if (status.status === 'green') {
        statusColor.classList.add('green');
        statusText.textContent = '🟢 GREEN - Unknown Person';
    }
    
    // Update details
    personId.textContent = status.person_identity || 'no_person';
    confidence.textContent = status.confidence ? `${Math.round(status.confidence * 100)}%` : '0%';
    duration.textContent = status.duration_in_state ? `${status.duration_in_state.toFixed(1)}s` : '0s';
}

// Update status display based on service state
function updateStatusDisplayBasedOnService(anyServiceRunning) {
    const statusColor = document.getElementById('status-color');
    const statusText = document.getElementById('status-text');
    const serviceMessage = document.getElementById('status-service-message');
    
    if (!audioServiceRunning) {
        // Audio service not running - show white and message
        statusColor.className = 'status-color'; // White background (default)
        statusText.textContent = 'Service Not Running';
        if (serviceMessage) {
            if (!anyServiceRunning) {
                serviceMessage.textContent = '⚠️ No services are running. Start the audio service to see status colors.';
            } else {
                serviceMessage.textContent = '⚠️ Audio service is not running. Start the service to see status colors.';
            }
            serviceMessage.style.display = 'block';
        }
    } else {
        // Service is running - hide message, colors will be set by status updates
        if (serviceMessage) {
            serviceMessage.style.display = 'none';
        }
        // If we have a current status, update it
        if (currentStatus) {
            updateStatusDisplay(currentStatus);
        } else {
            // Service running but no status yet - show default blue
            statusColor.className = 'status-color blue';
            statusText.textContent = '🔵 BLUE - No Person (Idle)';
        }
    }
}

// Service Control
async function loadServices() {
    try {
        const response = await fetch(`${API_BASE_URL}/services/status`);
        const services = await response.json();
        displayServices(services);
    } catch (error) {
        console.error('Failed to load services:', error);
    }
}

function displayServices(services) {
    const servicesList = document.getElementById('services-list');
    servicesList.innerHTML = '';
    
    // Check if audio service is running
    audioServiceRunning = false;
    let anyServiceRunning = false;
    let previousCameraStreamStatus = cameraStreamServiceRunning;
    
    for (const [serviceName, serviceInfo] of Object.entries(services)) {
        const serviceCard = document.createElement('div');
        serviceCard.className = `service-card ${serviceInfo.status}`;
        
        const statusText = serviceInfo.status === 'active' ? '● Running' : '○ Stopped';
        
        // Check if this is the audio service (the key is "audio" in the API)
        if (serviceName === 'audio') {
            audioServiceRunning = (serviceInfo.status === 'active');
        }
        
        // Check if this is the camera stream service
        if (serviceName === 'camera-stream') {
            cameraStreamServiceRunning = (serviceInfo.status === 'active');
        }
        
        if (serviceInfo.status === 'active') {
            anyServiceRunning = true;
        }
        
        serviceCard.innerHTML = `
            <div>
                <div class="service-name">${serviceName}</div>
                <div style="font-size: 12px; color: #666;">${statusText}</div>
            </div>
            <div class="service-controls">
                <button class="btn btn-start" onclick="controlService('${serviceName}', 'start')">Start</button>
                <button class="btn btn-stop" onclick="controlService('${serviceName}', 'stop')">Stop</button>
                <button class="btn btn-restart" onclick="controlService('${serviceName}', 'restart')">Restart</button>
            </div>
        `;
        
        servicesList.appendChild(serviceCard);
    }
    
    // Update status display based on service state
    updateStatusDisplayBasedOnService(anyServiceRunning);
    
    // Update camera stream display if status changed
    if (previousCameraStreamStatus !== cameraStreamServiceRunning) {
        updateCameraStreamDisplay();
    }
}

// Camera Stream Functions
function toggleCameraStream() {
    const action = cameraStreamServiceRunning ? 'stop' : 'start';
    controlService('camera-stream', action);
}

function updateCameraStreamDisplay() {
    const streamContainer = document.getElementById('stream-container-wrapper');
    const streamPlaceholder = document.getElementById('stream-placeholder');
    const streamImg = document.getElementById('camera-stream');
    const streamStatus = document.getElementById('stream-status');
    const toggleBtn = document.getElementById('stream-toggle-btn');
    
    if (cameraStreamServiceRunning) {
        // Service is running - show stream
        if (streamPlaceholder) streamPlaceholder.style.display = 'none';
        if (streamContainer) streamContainer.style.display = 'block';
        if (streamStatus) {
            streamStatus.textContent = '● Running';
            streamStatus.className = 'stream-status running';
        }
        if (toggleBtn) toggleBtn.textContent = 'Stop Stream';
        if (streamImg) {
            const streamUrl = `http://${window.location.hostname}:8081/stream`;
            streamImg.src = streamUrl;
            streamImg.onerror = () => {
                if (streamImg) streamImg.style.display = 'none';
                if (streamPlaceholder) {
                    streamPlaceholder.style.display = 'block';
                    streamPlaceholder.innerHTML = '<p>Stream unavailable</p>';
                }
            };
        }
    } else {
        // Service is stopped - hide stream
        if (streamContainer) streamContainer.style.display = 'none';
        if (streamPlaceholder) {
            streamPlaceholder.style.display = 'block';
            streamPlaceholder.innerHTML = '<p>Camera stream not active</p>';
        }
        if (streamStatus) {
            streamStatus.textContent = '● Stopped';
            streamStatus.className = 'stream-status stopped';
        }
        if (toggleBtn) toggleBtn.textContent = 'Start Stream';
        if (streamImg) streamImg.src = '';
    }
}

async function controlService(serviceName, action) {
    try {
        const response = await fetch(`${API_BASE_URL}/services/${serviceName}/${action}`, {
            method: 'POST'
        });
        const result = await response.json();
        
        if (result.success) {
            addStreamMessage('status', `Service ${serviceName} ${action}ed successfully`);
            setTimeout(() => {
                loadServices();  // Reload after 1 second to update status display
                // If audio service was started, status will update automatically
            }, 1000);
        } else {
            alert(`Failed to ${action} service: ${result.error || 'Unknown error'}`);
        }
    } catch (error) {
        console.error(`Failed to ${action} service:`, error);
        alert(`Failed to ${action} service`);
    }
}

function startServiceStatusPolling() {
    setInterval(loadServices, 5000);  // Poll every 5 seconds
}

// Volume Control
async function loadVolume() {
    try {
        const response = await fetch(`${API_BASE_URL}/audio/volume`);
        const data = await response.json();
        const volumePercent = Math.round(data.volume * 100);
        document.getElementById('volume-slider').value = volumePercent;
        document.getElementById('volume-value').textContent = `${volumePercent}%`;
    } catch (error) {
        console.error('Failed to load volume:', error);
    }
}

document.getElementById('volume-slider').addEventListener('input', async (e) => {
    const volumePercent = parseInt(e.target.value);
    document.getElementById('volume-value').textContent = `${volumePercent}%`;
    
    const volume = volumePercent / 100;
    try {
        const response = await fetch(`${API_BASE_URL}/audio/volume`, {
            method: 'POST',
            headers: {'Content-Type': 'application/json'},
            body: JSON.stringify({volume: volume})
        });
        const result = await response.json();
        if (result.success) {
            addStreamMessage('status', `Volume set to ${volumePercent}%`);
        }
    } catch (error) {
        console.error('Failed to set volume:', error);
    }
});

function setVolumePreset(percent) {
    document.getElementById('volume-slider').value = percent;
    document.getElementById('volume-value').textContent = `${percent}%`;
    
    const volume = percent / 100;
    fetch(`${API_BASE_URL}/audio/volume`, {
        method: 'POST',
        headers: {'Content-Type': 'application/json'},
        body: JSON.stringify({volume: volume})
    }).then(() => {
        addStreamMessage('status', `Volume set to ${percent}%`);
    });
}

// Training Control
async function startTraining(option) {
    const personName = document.getElementById('person-name-input').value.trim();
    if (!personName) {
        alert('Please enter a person name');
        return;
    }
    
    const endpoints = {
        1: '/capture',
        2: '/add_pictures',
        3: '/retrain',
        4: '/test_accuracy',
        5: '/realtime_test'
    };
    
    const endpoint = endpoints[option];
    if (!endpoint) {
        alert('Invalid training option');
        return;
    }
    
    try {
        const response = await fetch(`${API_BASE_URL}/training${endpoint}`, {
            method: 'POST',
            headers: {'Content-Type': 'application/json'},
            body: JSON.stringify({person_name: personName, option: option})
        });
        const result = await response.json();
        
        if (result.task_id) {
            currentTaskId = result.task_id;
            showTrainingStatus(result.task_id);
            pollTrainingStatus(result.task_id);
        } else {
            alert(result.message || 'Training started');
        }
    } catch (error) {
        console.error('Failed to start training:', error);
        alert('Failed to start training');
    }
}

async function pollTrainingStatus(taskId) {
    try {
        const response = await fetch(`${API_BASE_URL}/training/status/${taskId}`);
        const status = await response.json();
        
        updateTrainingStatus(status);
        
        if (status.status === 'running') {
            setTimeout(() => pollTrainingStatus(taskId), 1000);  // Poll every second
        }
    } catch (error) {
        console.error('Failed to get training status:', error);
    }
}

function showTrainingStatus(taskId) {
    document.getElementById('training-status').style.display = 'block';
    document.getElementById('training-progress').textContent = 'Running...';
    document.getElementById('training-logs').innerHTML = '';
}

function updateTrainingStatus(status) {
    const progress = document.getElementById('training-progress');
    const logs = document.getElementById('training-logs');
    
    progress.textContent = `${status.current_step} (${status.status})`;
    
    // Update logs
    logs.innerHTML = '';
    status.logs.slice(-20).forEach(log => {
        const logDiv = document.createElement('div');
        logDiv.textContent = log;
        logs.appendChild(logDiv);
    });
    logs.scrollTop = logs.scrollHeight;
    
    if (status.status === 'completed') {
        progress.textContent = 'Completed!';
        addStreamMessage('status', `Training completed for ${status.person_name}`);
        loadPeopleList();
    } else if (status.status === 'failed') {
        progress.textContent = `Failed: ${status.error || 'Unknown error'}`;
    }
}

async function listPeople() {
    try {
        const response = await fetch(`${API_BASE_URL}/training/list`);
        const data = await response.json();
        displayPeopleList(data);
    } catch (error) {
        console.error('Failed to load people list:', error);
    }
}

function displayPeopleList(data) {
    const peopleList = document.getElementById('people-list');
    peopleList.innerHTML = '<h3>Trained People</h3>';
    
    if (data.trained_people && data.trained_people.length > 0) {
        data.trained_people.forEach(person => {
            const item = document.createElement('div');
            item.className = 'people-list-item';
            item.innerHTML = `
                <div>
                    <strong>${person.name}</strong><br>
                    <small>Model: ${person.model_size_kb} KB | Images: ${person.image_count}</small>
                </div>
            `;
            peopleList.appendChild(item);
        });
    } else {
        peopleList.innerHTML += '<p>No trained people yet</p>';
    }
}

async function loadPeopleList() {
    await listPeople();
}

function showDeleteDialog() {
    document.getElementById('delete-modal').style.display = 'flex';
}

function closeDeleteDialog() {
    document.getElementById('delete-modal').style.display = 'none';
}

async function confirmDelete() {
    const personName = document.getElementById('delete-person-input').value.trim();
    if (!personName) {
        alert('Please enter a person name');
        return;
    }
    
    if (!confirm(`Are you sure you want to delete ${personName}? This will remove all images and model.`)) {
        return;
    }
    
    try {
        const response = await fetch(`${API_BASE_URL}/training/${personName}`, {
            method: 'DELETE'
        });
        const result = await response.json();
        
        if (result.success) {
            addStreamMessage('status', `Deleted ${personName}`);
            closeDeleteDialog();
            loadPeopleList();
        } else {
            alert(`Failed to delete: ${result.error || 'Unknown error'}`);
        }
    } catch (error) {
        console.error('Failed to delete person:', error);
        alert('Failed to delete person');
    }
}

// Heartbeat Monitoring
function startHeartbeatMonitoring() {
    // Check heartbeat status every second
    heartbeatCheckInterval = setInterval(() => {
        checkHeartbeatStatus();
    }, 1000);
}

function checkHeartbeatStatus() {
    const now = Date.now();
    const statusIcon = document.getElementById('health-status-icon');
    const statusText = document.getElementById('health-status-text');
    const timestamp = document.getElementById('health-timestamp');
    
    if (lastHeartbeatTime && (now - lastHeartbeatTime) < 3000) {
        // Heartbeat received within last 3 seconds - R2D2 is running
        statusIcon.className = 'health-status-icon running';
        statusIcon.textContent = '🟢';
        statusText.textContent = 'R2D2 is Running';
    } else {
        // No heartbeat for >3 seconds - R2D2 not responding
        statusIcon.className = 'health-status-icon stopped';
        statusIcon.textContent = '🔴';
        statusText.textContent = 'R2D2 Not Responding';
        if (timestamp) {
            timestamp.textContent = 'No heartbeat received';
        }
    }
}

function updateHealthDisplay(heartbeatData) {
    // Update status
    const statusIcon = document.getElementById('health-status-icon');
    const statusText = document.getElementById('health-status-text');
    const timestamp = document.getElementById('health-timestamp');
    
    if (heartbeatData.status === 'running') {
        statusIcon.className = 'health-status-icon running';
        statusIcon.textContent = '🟢';
        statusText.textContent = 'R2D2 is Running';
    } else {
        statusIcon.className = 'health-status-icon stopped';
        statusIcon.textContent = '🔴';
        statusText.textContent = 'R2D2 Not Responding';
    }
    
    if (timestamp && heartbeatData.timestamp) {
        const date = new Date(heartbeatData.timestamp);
        timestamp.textContent = `Last update: ${date.toLocaleTimeString()}`;
    }
    
    // Update CPU usage
    if (heartbeatData.cpu_percent !== undefined) {
        const cpuValue = document.getElementById('cpu-value');
        const cpuBarFill = document.getElementById('cpu-bar-fill');
        const cpuPercent = heartbeatData.cpu_percent;
        
        if (cpuValue) {
            cpuValue.textContent = `${cpuPercent}%`;
        }
        if (cpuBarFill) {
            cpuBarFill.style.width = `${Math.min(cpuPercent, 100)}%`;
        }
    }
    
    // Update GPU usage
    if (heartbeatData.gpu_percent !== undefined) {
        const gpuValue = document.getElementById('gpu-value');
        const gpuBarFill = document.getElementById('gpu-bar-fill');
        const gpuPercent = heartbeatData.gpu_percent;
        
        if (gpuValue) {
            gpuValue.textContent = `${gpuPercent}%`;
        }
        if (gpuBarFill) {
            gpuBarFill.style.width = `${Math.min(gpuPercent, 100)}%`;
        }
    }
    
    // Update temperature
    if (heartbeatData.temperature_c !== undefined) {
        const tempValue = document.getElementById('temperature-value');
        const temp = heartbeatData.temperature_c;
        
        if (tempValue) {
            tempValue.textContent = temp.toFixed(1);
            // Color code based on temperature
            tempValue.className = 'temperature-value';
            if (temp < 50) {
                tempValue.classList.add('cool');
            } else if (temp < 70) {
                tempValue.classList.add('warm');
            } else {
                tempValue.classList.add('hot');
            }
        }
    }
}

// Stream Messages
function addStreamMessage(type, message) {
    const container = document.getElementById('stream-container');
    const messageDiv = document.createElement('div');
    messageDiv.className = `stream-message ${type}`;
    
    const timestamp = new Date().toLocaleTimeString();
    messageDiv.innerHTML = `<span class="stream-timestamp">${timestamp}</span>${message}`;
    
    container.insertBefore(messageDiv, container.firstChild);
    
    // Keep only last 50 messages
    while (container.children.length > 50) {
        container.removeChild(container.lastChild);
    }
}

