# R2D2 Integration Guide: Adding Phase 2-4 Features
**Date:** December 7, 2025  
**Audience:** Developers implementing speech, navigation, multi-modal features  
**References:** ARCHITECTURE_OVERVIEW.md (design), OPERATIONS_CHECKLIST.md (operational context)

---

## Quick Reference: Integration Checklist

```
Adding a new Phase 2-4 feature?

☐ Step 1: Understand the current architecture (read Section 1 below)
☐ Step 2: Decide: subscribe to existing topics or new pipeline?
☐ Step 3: Create new ROS 2 package (Section 2.1)
☐ Step 4: Implement node with proper setup.py (Section 2.2)
☐ Step 5: Define message types if needed (Section 3)
☐ Step 6: Add to launch file (Section 4)
☐ Step 7: Test in isolation (Section 5)
☐ Step 8: Integrate with perception (Section 6)
☐ Step 9: Monitor performance (Section 7)
☐ Step 10: Document and commit (Section 8)
```

---

## Part 1: Understanding the Current Architecture

### 1.1 The Four-Layer Stack

```
LAYER 4: Applications (Phase 2-4)
├── Speech-to-Text (future)
├── Language Model (future)
├── Navigation (future)
└── Multi-modal Fusion (future)
      ↓ listens to outputs from Layer 2-3
      
LAYER 3: Core Perception (Phase 1, ACTIVE)
├── Brightness monitoring → /r2d2/perception/brightness
├── Face detection → /r2d2/perception/face_count
├── Face recognition → /r2d2/perception/person_id
└── Image publishing → /oak/rgb/image_raw
      ↓ subscribes to Layer 1
      
LAYER 2: Communication Hub (Phase 1, minimal)
├── Heartbeat signal → /r2d2/heartbeat
└── [Future: centralized message routing]
      
LAYER 1: Hardware Interface (Phase 1, ACTIVE)
├── OAK-D Lite camera → /oak/rgb/image_raw (30 Hz)
└── [Future: IMU, motors, other sensors]
```

### 1.2 Current Data Flow (What Phase 2-4 Inherits)

```
OAK-D Camera (30 Hz)
    ↓
/oak/rgb/image_raw [sensor_msgs/Image, 1920×1080, 8-bit RGB]
    ↓
image_listener node (perception)
    ├─→ Downscale 1920×1080 → 640×360
    ├─→ Grayscale conversion
    ├─→ Brightness computation → /r2d2/perception/brightness (13 Hz)
    ├─→ Face detection (Haar) → /r2d2/perception/face_count (13 Hz)
    └─→ Face recognition (LBPH) → /r2d2/perception/person_id (6.5 Hz)

Status signals:
    /r2d2/heartbeat (1 Hz) ← heartbeat_node
```

### 1.3 Integration Points for Phase 2-4

Your Phase 2-4 nodes can hook in at these points:

| Integration Point | Topic | Message Type | Frequency | Use Case |
|------|------|------|------|------|
| Raw camera | `/oak/rgb/image_raw` | sensor_msgs/Image | 30 Hz | Real-time analysis |
| Brightness | `/r2d2/perception/brightness` | std_msgs/Float32 | 13 Hz | Lighting-aware decisions |
| Face count | `/r2d2/perception/face_count` | std_msgs/Int32 | 13 Hz | Presence detection |
| Person ID | `/r2d2/perception/person_id` | std_msgs/String | 6.5 Hz | Individual recognition |
| Confidence | `/r2d2/perception/face_confidence` | std_msgs/Float32 | 6.5 Hz | Quality metric |
| Is target | `/r2d2/perception/is_severin` | std_msgs/Bool | 6.5 Hz | Binary decision |
| Heartbeat | `/r2d2/heartbeat` | std_msgs/Bool | 1 Hz | System health |

**Key insight:** Most Phase 2-4 features will **subscribe** to perception outputs, not publish to camera.

---

## Part 2: Creating a New Package

### 2.1 Package Setup (Speech-to-Text Example)

```bash
# Navigate to workspace
cd ~/dev/r2d2/ros2_ws/src

# Create package (example: speech recognition)
ros2 pkg create \
  --build-type ament_python \
  --description "Speech-to-text processing" \
  r2d2_speech
# Creates: r2d2_speech/ with package.xml and setup.py

# Add to git immediately
cd ~/dev/r2d2
git add ros2_ws/src/r2d2_speech/
git commit -m "Add r2d2_speech package skeleton"
```

### 2.2 Essential Package Structure

```bash
# Your new package should look like this:
r2d2_speech/
├── package.xml
├── setup.py
├── resource/r2d2_speech/
│   └── (marker file, auto-created)
└── r2d2_speech/
    ├── __init__.py
    ├── speech_node.py         ← Your main node
    ├── speech_processor.py    ← Helper classes
    └── config/
        └── default_params.yaml ← Parameter defaults
```

### 2.3 Package Dependencies (package.xml)

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>r2d2_speech</name>
  <version>0.1.0</version>
  <description>Speech-to-text processing for R2D2</description>
  
  <maintainer email="you@example.com">Your Name</maintainer>
  <license>BSD-3-Clause</license>
  
  <!-- These are CRITICAL for ROS 2 Python packages -->
  <buildtool_depend>ament_cmake_python</buildtool_depend>
  <build_depend>rosidl_default_generators</build_depend>
  <exec_depend>rosidl_default_runtime</exec_depend>
  <test_depend>ament_copyright</test_depend>
  <test_depend>ament_flake8</test_depend>
  <test_depend>ament_pep257</test_depend>
  <test_depend>python3-pytest</test_depend>
  
  <!-- ROS 2 core dependencies -->
  <depend>rclpy</depend>
  <depend>std_msgs</depend>
  <depend>sensor_msgs</depend>
  
  <!-- Your specific dependencies (adjust as needed) -->
  <!-- Example for speech: -->
  <!-- <depend>pyaudio</depend> -->
  <!-- <depend>sounddevice</depend> -->
  
  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

### 2.4 Setup File (setup.py)

```python
# setup.py
from setuptools import setup
import os
from glob import glob

package_name = 'r2d2_speech'

# Find all Python packages and resource files
data_files = []
data_files.append(('share/ament_index/resource_index/packages',
                   ['resource/' + package_name]))
data_files.append((os.path.join('share', package_name), ['package.xml']))

# Include any config YAML files
data_files.append((os.path.join('share', package_name, 'config'),
                   glob(os.path.join('r2d2_speech', 'config', '*.yaml'))))

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='you@example.com',
    description='Speech-to-text processing for R2D2',
    license='BSD-3-Clause',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # Register your executable nodes here
            'speech_node = r2d2_speech.speech_node:main',
        ],
    },
)
```

---

## Part 3: Creating Message Types (Custom Data)

### 3.1 When Do You Need Custom Messages?

**Use existing ROS 2 message types (std_msgs, sensor_msgs) for:**
- ✅ Simple values: Bool, Int32, Float32, String
- ✅ Strings: std_msgs/String
- ✅ Sensor data: sensor_msgs/Image, sensor_msgs/Imu
- ✅ Collections: std_msgs/Float32MultiArray

**Create custom message types (interfaces/) for:**
- ❌ Complex structured data
- ❌ Multiple related fields (e.g., speech + confidence + timestamp)
- ❌ Nested information

### 3.2 Example: Custom Message for Speech Recognition

```bash
# If you need a custom message, create interface directory
cd ~/dev/r2d2/ros2_ws/src/r2d2_speech
mkdir -p msg

# Create SpeechResult.msg file
cat > msg/SpeechResult.msg << 'EOF'
# Detected speech with confidence
string text                    # Transcribed text
float32 confidence             # 0.0-1.0 confidence score
int32 duration_ms              # Duration of speech in milliseconds
bool is_final                  # True if final result, false if intermediate
EOF

# Update package.xml to generate interfaces
# Add to package.xml (after <depend> tags):
#   <build_depend>rosidl_default_generators</build_depend>
#   <exec_depend>rosidl_default_runtime</exec_depend>

# Update setup.py to include message generation
# (ROS 2 handles this automatically for .msg files)
```

### 3.3 Using Custom Messages in Your Node

```python
# In your node code
from r2d2_speech.msg import SpeechResult

# Publisher
self.publisher = self.create_publisher(
    SpeechResult,
    '/r2d2/speech/result',
    10
)

# Publishing
msg = SpeechResult()
msg.text = "hello world"
msg.confidence = 0.95
msg.duration_ms = 1500
msg.is_final = True
self.publisher.publish(msg)

# Subscriber
self.subscriber = self.create_subscription(
    SpeechResult,
    '/r2d2/speech/result',
    self.speech_callback,
    10
)

def speech_callback(self, msg):
    self.get_logger().info(f"Got speech: {msg.text} ({msg.confidence:.2%})")
```

---

## Part 4: Implementing Your Node

### 4.1 Template: Basic ROS 2 Python Node

```python
# r2d2_speech/speech_node.py
"""
Speech-to-text node for R2D2.

Subscribes to: /oak/rgb/image_raw (optional, for visual context)
Publishes to: /r2d2/speech/result
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image
import numpy as np

class SpeechNode(Node):
    """Speech recognition node."""
    
    def __init__(self):
        super().__init__('speech_node')
        
        # Declare and get parameters
        self.declare_parameter('model_path', '/path/to/model')
        self.declare_parameter('confidence_threshold', 0.7)
        self.declare_parameter('enable_logging', True)
        
        self.model_path = self.get_parameter('model_path').value
        self.confidence_threshold = self.get_parameter('confidence_threshold').value
        self.enable_logging = self.get_parameter('enable_logging').value
        
        # Create publisher (to send speech results)
        self.publisher = self.create_publisher(
            String,
            '/r2d2/speech/result',
            10
        )
        
        # Create subscriber (to listen to camera, optional)
        self.subscriber = self.create_subscription(
            Image,
            '/oak/rgb/image_raw',
            self.camera_callback,
            10
        )
        
        # Timer for periodic processing (e.g., every 100ms)
        self.timer = self.create_timer(0.1, self.timer_callback)
        
        self.get_logger().info('Speech node initialized')
        
    def camera_callback(self, msg: Image):
        """Called when new camera frame arrives."""
        if self.enable_logging:
            self.get_logger().debug(f"Received image: {msg.width}x{msg.height}")
        
        # Process image if needed
        # (convert to numpy, process, etc.)
        
    def timer_callback(self):
        """Called periodically to process speech."""
        # Your speech processing logic here
        
        # Publish result when ready
        result_msg = String()
        result_msg.data = "detected speech text"
        self.publisher.publish(result_msg)
        

def main(args=None):
    """Entry point for ROS 2 node."""
    rclpy.init(args=args)
    
    node = SpeechNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### 4.2 Template: Subscribe to Perception Data

```python
# Common pattern: react to face detection/recognition

from std_msgs.msg import Int32, String, Float32

class PerceptionListenerNode(Node):
    """Node that responds to perception events."""
    
    def __init__(self):
        super().__init__('perception_listener')
        
        # Subscribe to perception topics
        self.create_subscription(
            Int32,
            '/r2d2/perception/face_count',
            self.face_count_callback,
            10
        )
        
        self.create_subscription(
            String,
            '/r2d2/perception/person_id',
            self.person_id_callback,
            10
        )
        
        self.create_subscription(
            Float32,
            '/r2d2/perception/brightness',
            self.brightness_callback,
            10
        )
        
        self.get_logger().info('Perception listener ready')
    
    def face_count_callback(self, msg: Int32):
        """React to face detection."""
        if msg.data > 0:
            self.get_logger().info(f'Detected {msg.data} face(s)')
            # Trigger response: greet, track, etc.
        
    def person_id_callback(self, msg: String):
        """React to person identification."""
        self.get_logger().info(f'Identified: {msg.data}')
        # Personalized response here
        
    def brightness_callback(self, msg: Float32):
        """React to brightness changes."""
        if msg.data < 100:
            self.get_logger().warning('Low light detected')
            # Adjust processing, request lighting, etc.
```

### 4.3 Best Practices for Phase 2-4 Nodes

```python
# ✅ DO:
# - Use meaningful node names
node = SpeechNode()  # Clear purpose

# - Parameterize everything
self.declare_parameter('confidence_threshold', 0.7)

# - Use logging appropriately
self.get_logger().info('Starting processing')      # Major events
self.get_logger().debug('Processing frame 42')     # Detailed debug

# - Handle shutdown gracefully
try:
    rclpy.spin(node)
finally:
    node.destroy_node()
    rclpy.shutdown()

# - Subscribe to perception for context awareness
self.create_subscription(Int32, '/r2d2/perception/face_count', ...)

# ❌ DON'T:
# - Don't hardcode paths
model_path = "/home/user/model.pkl"  # BAD

# - Don't publish raw data without context
self.publisher.publish(confidence_value)  # Lost semantics

# - Don't ignore errors
try:
    process_audio()
except:
    pass  # BAD - errors hidden

# - Don't ignore parameters
# Always declare parameters, even if using defaults
```

---

## Part 5: Testing Your Node in Isolation

### 5.1 Unit Test Template

```python
# tests/test_speech_node.py

import unittest
import rclpy
from r2d2_speech.speech_node import SpeechNode


class TestSpeechNode(unittest.TestCase):
    
    @classmethod
    def setUpClass(cls):
        """Initialize ROS 2 once for all tests."""
        rclpy.init()
    
    @classmethod
    def tearDownClass(cls):
        """Clean up ROS 2 after all tests."""
        rclpy.shutdown()
    
    def setUp(self):
        """Create node before each test."""
        self.node = SpeechNode()
    
    def tearDown(self):
        """Clean up node after each test."""
        self.node.destroy_node()
    
    def test_node_initialization(self):
        """Test that node initializes without error."""
        self.assertIsNotNone(self.node)
        self.assertEqual(self.node.get_name(), 'speech_node')
    
    def test_parameters_loaded(self):
        """Test that parameters are loaded correctly."""
        param = self.node.get_parameter('confidence_threshold')
        self.assertIsNotNone(param.value)
    
    def test_publisher_created(self):
        """Test that publisher is created."""
        self.assertIsNotNone(self.node.publisher)


if __name__ == '__main__':
    unittest.main()
```

### 5.2 Manual Testing (Before Integration)

```bash
# Terminal 1: Keep perception running
source ~/depthai_env/bin/activate
export OPENBLAS_CORETYPE=ARMV8
source ~/.bashrc
cd ~/dev/r2d2/ros2_ws && source install/setup.bash
ros2 launch r2d2_bringup r2d2_camera_perception.launch.py

# Terminal 2: Build and test your new package
source ~/depthai_env/bin/activate
export OPENBLAS_CORETYPE=ARMV8
source ~/.bashrc
cd ~/dev/r2d2/ros2_ws
colcon build --packages-select r2d2_speech
source install/setup.bash

# Run your node standalone
ros2 run r2d2_speech speech_node

# Terminal 3: Monitor your outputs
ros2 topic echo /r2d2/speech/result

# Terminal 4: Check it's not killing CPU
watch -n 1 'top -bn1 | grep python3'
```

---

## Part 6: Integrating with Launch File

### 6.1 Adding Your Node to Existing Launch

```python
# ros2_ws/src/r2d2_bringup/launch/r2d2_camera_perception.launch.py

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
import os

def generate_launch_description():
    # Existing launch configuration...
    
    # NEW: Add your speech node
    speech_node = Node(
        package='r2d2_speech',
        executable='speech_node',
        name='speech_node',
        output='screen',
        parameters=[
            {
                'model_path': LaunchConfiguration('speech_model_path'),
                'confidence_threshold': LaunchConfiguration('speech_confidence'),
                'enable_logging': LaunchConfiguration('enable_speech_logging'),
            }
        ],
        remappings=[
            # If needed, remap topics
            # ('/input_topic', '/remapped_topic'),
        ]
    )
    
    # Register launch configuration (add to LaunchDescription arguments)
    launch_args = [
        # ... existing args ...
        
        # NEW: Add your node's launch arguments
        DeclareLaunchArgument(
            'speech_model_path',
            default_value=os.path.expanduser('~/dev/r2d2/data/speech_model.pkl'),
            description='Path to speech recognition model'
        ),
        DeclareLaunchArgument(
            'speech_confidence',
            default_value='0.7',
            description='Confidence threshold for speech recognition'
        ),
        DeclareLaunchArgument(
            'enable_speech_logging',
            default_value='true',
            description='Enable detailed speech logging'
        ),
    ]
    
    return LaunchDescription(launch_args + [
        # ... existing nodes ...
        speech_node,  # NEW: include your node
    ])
```

### 6.2 Launching with Your New Package

```bash
# Terminal 1: Launch with speech node
ros2 launch r2d2_bringup r2d2_camera_perception.launch.py \
  enable_speech_logging:=true \
  speech_confidence:=0.8

# Terminal 2: Verify all nodes running
ros2 node list
# Should show:
# /camera_node
# /image_listener
# /heartbeat_node
# /speech_node (NEW)

# Terminal 2: Check all topics
ros2 topic list
# Should include:
# /r2d2/speech/result (NEW)
```

---

## Part 7: Performance Monitoring

### 7.1 CPU Budget for Phase 2-4

```
JETSON AGX ORIN CAPACITY (12-core ARM):
═══════════════════════════════════════════════════════════

Baseline (Phase 1 only):
├── camera_node: 2-3% CPU
├── image_listener: 8-15% CPU (varies with face recognition)
├── heartbeat_node: <0.1% CPU
└── Total allocated: ~15% CPU

Available for Phase 2-4:
├── **Without** face recognition: ~85% CPU available
└── **With** face recognition: ~70% CPU available

RECOMMENDATION for Phase 2:
├── Speech-to-text: 20-30% CPU typical
├── LLM processing: 30-50% CPU typical  
└── Route to: Turn OFF face recognition when speech active
    (or run serial, not parallel)
```

### 7.2 Monitoring During Development

```bash
# Real-time CPU check
watch -n 1 'top -bn1 | grep -E "python3|Cpu" | head -15'

# Per-node monitoring
ps aux | grep "python3" | awk '{print $2, $3, $11}'
# Shows: PID CPU% COMMAND

# Jetson-specific (best for ARM monitoring)
tegrastats
# Shows CPU, GPU, memory, thermal, power draw
# Press Ctrl+C to exit

# Memory growth detection (memory leak check)
ps aux | grep python3 | sort -k6 -rn | head -5
# Shows process and memory usage (VSZ column)

# Topic frequency monitoring
for topic in /r2d2/perception/brightness /r2d2/speech/result; do
  echo "=== $topic ===" 
  timeout 5 ros2 topic hz $topic 2>/dev/null || echo "Not publishing"
done
```

### 7.3 Setting CPU Limits (Prevent Runaway Processes)

```bash
# Limit a Python process to 50% CPU
# Terminal 1: Launch your node
python3 my_node.py &
NODE_PID=$!

# Terminal 2: Limit CPU
cpulimit -p $NODE_PID -l 50

# Or in code (soft limit):
import resource
soft, hard = resource.getrlimit(resource.RLIMIT_CPU)
resource.setrlimit(resource.RLIMIT_CPU, (300, hard))  # 5 min timeout
```

---

## Part 8: Documentation & Committing

### 8.1 Document Your Node (README)

```markdown
# R2D2 Speech Module

## Overview
Speech-to-text processing for R2D2. Converts audio input to text with confidence scores.

## Topics

### Published
- `/r2d2/speech/result` (std_msgs/String)
  - Raw detected speech text
  - Frequency: ~1-5 Hz (event-driven)

### Subscribed
- `/oak/rgb/image_raw` (sensor_msgs/Image, optional)
  - Used for lip-sync or visual context
  - Frequency: 30 Hz

## Parameters

- `model_path` (string): Path to speech model
- `confidence_threshold` (float, 0.0-1.0): Minimum confidence to publish
- `enable_logging` (bool): Verbose logging for debugging

## Startup

```bash
ros2 run r2d2_speech speech_node --ros-args -p confidence_threshold:=0.8
```

## Performance

- CPU: ~25% single core
- Latency: 500-1000ms (model dependent)
- Memory: ~200-500 MB

## Troubleshooting

If no results: check that model file exists at `model_path`
If CPU high: disable logging, increase batch size
```

### 8.2 Commit to Git

```bash
# Add all new files
cd ~/dev/r2d2
git add ros2_ws/src/r2d2_speech/

# Commit with clear message
git commit -m "Add speech-to-text node (Phase 2)

- Implements speech recognition using [model name]
- Publishes to /r2d2/speech/result
- Subscribes to perception topics for context
- Includes tests and documentation
- CPU: ~25%, Memory: ~300 MB

Phase: 2 (Speech integration)
Issue: #45 (if tracking)"

# Push when ready (after integration testing)
# git push origin develop
```

---

## Part 9: Common Integration Patterns

### 9.1 Face Detection → Speech Response

```python
# Full example: greet when face detected

from rclpy.node import Node
from std_msgs.msg import Int32
from geometry_msgs.msg import Point

class GreeterNode(Node):
    def __init__(self):
        super().__init__('greeter')
        
        self.create_subscription(
            Int32,
            '/r2d2/perception/face_count',
            self.face_callback,
            10
        )
        
        # Publisher for speech command
        self.speech_pub = self.create_publisher(String, '/r2d2/speech/say', 10)
        
        self.last_face_count = 0
    
    def face_callback(self, msg: Int32):
        # Trigger greeting when new face appears
        if msg.data > self.last_face_count:
            self.get_logger().info(f'New face detected! ({msg.data} total)')
            
            greeting = String()
            greeting.data = "Hello there! Nice to see you."
            self.speech_pub.publish(greeting)
        
        self.last_face_count = msg.data
```

### 9.2 Brightness-Aware Processing

```python
# Skip heavy processing in low light

from std_msgs.msg import Float32

class LightingAwareNode(Node):
    BRIGHT_THRESHOLD = 120
    DARK_THRESHOLD = 80
    
    def __init__(self):
        super().__init__('lighting_aware')
        
        self.create_subscription(
            Float32,
            '/r2d2/perception/brightness',
            self.brightness_callback,
            10
        )
        
        self.current_brightness = 130
        self.processing_enabled = True
    
    def brightness_callback(self, msg: Float32):
        self.current_brightness = msg.data
        
        if msg.data < self.DARK_THRESHOLD:
            self.processing_enabled = False
            self.get_logger().warn('Low light: disabling heavy processing')
            # Skip resource-intensive tasks
        
        elif msg.data > self.BRIGHT_THRESHOLD:
            self.processing_enabled = True
            self.get_logger().info('Good lighting: processing enabled')
```

### 9.3 Multi-Modal Fusion Example

```python
# Combine vision + speech for decisions

from std_msgs.msg import String, Int32
from sensor_msgs.msg import Image

class MultiModalNode(Node):
    def __init__(self):
        super().__init__('multimodal_fusion')
        
        # Vision input
        self.create_subscription(String, '/r2d2/perception/person_id',
                                self.vision_callback, 10)
        
        # Audio input
        self.create_subscription(String, '/r2d2/speech/detected_text',
                                self.speech_callback, 10)
        
        self.last_person = None
        self.last_speech = None
    
    def vision_callback(self, msg: String):
        self.last_person = msg.data
        self.try_fuse()
    
    def speech_callback(self, msg: String):
        self.last_speech = msg.data
        self.try_fuse()
    
    def try_fuse(self):
        """Combine vision and speech for decisions."""
        if self.last_person and self.last_speech:
            self.get_logger().info(
                f'Person ({self.last_person}) said: "{self.last_speech}"'
            )
            
            # Make context-aware decision
            if self.last_person == "severin" and "hello" in self.last_speech.lower():
                self.respond_with_context()
            
            # Reset
            self.last_person = None
            self.last_speech = None
    
    def respond_with_context(self):
        """Personalized response."""
        self.get_logger().info('Context: Severin greeting detected')
```

---

## Part 10: Troubleshooting Integration

### 10.1 Node Won't Start

```
Error: "No module named 'r2d2_speech'"

Fix:
1. Rebuild workspace: cd ~/dev/r2d2/ros2_ws && colcon build
2. Source setup: source install/setup.bash
3. Verify package: ros2 pkg list | grep r2d2
4. Check package.xml syntax: xmllint ros2_ws/src/r2d2_speech/package.xml
```

### 10.2 Dependencies Missing

```
Error: "Cannot import module X"

Fix:
1. Add to package.xml:
   <depend>python_module_name</depend>

2. Install in environment:
   pip install python-module-name
   (while depthai_env is activated)

3. Rebuild:
   colcon build --packages-select r2d2_speech
```

### 10.3 Topics Not Publishing

```
Node runs but publishes nothing.

Debug steps:
1. Add logging: self.get_logger().info("About to publish")
2. Check topic exists: ros2 topic list | grep your_topic
3. Echo the topic: ros2 topic echo /r2d2/speech/result
4. Check node parameters: ros2 param list /speech_node
```

### 10.4 CPU Usage Spike

```
Node consuming >30% CPU unexpectedly.

Investigate:
1. Check if loop is running: add timestamps to logs
2. Reduce publish frequency (add delays)
3. Disable logging: self.get_logger().disabled = True
4. Profile CPU: top -p $(pgrep -f speech_node)

Common cause: infinite loop without sleep/timer
```

---

## Part 11: Reference: Launch Configuration

### 11.1 Full Example Launch File

```python
# r2d2_camera_perception_v2.launch.py (with Phase 2 nodes)

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
import os

def generate_launch_description():
    """Generate full launch description for camera + perception + speech."""
    
    # Phase 1: Camera & Perception nodes
    camera_node = Node(
        package='r2d2_camera',
        executable='camera_node',
        name='camera_node',
        output='screen',
    )
    
    perception_node = Node(
        package='r2d2_perception',
        executable='image_listener',
        name='image_listener',
        output='screen',
        parameters=[{
            'enable_face_recognition': LaunchConfiguration('enable_face_recognition'),
            'recognition_frame_skip': LaunchConfiguration('recognition_frame_skip'),
        }],
    )
    
    heartbeat_node = Node(
        package='r2d2_hello',
        executable='heartbeat_node',
        name='heartbeat_node',
        output='screen',
    )
    
    # Phase 2: NEW Speech node
    speech_node = Node(
        package='r2d2_speech',
        executable='speech_node',
        name='speech_node',
        output='screen',
        parameters=[{
            'model_path': LaunchConfiguration('speech_model_path'),
            'confidence_threshold': LaunchConfiguration('speech_confidence'),
        }],
    )
    
    # Launch arguments (parameters)
    launch_args = [
        DeclareLaunchArgument(
            'enable_face_recognition',
            default_value='true',
            description='Enable face recognition'
        ),
        DeclareLaunchArgument(
            'recognition_frame_skip',
            default_value='2',
            description='Process every Nth frame for recognition'
        ),
        DeclareLaunchArgument(
            'speech_model_path',
            default_value=os.path.expanduser('~/models/speech.pkl'),
            description='Path to speech model'
        ),
        DeclareLaunchArgument(
            'speech_confidence',
            default_value='0.7',
            description='Speech confidence threshold'
        ),
    ]
    
    return LaunchDescription(launch_args + [
        camera_node,
        perception_node,
        heartbeat_node,
        speech_node,  # NEW
    ])
```

---

## Summary Checklist

For any new Phase 2-4 feature:

1. ✅ Understand architecture (how it fits)
2. ✅ Create ROS 2 package (proper structure)
3. ✅ Implement node (template provided)
4. ✅ Test in isolation (manual + unit tests)
5. ✅ Add to launch file (parameter-driven)
6. ✅ Monitor performance (CPU/memory)
7. ✅ Integrate with perception (subscribe to topics)
8. ✅ Document (README + comments)
9. ✅ Commit to git (with clear messages)

**Expected timeline per node:** 2-4 hours (implement + test + integrate)

---

*Integration Guide created: December 7, 2025*
