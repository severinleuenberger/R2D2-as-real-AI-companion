# R2D2 Wheel Motors & Differential Drive System Reference

**Date:** January 6, 2026  
**Status:** Specification Complete - Awaiting Phase 3 Implementation  
**Hardware:** 2× DeAgostini DC Motors + 2× Pololu G2 24v21 Drivers  
**Author:** AI Agent (Claude)

---

## Executive Summary

R2D2's locomotion system uses two DeAgostini DC motors with Hall effect encoders for differential drive control. Each motor is controlled by a Pololu G2 24v21 high-power motor driver, providing precise PWM speed control and quadrature encoder feedback for odometry.

**Key Components:**
- 2× DeAgostini DC motors with integrated Hall encoders (from R2D2 kit)
- 2× Pololu G2 24v21 motor drivers (assembled, not yet wired)
- Differential drive kinematics for forward/reverse/turn movements
- Encoder-based odometry for position tracking

**Current Status:** Hardware available, pin allocation finalized, awaiting Phase 3 implementation

---

## Table of Contents

1. [Hardware Specifications](#2-hardware-specifications)
2. [Electrical Connections](#3-electrical-connections)
3. [Differential Drive Kinematics](#4-differential-drive-kinematics)
4. [ROS 2 Integration](#5-ros-2-integration)
5. [Software Implementation](#6-software-implementation)
6. [Calibration Procedures](#7-calibration-procedures)
7. [Testing & Validation](#8-testing--validation)
8. [Safety Considerations](#9-safety-considerations)
9. [Cross-References](#10-cross-references)

---

## 2. Hardware Specifications

### 2.1 DeAgostini Wheel Motors

- **Model:** DeAgostini DC Motor (from R2D2 1:2 scale kit)
- **Quantity:** 2 (left wheel + right wheel)
- **Voltage:** 12-14.8V nominal (4S LiPo compatible)
- **Type:** Brushed DC motor with integrated gearbox
- **Encoder:** Hall effect, quadrature output (A/B channels)
- **Mounting:** Integrated into DeAgostini chassis leg assemblies

**Specifications to Measure:**
- [ ] Stall current (A) - measure with motor locked
- [ ] No-load current (A) - measure spinning freely
- [ ] RPM at 14.8V - measure with tachometer
- [ ] Gear ratio - count motor shaft vs wheel rotations
- [ ] Encoder pulses per revolution (PPR) - count pulses per wheel rotation
- [ ] Wheel diameter (mm) - measure with caliper
- [ ] Wheelbase (mm) - distance between left/right wheel centers

### 2.2 Pololu G2 24v21 Motor Drivers

- **Model:** Pololu G2 High-Power Motor Driver 24v21
- **Part Number:** [2995](https://www.pololu.com/product/2995)
- **Quantity:** 2 (one per wheel)
- **Voltage Range:** 6.5V - 40V (14.8V LiPo ✅ compatible)
- **Continuous Current:** 21A per driver (no heatsink required)
- **Peak Current:** 50A (adjustable threshold with current limiting)
- **Logic Compatibility:** 1.8V, 3.3V, 5V (Jetson 3.3V GPIO ✅ compatible)
- **Control Modes:**
  - Sign-magnitude: DIR (direction) + PWM (speed) ✅ Recommended
  - Locked-antiphase: Single PWM input
- **Protection Features:**
  - Reverse voltage protection
  - Undervoltage shutdown
  - Short circuit protection
  - Over-current limiting (chopping at 50A default)
- **Current Sense:** ~20mV/A analog output (CS pin for monitoring)
- **Board Size:** 1.3″ × 0.8″ (33mm × 20mm)

**Status:** ✅ Assembled, awaiting Phase 3 wiring

**Important Notes:**
- No over-temperature protection (monitor driver temperature manually)
- PWM frequency: Up to 100kHz (10-20kHz recommended for DC motors)
- Minimum I/O lines: 2 per driver (PWM + DIR) or 3 if using separate enable

### 2.3 Hall Effect Encoders

**Type:** Quadrature encoder (A/B channels, 90° phase shift)

**Connector:** 6-pin (typical configuration):
- Pin 1: VCC (5V)
- Pin 2: GND
- Pin 3: Channel A output
- Pin 4: Channel B output
- Pin 5: Index (optional - may not be present on DeAgostini)
- Pin 6: NC (not connected)

**Output Type:** Open collector (requires pull-up resistors)
- Pull-up voltage: 3.3V or 5V (Jetson 3.3V GPIO safe)
- Pull-up resistor: 4.7kΩ - 10kΩ typical
- May be built into encoder module or need external

**Resolution:** [TO BE MEASURED] pulses per motor shaft revolution
- With quadrature decoding (4× mode): base_resolution × 4
- After gearbox: PPR × gear_ratio
- Example: 12 PPR × 100:1 gear × 4× mode = 4800 pulses per wheel revolution

**Quadrature Decoding:**
- A and B channels 90° out of phase
- Forward rotation: A leads B
- Reverse rotation: B leads A
- 4× decoding: Interrupt on both rising and falling edges of both channels

---

## 3. Electrical Connections

**⚠️ CRITICAL UPDATE (January 6, 2026):**

The GPIO wiring shown below was the original plan but is **NOT VIABLE** due to GPIO output limitations discovered during pan motor testing. Most Jetson AGX Orin GPIO pins cannot output voltage due to pinmux configuration in JetPack R36.4.7.

**Recommended Approach:** Use **PCA9685 I2C PWM controller** (same as pan motor) to control both wheel motors:
- Left wheel: PCA9685 CH3 (PWM), CH4 (DIR)
- Right wheel: PCA9685 CH5 (PWM), CH6 (DIR)
- Only uses 2 I2C pins (Pins 3 & 5) instead of 10 GPIO pins

**See:** `999_Next_Task_Camera_Pan_Tilt_final.md` Section 5.1 for complete PCA9685 wiring details and channel allocation.

**The wiring diagrams below are preserved for reference but should be adapted to use PCA9685 outputs instead of direct GPIO.**

---

### 3.1 Left Wheel Motor (DEPRECATED - Use PCA9685 Instead)

**⚠️ Direct GPIO wiring shown below does NOT work - use PCA9685**

**Pin Allocation** (from [`002_HARDWARE_REFERENCE.md`](002_HARDWARE_REFERENCE.md) Section 4.5):

| Function | Pin | GPIO | Pololu Driver Pin | Notes |
|----------|-----|------|-------------------|-------|
| PWM Speed | 15 | GPIO22 | PWMA | Hardware PWM |
| Direction 1 | 35 | GPIO19 | AIN1 | Forward/reverse |
| Direction 2 | 37 | GPIO26 | AIN2 | H-bridge control |
| Encoder A | 19 | GPIO10 | — | From motor encoder |
| Encoder B | 21 | GPIO9 | — | From motor encoder |
| Common GND | 6/9/14/20/25/30/34/39 | GND | GND | Signal ground |

**Pololu Driver #1 Wiring:**
```
Pololu G2 Driver #1      Jetson + Battery
═══════════════════      ═════════════════════════════
VM    ───────────────►   14.8V Battery + (via 10A fuse)
GND   ───────────────►   Battery GND (common with Jetson GND)
VCC   ───────────────►   Pin 1/17 (3.3V logic power)
PWMA  ───────────────►   Pin 15 (GPIO22)
AIN1  ───────────────►   Pin 35 (GPIO19)
AIN2  ───────────────►   Pin 37 (GPIO26)
OUTA  ───────────────►   Left motor terminal +
OUTB  ───────────────►   Left motor terminal -
```

**Left Wheel Encoder Wiring:**
```
Left Encoder            Jetson AGX Orin
═══════════════         ═════════════════════════════
VCC (Pin 1, red)  ───►  Pin 2/4 (5V)
GND (Pin 2, black)───►  Pin 6/9/14/20/25/30/34/39 (GND)
Ch A (Pin 3)      ───►  Pin 19 (GPIO10) + 4.7kΩ pull-up to 3.3V
Ch B (Pin 4)      ───►  Pin 21 (GPIO9) + 4.7kΩ pull-up to 3.3V
```

### 3.2 Right Wheel Motor (DEPRECATED - Use PCA9685 Instead)

**⚠️ Direct GPIO wiring shown below does NOT work - use PCA9685**

**Pin Allocation:**

| Function | Pin | GPIO | Pololu Driver Pin | Notes |
|----------|-----|------|-------------------|-------|
| PWM Speed | 11 | GPIO17 | PWMA | Software PWM OK |
| Direction 1 | 36 | GPIO16 | AIN1 | Forward/reverse |
| Direction 2 | 38 | GPIO20 | AIN2 | H-bridge control |
| Encoder A | 24 | GPIO8 | — | From motor encoder |
| Encoder B | 26 | GPIO7 | — | From motor encoder |
| Common GND | 6/9/14/20/25/30/34/39 | GND | GND | Signal ground |

**Pololu Driver #2 Wiring:**
```
Pololu G2 Driver #2      Jetson + Battery
═══════════════════      ═════════════════════════════
VM    ───────────────►   14.8V Battery + (via 10A fuse)
GND   ───────────────►   Battery GND (common with Jetson GND)
VCC   ───────────────►   Pin 1/17 (3.3V logic power)
PWMA  ───────────────►   Pin 11 (GPIO17)
AIN1  ───────────────►   Pin 36 (GPIO16)
AIN2  ───────────────►   Pin 38 (GPIO20)
OUTA  ───────────────►   Right motor terminal +
OUTB  ───────────────►   Right motor terminal -
```

**Right Wheel Encoder Wiring:**
```
Right Encoder           Jetson AGX Orin
═══════════════         ═════════════════════════════
VCC (Pin 1, red)  ───►  Pin 2/4 (5V)
GND (Pin 2, black)───►  Pin 6/9/14/20/25/30/34/39 (GND)
Ch A (Pin 3)      ───►  Pin 24 (GPIO8) + 4.7kΩ pull-up to 3.3V
Ch B (Pin 4)      ───►  Pin 26 (GPIO7) + 4.7kΩ pull-up to 3.3V
```

### 3.3 Power Distribution

**Motor Power (14.8V Rail):**
```
3× Turnigy 4S LiPo (14.8V, parallel) → Power Distribution Board
    ├─► 10A Fuse → Pololu Driver #1 VM (left motor)
    └─► 10A Fuse → Pololu Driver #2 VM (right motor)
```

**Logic Power (3.3V):**
- Pololu VCC: Pin 1 or 17 (3.3V from Jetson) - logic power only
- Encoder VCC: Pin 2 or 4 (5V from Jetson) - encoder power

**Critical Requirements:**
- ✅ Battery GND and Jetson GND MUST be common (shared ground plane)
- ✅ Motor power (VM) from battery, NOT from Jetson
- ✅ Logic power (VCC) from Jetson 3.3V (low current, signals only)
- ✅ Separate fuses for each motor driver (10A recommended)

---

## 4. Differential Drive Kinematics

### 4.1 Wheel Configuration

**Parameters to Measure:**
- `L` (wheelbase): Distance between left and right wheel centers (meters)
- `D` (wheel diameter): Diameter of driven wheels (meters)
- `C` (wheel circumference): π × D (meters)
- `PPR` (pulses per revolution): Encoder pulses per wheel revolution (in 4× mode)

**Example R2D2 DeAgostini Measurements (TO BE VERIFIED):**
- Wheelbase (L): ~0.15m (estimated, measure actual)
- Wheel diameter (D): ~0.05m (estimated, measure actual)
- Gear ratio: ~100:1 (typical for DeAgostini)

### 4.2 Motion Control Equations

**Forward/Reverse (straight line):**
```python
left_speed = target_linear_velocity
right_speed = target_linear_velocity
```

**Turn Left (arc or rotate):**
```python
left_speed = target_linear_velocity - (angular_velocity * L / 2)
right_speed = target_linear_velocity + (angular_velocity * L / 2)
```

**Turn Right (arc or rotate):**
```python
left_speed = target_linear_velocity + (angular_velocity * L / 2)
right_speed = target_linear_velocity - (angular_velocity * L / 2)
```

**Spin in Place (rotate without translation):**
```python
# Set linear velocity to 0
left_speed = -(angular_velocity * L / 2)
right_speed = +(angular_velocity * L / 2)
```

**From ROS Twist to Wheel Speeds:**
```python
# Input: geometry_msgs/Twist
#   linear.x: forward/backward velocity (m/s)
#   angular.z: rotation rate (rad/s)

# Output: left and right wheel velocities (m/s)
left_velocity = linear.x - (angular.z * wheelbase / 2)
right_velocity = linear.x + (angular.z * wheelbase / 2)
```

### 4.3 Odometry Calculation

**From Encoder Counts to Wheel Velocities:**
```python
# Constants
pulses_per_meter = encoder_ppr / wheel_circumference

# Measure encoder pulse deltas since last update
left_pulses_delta = left_encoder_count - last_left_count
right_pulses_delta = right_encoder_count - last_right_count

# Calculate wheel velocities (m/s)
dt = time_since_last_update  # seconds
left_velocity = (left_pulses_delta / pulses_per_meter) / dt
right_velocity = (right_pulses_delta / pulses_per_meter) / dt
```

**From Wheel Velocities to Robot Velocities:**
```python
# Robot linear velocity (forward/backward, m/s)
linear_velocity = (left_velocity + right_velocity) / 2

# Robot angular velocity (rotation, rad/s)
angular_velocity = (right_velocity - left_velocity) / wheelbase
```

**Position Integration (Dead Reckoning):**
```python
# Update heading (orientation)
heading += angular_velocity * dt

# Update position (x, y in meters)
position_x += linear_velocity * cos(heading) * dt
position_y += linear_velocity * sin(heading) * dt
```

**Odometry Covariance:**
- Odometry accumulates error over time (wheel slip, encoder noise)
- Typical covariance increases with distance traveled
- Requires periodic re-calibration (e.g., visual SLAM, fiducial markers)

---

## 5. ROS 2 Integration

### 5.1 Topics

**Command Topics (inputs):**
- `/r2d2/cmd_vel` (geometry_msgs/Twist) - Velocity commands from navigation/teleop
  - `linear.x`: Forward/backward speed (m/s)
  - `angular.z`: Rotation rate (rad/s)

**Feedback Topics (outputs):**
- `/r2d2/odom` (nav_msgs/Odometry) - Odometry pose estimate and velocities
  - `pose.pose`: Estimated position (x, y, z) and orientation (quaternion)
  - `twist.twist`: Current linear and angular velocities
  - `pose.covariance`: Position uncertainty
  - `twist.covariance`: Velocity uncertainty
- `/r2d2/wheel/left/speed` (std_msgs/Float32) - Left wheel velocity (m/s)
- `/r2d2/wheel/right/speed` (std_msgs/Float32) - Right wheel velocity (m/s)
- `/r2d2/wheel/left/current` (std_msgs/Float32) - Left motor current (A)
- `/r2d2/wheel/right/current` (std_msgs/Float32) - Right motor current (A)

**TF Transforms:**
- `odom` → `base_link`: Published by wheel odometry
- `map` → `odom`: Published by SLAM/localization (Phase 4+)

### 5.2 ROS 2 Package Structure

**Package Name:** `r2d2_wheels` (or integrate into existing `r2d2_head_control`)

```
r2d2_wheels/
├── config/
│   └── wheel_params.yaml              # Configuration parameters
├── launch/
│   └── wheels.launch.py               # Launch file
├── r2d2_wheels/
│   ├── __init__.py
│   ├── wheel_motor_driver.py          # Motor control + encoder reading
│   └── wheel_controller_node.py       # ROS 2 node (cmd_vel → motors)
├── package.xml
├── setup.py
└── setup.cfg
```

### 5.3 Node: wheel_controller_node

**Responsibilities:**
- Subscribe to `/r2d2/cmd_vel`
- Convert Twist (linear.x, angular.z) to left/right wheel speeds
- Control motors via PWM + direction pins
- Read encoders (interrupt-based quadrature decoding)
- Compute odometry from encoder feedback
- Publish `/r2d2/odom` with TF transform
- Publish wheel speeds and motor current
- Safety: Emergency stop, current limiting, timeout protection

**Parameters:**
- `wheelbase`: Distance between wheels (m)
- `wheel_diameter`: Wheel diameter (m)
- `encoder_ppr`: Pulses per revolution (4× mode)
- `max_linear_velocity`: Maximum forward speed (m/s)
- `max_angular_velocity`: Maximum rotation rate (rad/s)
- `motor_timeout`: Cmd_vel timeout before auto-stop (seconds)

---

## 6. Software Implementation

### 6.1 Wheel Motor Driver Class

**File:** `r2d2_wheels/wheel_motor_driver.py`

Similar structure to `pan_motor_driver.py`:

```python
class WheelMotorDriver:
    """DC motor driver with quadrature encoder for differential drive."""
    
    def __init__(self, pwm_pin, dir1_pin, dir2_pin, 
                 encoder_a_pin, encoder_b_pin, ppr, wheel_diameter):
        # GPIO setup (BCM mode)
        # PWM setup (10-20 kHz)
        # Encoder interrupt handlers (both edges, A and B)
        # Position/velocity tracking
        
    def set_speed(self, speed: float):
        """Set motor speed: -1.0 (full reverse) to +1.0 (full forward)"""
        # Clamp speed
        # Set direction (AIN1/AIN2)
        # Set PWM duty cycle
        
    def get_velocity(self) -> float:
        """Get wheel velocity in m/s (from encoder)"""
        # Calculate from encoder pulse rate
        
    def get_distance(self) -> float:
        """Get distance traveled since last reset (meters)"""
        # Calculate from total encoder pulses
        
    def reset_encoder(self):
        """Reset encoder count to zero"""
```

### 6.2 Encoder Reading (Quadrature Decoding)

**Interrupt-based 4× resolution decoding:**

```python
import Jetson.GPIO as GPIO
from threading import Lock

class EncoderDecoder:
    def __init__(self, pin_a, pin_b):
        self.pin_a = pin_a
        self.pin_b = pin_b
        self._count = 0
        self._lock = Lock()
        
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(pin_a, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(pin_b, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        
        # Interrupt on both edges for 4× resolution
        GPIO.add_event_detect(pin_a, GPIO.BOTH, callback=self._callback_a)
        GPIO.add_event_detect(pin_b, GPIO.BOTH, callback=self._callback_b)
    
    def _callback_a(self, channel):
        a_state = GPIO.input(self.pin_a)
        b_state = GPIO.input(self.pin_b)
        
        with self._lock:
            if a_state == b_state:
                self._count += 1  # Forward
            else:
                self._count -= 1  # Reverse
    
    def _callback_b(self, channel):
        a_state = GPIO.input(self.pin_a)
        b_state = GPIO.input(self.pin_b)
        
        with self._lock:
            if a_state != b_state:
                self._count += 1  # Forward
            else:
                self._count -= 1  # Reverse
    
    def get_count(self) -> int:
        with self._lock:
            return self._count
```

---

## 7. Calibration Procedures

### 7.1 Encoder Resolution Measurement

**Procedure:**
1. Elevate robot so wheels don't touch ground
2. Mark starting position on wheel (tape mark)
3. Rotate wheel manually exactly 10 complete revolutions
4. Read encoder pulse count from software
5. Calculate: `PPR = total_pulses / 10`
6. Repeat 3 times, average results

**Expected Result:** PPR in range 100-10000 depending on gear ratio and encoder resolution

### 7.2 Wheel Diameter Measurement

**Procedure:**
1. Mark wheel at ground contact point (chalk or tape)
2. Roll robot forward in straight line for exactly 10 wheel rotations
3. Measure distance traveled with tape measure (meters)
4. Calculate: `diameter = distance / (10 × π)`

**Alternative method:**
- Measure wheel diameter directly with caliper (less accurate due to tire deformation)

### 7.3 Wheelbase Measurement

**Procedure:**
1. Measure distance between left and right wheel contact patches (centers)
2. Use caliper or ruler for precision
3. Critical for accurate turning calculations

**Verification:**
- Command robot to spin 360° in place
- Measure actual rotation angle
- If error > 5°, adjust wheelbase parameter

### 7.4 Odometry Accuracy Test

**Straight Line Test:**
1. Place robot at starting position, mark ground
2. Command: linear.x = 0.3 m/s, angular.z = 0, duration = 10s
3. Measure actual distance traveled vs expected (3.0m)
4. Calculate error percentage

**Spin Test:**
1. Place robot at starting position, mark orientation
2. Command: linear.x = 0, angular.z = π/4 rad/s, duration = 8s
3. Expected: 360° rotation (2π radians)
4. Measure actual rotation angle
5. Adjust wheelbase if error > 5%

---

## 8. Testing & Validation

### 8.1 Motor Driver Bench Test

**Test Script:** `scripts/test/test_wheel_motors.py`

**Tests:**
1. Forward motion at 25%, 50%, 75%, 100% speed (2s each)
2. Reverse motion at 25%, 50%, 75%, 100% speed (2s each)
3. Direction changes (forward → stop → reverse)
4. PWM frequency verification (oscilloscope)
5. Current draw measurement (multimeter on motor power)

**Expected Results:**
- Smooth motor operation at all speeds
- No excessive current (< 5A typical, < 10A peak)
- No overheating of Pololu drivers
- No interference with LED or other GPIO

### 8.2 Encoder Test

**Test Script:** `scripts/test/test_wheel_encoders.py`

**Tests:**
1. Manually rotate wheel, verify pulse counting
2. Check direction detection (forward vs reverse)
3. Verify quadrature decoding (4× resolution)
4. Measure encoder PPR

**Expected Results:**
- Stable pulse count (no missed pulses)
- Correct direction (count increases forward, decreases reverse)
- 4× resolution working (count changes on all edges)

### 8.3 Integrated Motion Tests

**Test 1: Forward/Reverse Straight Line**
- Command: linear.x = 0.3 m/s, angular.z = 0
- Duration: 2 meters
- Measure: Actual distance, deviation from straight line
- **Pass criteria:** Distance error < 5%, lateral deviation < 0.1m

**Test 2: Spin in Place**
- Command: linear.x = 0, angular.z = π/4 rad/s
- Duration: 360° (8 seconds at π/4 rad/s)
- Measure: Actual rotation angle, center point movement
- **Pass criteria:** Rotation error < 5°, center movement < 0.05m

**Test 3: Figure-8 Pattern**
- Test combined linear + angular motion
- Measure odometry drift after complete figure-8
- **Pass criteria:** Position error < 0.2m after 10m path

**Test 4: Obstacle Avoidance (Future)**
- Integrate with distance sensors
- Stop when obstacle detected
- Navigate around obstacles

---

## 9. Safety Considerations

### 9.1 Current Limiting

**Pololu G2 Built-in Protection:**
- Default current limit: 50A (adjustable)
- Chopping frequency: 20 kHz
- Triggers on sustained over-current

**Software Monitoring:**
- Read CS (current sense) pin: ~20mV/A
- If current > 10A for > 2 seconds: Reduce PWM
- If current > 15A: Emergency stop
- Log current spikes for diagnostics

### 9.2 Thermal Management

**Pololu G2 Thermal Ratings:**
- Rated 21A continuous WITHOUT heatsink
- Monitor driver temperature if sustained high load
- Visual inspection: Look for discoloration, smell burning

**Mitigation:**
- Limit continuous operation to 50% duty cycle
- Add heatsink if running > 10A sustained
- Monitor ambient temperature (< 40°C recommended)

### 9.3 Emergency Stop

**Trigger Conditions:**
- ROS node crash or timeout (> 1s without cmd_vel)
- Over-current detection (> 15A)
- Communication loss with base station
- Physical emergency stop button (future)

**Emergency Stop Action:**
- Set both motors to speed 0 immediately
- Disable PWM outputs
- Log event to file
- Publish `/r2d2/emergency_stop` topic

### 9.4 Collision Detection

**Current Implementation:**
- Wheel stall detection (encoder stops, current high)

**Future Integration:**
- Ultrasonic distance sensors (forward/rear)
- Lidar 2D scanner (360° obstacle detection)
- IMU (detect unexpected impacts)
- Camera-based obstacle detection

---

## 10. Cross-References

**Related Documentation:**
- [`002_HARDWARE_REFERENCE.md`](002_HARDWARE_REFERENCE.md) - Complete hardware reference with GPIO limitations (Section 4.6)
- [`999_Next_Task_Camera_Pan_Tilt_final.md`](999_Next_Task_Camera_Pan_Tilt_final.md) - Pan/tilt system with PCA9685 solution
- [`001_ARCHITECTURE_OVERVIEW.md`](001_ARCHITECTURE_OVERVIEW.md) - System integration overview
- Power budget: [`002_HARDWARE_REFERENCE.md`](002_HARDWARE_REFERENCE.md) Section 5.2

**Motor Driver Resources:**
- [Pololu G2 24v21 Product Page](https://www.pololu.com/product/2995)
- [Pololu G2 Motor Driver User Guide](https://www.pololu.com/docs/pdf/0J71/g2_motor_driver.pdf)

**ROS 2 Resources:**
- [nav_msgs/Odometry Message](http://docs.ros.org/en/noetic/api/nav_msgs/html/msg/Odometry.html)
- [geometry_msgs/Twist Message](http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/Twist.html)
- [Differential Drive Tutorial](http://wiki.ros.org/differential_drive)

---

## Appendix A: Troubleshooting

### Motor Doesn't Respond

**Symptoms:** No motor movement when commanded

**Checks:**
1. Verify motor driver wiring (VM, GND, PWM, DIR pins)
2. Check battery voltage (should be 14.0-16.8V)
3. Verify PWM signal with oscilloscope (should be 10-20 kHz)
4. Check motor power fuse (10A, not blown)
5. Test motor directly with bench power supply (bypass driver)

### Encoder Count Unstable

**Symptoms:** Erratic pulse counting, count drifts when stationary

**Checks:**
1. Verify pull-up resistors (4.7kΩ to 3.3V)
2. Check encoder power (5V stable)
3. Check for electrical noise (shielded cable recommended)
4. Verify interrupt edge detection (GPIO.BOTH)
5. Check for loose connections

### Robot Doesn't Drive Straight

**Symptoms:** Curves to one side when commanded straight

**Causes:**
1. Motor speed mismatch (one motor faster)
2. Wheel diameter difference
3. Mechanical friction difference
4. Encoder calibration error

**Solutions:**
1. Add speed compensation in software (PID per wheel)
2. Measure and calibrate wheel diameters independently
3. Lubricate wheel bearings
4. Re-calibrate encoder PPR for each wheel

### Odometry Drift

**Symptoms:** Position error accumulates over time

**Causes:**
- Wheel slip on smooth surfaces
- Encoder noise
- Wheelbase measurement error
- Wheel diameter measurement error

**Solutions:**
- Periodic re-calibration (visual fiducials)
- Sensor fusion (IMU, camera odometry)
- Better traction (tire material)
- Accurate calibration (repeat measurement procedure)

---

**Document Status:** ✅ Specification Complete - Ready for Phase 3 Implementation  
**Last Updated:** January 6, 2026  
**Next Step:** Wire motors after pan motor implementation complete

---

**End of Document**

