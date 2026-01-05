#!/usr/bin/env python3
"""
PID Controller for smooth head tracking.

Implements a standard PID controller with:
- Anti-windup (integral clamping)
- Output limiting
- Derivative filtering (optional)
"""

import time


class PIDController:
    """
    PID Controller with anti-windup and output limiting.
    
    Designed for smooth servo/motor control in face tracking applications.
    """
    
    def __init__(
        self,
        kp: float = 1.0,
        ki: float = 0.0,
        kd: float = 0.0,
        output_min: float = -1.0,
        output_max: float = 1.0,
        integral_limit: float = None,
        derivative_filter_alpha: float = 0.1
    ):
        """
        Initialize PID controller.
        
        Args:
            kp: Proportional gain
            ki: Integral gain
            kd: Derivative gain
            output_min: Minimum output value
            output_max: Maximum output value
            integral_limit: Maximum integral accumulation (anti-windup)
            derivative_filter_alpha: Low-pass filter for derivative (0=heavy filter, 1=no filter)
        """
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_min = output_min
        self.output_max = output_max
        self.integral_limit = integral_limit if integral_limit is not None else output_max
        self.derivative_filter_alpha = derivative_filter_alpha
        
        # Internal state
        self._integral = 0.0
        self._last_error = 0.0
        self._last_derivative = 0.0
        self._last_time = None
    
    def reset(self):
        """Reset controller state (integral, derivative memory)."""
        self._integral = 0.0
        self._last_error = 0.0
        self._last_derivative = 0.0
        self._last_time = None
    
    def compute(self, error: float, dt: float = None) -> float:
        """
        Compute PID output for given error.
        
        Args:
            error: Current error (setpoint - measurement)
            dt: Time delta in seconds (auto-computed if None)
            
        Returns:
            Control output (clamped to output_min/output_max)
        """
        current_time = time.time()
        
        # Compute dt if not provided
        if dt is None:
            if self._last_time is None:
                dt = 0.033  # Default to ~30Hz
            else:
                dt = current_time - self._last_time
        
        # Prevent division by zero or huge derivatives from time glitches
        dt = max(dt, 0.001)
        dt = min(dt, 1.0)
        
        self._last_time = current_time
        
        # Proportional term
        p_term = self.kp * error
        
        # Integral term with anti-windup
        self._integral += error * dt
        self._integral = self._clamp(self._integral, -self.integral_limit, self.integral_limit)
        i_term = self.ki * self._integral
        
        # Derivative term with filtering
        raw_derivative = (error - self._last_error) / dt
        filtered_derivative = (
            self.derivative_filter_alpha * raw_derivative +
            (1.0 - self.derivative_filter_alpha) * self._last_derivative
        )
        self._last_derivative = filtered_derivative
        d_term = self.kd * filtered_derivative
        
        self._last_error = error
        
        # Compute total output
        output = p_term + i_term + d_term
        
        # Clamp output
        output = self._clamp(output, self.output_min, self.output_max)
        
        return output
    
    def _clamp(self, value: float, min_val: float, max_val: float) -> float:
        """Clamp value to range [min_val, max_val]."""
        return max(min_val, min(max_val, value))
    
    def set_gains(self, kp: float = None, ki: float = None, kd: float = None):
        """
        Update PID gains at runtime.
        
        Args:
            kp: New proportional gain (or None to keep current)
            ki: New integral gain (or None to keep current)
            kd: New derivative gain (or None to keep current)
        """
        if kp is not None:
            self.kp = kp
        if ki is not None:
            self.ki = ki
        if kd is not None:
            self.kd = kd
    
    def set_output_limits(self, output_min: float, output_max: float):
        """Update output limits."""
        self.output_min = output_min
        self.output_max = output_max

