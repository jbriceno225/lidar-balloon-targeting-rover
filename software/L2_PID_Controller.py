"""
PID Controller for smooth tracking movements
Implements a proportional-integral-derivative controller
"""

import numpy as np
import L2_Tracking_Config as config

class PID:
    """PID Controller for smooth and accurate tracking"""
    def __init__(self, kp, ki, kd):
        # Controller gains
        self.kp = kp  # Proportional gain - immediate response
        self.ki = ki  # Integral gain - reduces steady-state error
        self.kd = kd  # Derivative gain - reduces overshoot

        # Controller state
        self.prev_error = 0
        self.integral = 0.4

    def update(self, error):
        """Update PID calculation with new error value"""
        # Accumulate error for integral term
        self.integral += error
        
        # Calculate derivative term (rate of change of error)
        derivative = error - self.prev_error
        
        # Calculate total output as weighted sum of P, I, and D terms
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        
        # Store current error for next iteration
        self.prev_error = error
        
        return output

def apply_deadzone(error, deadzone, bias=0):
    """
    Apply a deadzone to error values to prevent tiny movements
    
    Args:
        error: The error value
        deadzone: Ignore errors smaller than this value
        bias: Optional bias to add (used to favor one direction)
        
    Returns:
        0 if error is within deadzone, otherwise the adjusted error
    """
    adjusted_error = error + bias
    return 0 if abs(adjusted_error) < deadzone else adjusted_error

# Initialize pan and tilt PID controllers
pan_controller = PID(
    config.pan_pid_params['kp'],
    config.pan_pid_params['ki'],
    config.pan_pid_params['kd']
)

tilt_controller = PID(
    config.tilt_pid_params['kp'],
    config.tilt_pid_params['ki'],
    config.tilt_pid_params['kd']
)

def update_pan_tilt(x_offset, y_offset):
    """
    Calculates PID control outputs for pan and tilt servos
    
    Args:
        x_offset: Horizontal offset from center in pixels
        y_offset: Vertical offset from center in pixels
        
    Returns:
        Tuple of (pan_angle, tilt_angle) to set
    """
    # Apply deadzones and biases
    x_offset = apply_deadzone(x_offset, config.x_deadzone)
    y_offset = apply_deadzone(y_offset, config.y_deadzone, config.y_bias)
    
    # Calculate PID adjustments for servos
    pan_adjust = np.clip(pan_controller.update(-x_offset), -4, 4)  # Negate x_offset for correct direction
    tilt_adjust = np.clip(tilt_controller.update(y_offset), -4, 4)
    
    # Update servo angles
    pan_angle = np.clip(config.pan_angle + pan_adjust, 0, 180)
    tilt_angle = np.clip(config.tilt_angle + tilt_adjust, 0, 180)
    
    # Update global angle values
    config.pan_angle = pan_angle
    config.tilt_angle = tilt_angle
    
    return pan_angle, tilt_angle, pan_adjust, tilt_adjust