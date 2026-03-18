"""
Configuration for camera tracking system
Contains global variables and tracking settings
"""

import numpy as np

# Global tracking state variables
target_x = 0
target_y = 0
has_target = False
pan_angle = 90
tilt_angle = 90

# Color mode - 0:Red, 1:Green, 2:Blue
color_mode = 0
color_mode_names = ["RED", "GREEN", "BLUE"]

# Frame dimensions used for calculating target offsets
frame_width, frame_height = 800, 600
center_x, center_y = frame_width // 2, frame_height // 2

# Color ranges for different tracking modes (in HSV)
# Format: [lower_range1, upper_range1, lower_range2, upper_range2]
color_ranges = [
    # Red (0) - Needs two ranges because red wraps around the hue circle
    [
        np.array([0, 120, 100]), np.array([10, 255, 255]),    # Lower red
        np.array([170, 120, 100]), np.array([180, 255, 255])  # Upper red
    ],
    # Green (1)
    [
        np.array([35, 100, 100]), np.array([85, 255, 255]),   # Green range
        None, None  # No secondary range needed
    ],
    # Blue (2)
    [
        np.array([100, 100, 100]), np.array([140, 255, 255]), # Blue range
        None, None  # No secondary range needed
    ]
]

# PID controller tuning parameters
# Adjust these values to control tracking behavior
pan_pid_params = {
    'kp': 0.1, 
    'ki': 0.00,
    'kd': 0.05
}

tilt_pid_params = {
    'kp': 0.1,
    'ki': 0,
    'kd': 0.05
}

# Default deadzone values for x and y offsets
x_deadzone = 50
y_deadzone = 50
y_bias = 0  # Bias to keep target lower in frame (better stability)