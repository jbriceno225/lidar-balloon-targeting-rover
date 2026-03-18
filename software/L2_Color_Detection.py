"""
Color detection module for tracking system
Handles frame preparation, color filtering, and object detection
"""

import cv2
import numpy as np
import L2_Tracking_Config as config

def prepare_frame(frame):
    """
    Prepare frame for color detection
    
    Args:
        frame: Original camera frame
        
    Returns:
        Tuple of (small_frame, hsv)
    """
    # Resize for faster processing
    small_frame = cv2.resize(frame, (400, 300))
    
    # Convert to HSV color space for better color detection
    hsv = cv2.cvtColor(small_frame, cv2.COLOR_BGR2HSV)
    
    return small_frame, hsv

def create_mask(hsv, color_mode_index):
    """
    Create mask for the specified color
    
    Args:
        hsv: HSV color image
        color_mode_index: Index of color to track (0=red, 1=green, 2=blue)
        
    Returns:
        Tuple of (mask, blurred_mask)
    """
    # Get current color ranges
    current_color = config.color_ranges[color_mode_index]
    lower1 = current_color[0]
    upper1 = current_color[1]
    lower2 = current_color[2]
    upper2 = current_color[3]
    
    # Create primary color mask
    mask = cv2.inRange(hsv, lower1, upper1)
    
    # Add secondary range if needed (for red which wraps around hue circle)
    if lower2 is not None and upper2 is not None:
        mask2 = cv2.inRange(hsv, lower2, upper2)
        mask = cv2.bitwise_or(mask, mask2)
    
    # Clean up mask with morphological operations
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)   # Remove noise
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)  # Fill holes
    
    # Apply Gaussian blur for better circle detection
    blurred = cv2.GaussianBlur(mask, (7, 7), 2)
    
    return mask, blurred

def find_circles(blurred_mask):
    """
    Find circles in the mask using Hough transform
    
    Args:
        blurred_mask: Blurred binary mask image
        
    Returns:
        Detected circles or None if none found
    """
    # Find circles using Hough transform
    circles = cv2.HoughCircles(
        blurred_mask, 
        cv2.HOUGH_GRADIENT, 
        dp=1.2,         # Resolution accumulator ratio
        minDist=40,     # Minimum distance between circles
        param1=80,      # Upper threshold for Canny edge detector
        param2=30,      # Threshold for circle detection
        minRadius=10,   # Minimum circle radius
        maxRadius=100   # Maximum circle radius
    )
    
    return circles

def get_highlight_color(color_mode):
    """
    Get appropriate highlight color for current tracking mode
    
    Args:
        color_mode: Current color mode index
        
    Returns:
        BGR color tuple for highlighting
    """
    if color_mode == 0:      # Red
        return (0, 0, 255)   # BGR format
    elif color_mode == 1:    # Green
        return (0, 255, 0)
    else:                   # Blue
        return (255, 0, 0)