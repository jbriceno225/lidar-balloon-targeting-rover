"""
Telemetry module for tracking system
Handles logging of tracking data and keyboard input
"""

import time
import os
import L1_log as log
import L2_Tracking_Config as config

def logger_thread():
    """
    Thread function to continuously log telemetry data for Node-RED
    """
    print("Starting telemetry logger thread")
    
    # Get path to image directory
    current_dir = os.path.dirname(os.path.abspath(__file__))
    image_dir = os.path.join(current_dir, "camera_images")
    
    while True:
        try:
            # Log all telemetry data to SCUTTLE log files
            log.tmpFile(config.pan_angle, "pantilt_pan_angle.txt")
            log.tmpFile(config.tilt_angle, "pantilt_tilt_angle.txt")
            log.tmpFile(config.target_x, "pantilt_target_x.txt")
            log.tmpFile(config.target_y, "pantilt_target_y.txt")
            log.tmpFile(1 if config.has_target else 0, "pantilt_has_target.txt")
            log.tmpFile(config.color_mode, "pantilt_color_mode.txt")
            
            # Also save telemetry to camera_images directory for easy access
            with open(os.path.join(image_dir, "telemetry.txt"), "w") as f:
                f.write(f"Pan: {config.pan_angle:.1f}\n")
                f.write(f"Tilt: {config.tilt_angle:.1f}\n")
                f.write(f"Target: {'Yes' if config.has_target else 'No'}\n")
                f.write(f"X: {config.target_x}\n")
                f.write(f"Y: {config.target_y}\n")
                f.write(f"Color: {config.color_mode_names[config.color_mode]}\n")
                
            time.sleep(0.1)  # Update at 10Hz
        except Exception as e:
            print(f"Logger error: {e}")
            time.sleep(1)  # Wait a bit longer if there's an error

def keyboard_handler():
    """
    Thread function to handle keyboard input for color mode switching
    Requires the keyboard module to be installed
    """
    print("Starting keyboard handler thread")
    try:
        import keyboard
        while True:
            # Check for number keys 1, 2, 3 to change color mode
            if keyboard.is_pressed('1'):
                config.color_mode = 0  # Red
                print(f"Tracking mode changed to {config.color_mode_names[config.color_mode]}")
                time.sleep(0.3)  # Debounce
            elif keyboard.is_pressed('2'):
                config.color_mode = 1  # Green
                print(f"Tracking mode changed to {config.color_mode_names[config.color_mode]}")
                time.sleep(0.3)  # Debounce
            elif keyboard.is_pressed('3'):
                config.color_mode = 2  # Blue
                print(f"Tracking mode changed to {config.color_mode_names[config.color_mode]}")
                time.sleep(0.3)  # Debounce
            time.sleep(0.1)
    except Exception as e:
        print(f"Keyboard handler error: {e}")