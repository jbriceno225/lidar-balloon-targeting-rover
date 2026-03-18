"""
Main script for camera tracking system
This script coordinates all the components for the pan-tilt camera tracking system
Optimized for faster image updates
"""

import cv2
import numpy as np
import time
import os
import sys
import select
from threading import Thread
import L1_log as log

# Import our modular components
import L2_Tracking_Config as config
import L1_Camera_Handler
import L1_Servo_Controller
import L2_Color_Detection
import L2_PID_Controller
import L2_Telemetry

# Create directory for images
current_dir = os.path.dirname(os.path.abspath(__file__))
image_dir = os.path.join(current_dir, "camera_images")
os.makedirs(image_dir, exist_ok=True)

print("Initializing Camera Tracking System")

# Start the telemetry logger in a separate thread
logger_thread = Thread(target=L2_Telemetry.logger_thread, daemon=True)
logger_thread.start()

# Initialize keyboard handler if available
try:
    import keyboard
    keyboard_thread = Thread(target=L2_Telemetry.keyboard_handler, daemon=True)
    keyboard_thread.start()
    print("Keyboard handler started")
    keyboard_available = True
except ImportError:
    print("Keyboard module not available, using direct console input instead")
    keyboard_available = False

# Initialize camera
camera = L1_Camera_Handler.setup_camera()

# Initialize servo controller
try:
    L1_Servo_Controller.initialize()
    servo_available = True
    print("Servo controller initialized")
except Exception as e:
    print(f"Warning: Servo controller initialization failed: {e}")
    print("Running in camera-only mode (no servo control)")
    servo_available = False

# Add timer for target detection
last_target_time = time.time()

print(f"Starting in {config.color_mode_names[config.color_mode]} tracking mode")
print("Use the following commands:")
print("- Type 'r' for RED tracking, 'g' for GREEN tracking, 'b' for BLUE tracking")
print("- Type 'q' to exit")

running = True
frame_count = 0
last_save_time = time.time()
save_interval = 0.1  # Save images at 10 Hz instead of every frame

# Function to check for console input (non-blocking)
def check_input():
    # Check if input is available
    if select.select([sys.stdin], [], [], 0)[0]:
        # Read a single character
        key = sys.stdin.read(1)
        return key
    return None

# Make stdin non-blocking
import fcntl
import termios
old_settings = termios.tcgetattr(sys.stdin)
new_settings = termios.tcgetattr(sys.stdin)
new_settings[3] = new_settings[3] & ~termios.ICANON & ~termios.ECHO
termios.tcsetattr(sys.stdin, termios.TCSANOW, new_settings)
fl = fcntl.fcntl(sys.stdin.fileno(), fcntl.F_GETFL)
fcntl.fcntl(sys.stdin.fileno(), fcntl.F_SETFL, fl | os.O_NONBLOCK)

# Set a smaller frame size for processing
config.frame_width = 640   # Reduced from 800
config.frame_height = 480  # Reduced from 600
config.center_x = config.frame_width // 2
config.center_y = config.frame_height // 2

# Main tracking loop
try:
    while running:
        loop_start = time.time()
        
        # Capture frame
        frame = L1_Camera_Handler.capture_frame(camera)
        
        if frame is None:
            print("Error: Failed to capture frame, retrying...")
            time.sleep(1)
            continue
        
        # Process frame
        small_frame, hsv = L2_Color_Detection.prepare_frame(frame)
        mask, blurred = L2_Color_Detection.create_mask(hsv, config.color_mode)
        circles = L2_Color_Detection.find_circles(blurred)
        
        # Calculate scale factors
        scale_x = frame.shape[1] / small_frame.shape[1]
        scale_y = frame.shape[0] / small_frame.shape[0]
        
        # Draw interface
        L1_Camera_Handler.draw_interface(frame, config.color_mode_names[config.color_mode])
        
        # Reset target detection
        config.has_target = False
        
        # Process circles if found
        if circles is not None:
            if servo_available:
                L1_Camera_Handler.process_target(
                    frame, circles, scale_x, scale_y, 
                    config.color_mode, L2_PID_Controller
                )
                last_target_time = time.time()  # Update last target time when target is found
            else:
                # Simplified target drawing without servo control
                circles = np.uint16(np.around(circles))
                largest_circle = max(circles[0], key=lambda c: c[2])
                x, y, radius = largest_circle
                
                x = int(x * scale_x)
                y = int(y * scale_y)
                radius = int(radius * ((scale_x + scale_y) / 2))
                
                if np.pi * (radius ** 2) >= 600:
                    config.has_target = True
                    config.target_x = x
                    config.target_y = y
                    last_target_time = time.time()  # Update last target time when target is found
                    
                    # Update target file
                    try:
                        with open("mxet300_labpantilt_has_target.txt", "w") as f:
                            f.write("1")
                    except Exception as e:
                        print(f"Error writing target file: {e}")
                    
                    highlight_color = L2_Color_Detection.get_highlight_color(config.color_mode)
                    cv2.circle(frame, (x, y), radius, highlight_color, 3)
                    cv2.circle(frame, (x, y), 5, (255, 255, 255), -1)
        else:
            # No target found - update target file
            try:
                with open("mxet300_labpantilt_has_target.txt", "w") as f:
                    f.write("0")
            except Exception as e:
                print(f"Error writing target file: {e}")
            
            # Check if we should center servos after 5 seconds of no target
            if servo_available and (time.time() - last_target_time) > 5:
                L1_Servo_Controller.center_servos()
                last_target_time = time.time()  # Reset timer after centering
            
            cv2.putText(frame, "No target", (20, 40),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
        
        # Save images at reduced frequency
        current_time = time.time()
        if current_time - last_save_time >= save_interval:
            # Save smaller frame with lower quality for faster updates
            resized_frame = cv2.resize(frame, (320, 240))
            
            # Save to camera_images directory
            cv2.imwrite(os.path.join(image_dir, "current_frame.jpg"), resized_frame, 
                        [cv2.IMWRITE_JPEG_QUALITY, 30])
            
            # Save mask at smaller size too
            mask_small = cv2.resize(mask, (320, 240))
            mask_color = cv2.cvtColor(mask_small, cv2.COLOR_GRAY2BGR)
            cv2.imwrite(os.path.join(image_dir, "mask.jpg"), mask_color, 
                        [cv2.IMWRITE_JPEG_QUALITY, 30])
            
            # Also save to /tmp for broader access
            cv2.imwrite("/tmp/current_frame.jpg", resized_frame, 
                       [cv2.IMWRITE_JPEG_QUALITY, 30])
            
            last_save_time = current_time
        
        # Check for keyboard input
        if not keyboard_available:
            key = check_input()
            if key:
                if key.lower() == 'q':
                    print("Exiting...")
                    running = False
                elif key.lower() == 'r':
                    config.color_mode = 0  # Red
                    print(f"Tracking mode changed to {config.color_mode_names[config.color_mode]}")
                elif key.lower() == 'g':
                    config.color_mode = 1  # Green
                    print(f"Tracking mode changed to {config.color_mode_names[config.color_mode]}")
                elif key.lower() == 'b':
                    config.color_mode = 2  # Blue
                    print(f"Tracking mode changed to {config.color_mode_names[config.color_mode]}")
        
        frame_count += 1
        
        # Calculate and adjust processing speed
        process_time = time.time() - loop_start
        if process_time < 0.05:  # Target 20fps
            time.sleep(0.05 - process_time)
        
        # Print FPS every 30 frames
        if frame_count % 30 == 0:
            fps = 1.0 / (time.time() - loop_start + 0.000001)
            print(f"FPS: {fps:.1f}")

except KeyboardInterrupt:
    print("Stopping tracking...")
except Exception as e:
    print(f"Error: {e}")
finally:
    # Restore terminal settings
    termios.tcsetattr(sys.stdin, termios.TCSANOW, old_settings)
    fcntl.fcntl(sys.stdin.fileno(), fcntl.F_SETFL, fl)
    
    # Clean up
    L1_Camera_Handler.shutdown_camera(camera)
    
    # Center servos before exit if available
    if servo_available:
        L1_Servo_Controller.center_servos()
    
    print("Tracking stopped")

def track_target(camera, servo_available):
    """Main tracking function that can be called from other scripts"""
    # Capture frame
    frame = L1_Camera_Handler.capture_frame(camera)
    
    if frame is None:
        print("Error: Failed to capture frame, retrying...")
        return
    
    # Process frame
    small_frame, hsv = L2_Color_Detection.prepare_frame(frame)
    mask, blurred = L2_Color_Detection.create_mask(hsv, config.color_mode)
    circles = L2_Color_Detection.find_circles(blurred)
    
    # Calculate scale factors
    scale_x = frame.shape[1] / small_frame.shape[1]
    scale_y = frame.shape[0] / small_frame.shape[0]
    
    # Draw interface
    L1_Camera_Handler.draw_interface(frame, config.color_mode_names[config.color_mode])
    
    # Reset target detection
    config.has_target = False
    
    # Process circles if found
    if circles is not None:
        if servo_available:
            L1_Camera_Handler.process_target(
                frame, circles, scale_x, scale_y, 
                config.color_mode, L2_PID_Controller
            )
            last_target_time = time.time()  # Update last target time when target is found
        else:
            # Simplified target drawing without servo control
            circles = np.uint16(np.around(circles))
            largest_circle = max(circles[0], key=lambda c: c[2])
            x, y, radius = largest_circle
            
            x = int(x * scale_x)
            y = int(y * scale_y)
            radius = int(radius * ((scale_x + scale_y) / 2))
            
            if np.pi * (radius ** 2) >= 600:
                config.has_target = True
                config.target_x = x
                config.target_y = y
                last_target_time = time.time()  # Update last target time when target is found
                
                # Update target file
                try:
                    with open("mxet300_labpantilt_has_target.txt", "w") as f:
                        f.write("1")
                except Exception as e:
                    print(f"Error writing target file: {e}")
                
                highlight_color = L2_Color_Detection.get_highlight_color(config.color_mode)
                cv2.circle(frame, (x, y), radius, highlight_color, 3)
                cv2.circle(frame, (x, y), 5, (255, 255, 255), -1)
    else:
        # No target found - update target file
        try:
            with open("mxet300_labpantilt_has_target.txt", "w") as f:
                f.write("0")
        except Exception as e:
            print(f"Error writing target file: {e}")
        
        # Check if we should center servos after 5 seconds of no target
        if servo_available and (time.time() - last_target_time) > 5:
            L1_Servo_Controller.center_servos()
            last_target_time = time.time()  # Reset timer after centering
        
        cv2.putText(frame, "No target", (20, 40),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
    
    # Save images at reduced frequency
    current_time = time.time()
    if current_time - last_save_time >= save_interval:
        # Save smaller frame with lower quality for faster updates
        resized_frame = cv2.resize(frame, (320, 240))
        
        # Save to camera_images directory
        cv2.imwrite(os.path.join(image_dir, "current_frame.jpg"), resized_frame, 
                    [cv2.IMWRITE_JPEG_QUALITY, 30])
        
        # Save mask at smaller size too
        mask_small = cv2.resize(mask, (320, 240))
        mask_color = cv2.cvtColor(mask_small, cv2.COLOR_GRAY2BGR)
        cv2.imwrite(os.path.join(image_dir, "mask.jpg"), mask_color, 
                    [cv2.IMWRITE_JPEG_QUALITY, 30])
        
        # Also save to /tmp for broader access
        cv2.imwrite("/tmp/current_frame.jpg", resized_frame, 
                   [cv2.IMWRITE_JPEG_QUALITY, 30])
        
        last_save_time = current_time