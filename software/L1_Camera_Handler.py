"""
Camera handling module for tracking system
Handles camera setup, frame capture, and visualization functions
With advanced debugging for camera connection issues
"""

import cv2
import time
import numpy as np
import os
import subprocess
import L2_Tracking_Config as config
import L1_Servo_Controller
import L2_Color_Detection

# Add a global variable to track if the servos have centered after detecting a target
has_centered_after_target = False

# Add a global variable to track if the offset has been applied
offset_applied = False

def debug_camera_info():
    """
    Print detailed debug information about available cameras
    """
    print("===== CAMERA DEBUG INFO =====")
    
    # Check video devices
    try:
        print("Looking for video devices:")
        devices = subprocess.check_output("ls -l /dev/video*", shell=True).decode('utf-8')
        print(devices)
    except subprocess.CalledProcessError:
        print("No video devices found with ls -l /dev/video*")
    
    # Try to get camera properties for all potential indices
    for i in range(10):
        cam = cv2.VideoCapture(i)
        if cam.isOpened():
            print(f"Camera index {i} opened successfully")
            # Try to read a frame
            ret, frame = cam.read()
            if ret:
                height, width, channels = frame.shape
                print(f"Read frame successfully: {width}x{height}, {channels} channels")
            else:
                print(f"Could not read frame from camera {i}")
            # Release the camera
            cam.release()
        else:
            print(f"Could not open camera at index {i}")
    
    print("============================")

def setup_camera():
    """
    Initialize and configure the USB Camera with extensive error handling
    
    Returns:
        Configured camera object
    """
    # Run debug first
    debug_camera_info()
    
    # Try GSTREAMER pipeline as an alternative method
    try_gstreamer = True
    
    # Get list of video devices
    video_devices = [f'/dev/video{i}' for i in range(20)]  # Check up to /dev/video19
    
    # Try different camera indices
    for device in video_devices:
        print(f"Trying camera at {device}...")
        
        # Standard OpenCV capture
        camera = cv2.VideoCapture(device)
        
        # Check if camera opened successfully
        if camera.isOpened():
            print(f"Successfully opened camera at {device}")
            
            # Set camera properties
            camera.set(cv2.CAP_PROP_FRAME_WIDTH, config.frame_width)
            camera.set(cv2.CAP_PROP_FRAME_HEIGHT, config.frame_height)
            print(f"Set camera resolution to {config.frame_width}x{config.frame_height}")
            
            # Wait for camera to initialize
            time.sleep(2)
            
            # Try to read a frame to make sure it's working
            ret, test_frame = camera.read()
            if ret:
                print(f"Camera {device} is working properly! Read a {test_frame.shape[1]}x{test_frame.shape[0]} frame")
                return camera
            else:
                print(f"Could not read frame from camera {device}, releasing and trying next...")
                camera.release()
        else:
            print(f"Could not open camera at {device}")
    
    # If we get here and GStreamer is available, try that as a fallback
    if try_gstreamer:
        try:
            print("Trying GStreamer pipeline...")
            # GStreamer pipeline for USB camera
            gst_pipeline = (
                f'v4l2src device=/dev/video0 ! '
                f'video/x-raw, width={config.frame_width}, height={config.frame_height} ! '
                f'videoconvert ! appsink'
            )
            camera = cv2.VideoCapture(gst_pipeline, cv2.CAP_GSTREAMER)
            
            if camera.isOpened():
                print("Successfully opened camera using GStreamer")
                # Try to read a frame
                ret, test_frame = camera.read()
                if ret:
                    print(f"GStreamer camera is working! Read a {test_frame.shape[1]}x{test_frame.shape[0]} frame")
                    return camera
                else:
                    print("Could not read frame from GStreamer camera")
                    camera.release()
            else:
                print("Could not open camera using GStreamer")
        except Exception as e:
            print(f"GStreamer error: {e}")
    
    # If all attempts fail, print error and exit
    print("ERROR: Could not open any camera after trying multiple methods")
    print("Please check your camera connection and permissions")
    print("Try running: sudo chmod 777 /dev/video*")
    exit()

def shutdown_camera(camera):
    """
    Properly close the camera connection
    
    Args:
        camera: Camera object to shut down
    """
    if camera is not None:
        camera.release()
        print("Camera released")

def capture_frame(camera):
    """
    Capture a frame from the camera
    
    Args:
        camera: Camera object
        
    Returns:
        BGR color frame
    """
    if camera is None:
        print("Error: Camera is None in capture_frame")
        return None
        
    ret, frame = camera.read()
    if not ret:
        print("Error: Failed to capture frame")
        return None
    
    return frame  # USB cameras typically return BGR format already, no conversion needed

def draw_interface(frame, mode_name):
    """
    Draw crosshairs and UI elements on the frame
    
    Args:
        frame: Frame to draw on
        mode_name: Current tracking mode name to display
    """
    # Draw crosshairs at center of frame
    cv2.line(frame, 
             (config.center_x, 0), 
             (config.center_x, config.frame_height), 
             (255, 255, 255), 1)
    cv2.line(frame, 
             (0, config.center_y), 
             (config.frame_width, config.center_y), 
             (255, 255, 255), 1)
    
    # Add tracking mode info to frame
    cv2.putText(frame, 
                f"MODE: {mode_name}", 
                (config.frame_width - 180, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

def process_target(frame, circles, scale_x, scale_y, color_mode, pid_module):
    """
    Process detected circles and update servo positions
    """
    global has_centered_after_target, offset_applied

    # Convert circles to numpy array and find the largest one
    circles = np.uint16(np.around(circles))
    largest_circle = max(circles[0], key=lambda c: c[2])
    x, y, radius = largest_circle
    
    # Scale coordinates back to original frame size
    x = int(x * scale_x)
    y = int(y * scale_y)
    radius = int(radius * ((scale_x + scale_y) / 2))
    
    # Calculate circle area
    circle_area = np.pi * (radius ** 2)
    
    # Only process if circle is large enough
    if circle_area >= 600:
        # Set target status and position
        config.has_target = True
        config.target_x = x
        config.target_y = y
        
        # Get highlight color for the current tracking mode
        highlight_color = L2_Color_Detection.get_highlight_color(color_mode)
        
        # Draw target circle and center point
        cv2.circle(frame, (x, y), radius, highlight_color, 3)
        cv2.circle(frame, (x, y), 5, (255, 255, 255), -1)
        
        # Calculate offsets from center
        x_offset = x - config.center_x
        y_offset = y - config.center_y
        
        # Add offset information to frame
        cv2.putText(frame, 
                    f"Offset: [{x_offset}, {y_offset}]", 
                    (20, 40),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, highlight_color, 2)
        
        # Update servo positions based on PID control
        pan_angle, tilt_angle, pan_adjust, tilt_adjust = pid_module.update_pan_tilt(x_offset, y_offset)
        
        # Check if target is centered (within deadzone)
        if abs(x_offset) < config.x_deadzone and abs(y_offset) < config.y_deadzone:
            # Activate laser when target is centered
            L1_Servo_Controller.activate_laser()
            
            # Set the flag to indicate the servos have centered after detecting a target
            has_centered_after_target = True
        
        # Debugging: Print the adjusted pan angle
        print(f"Adjusted Pan Angle: {pan_angle}")
        
        # Send commands to servos
        L1_Servo_Controller.set_servo_angles(int(pan_angle), int(tilt_angle))
        
        # Add PID debug info to frame
        cv2.putText(frame, 
                    f"PAN: {pan_angle:.1f} ({pan_adjust:.2f})", 
                    (20, 70),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 255), 1)
        cv2.putText(frame, 
                    f"TILT: {tilt_angle:.1f} ({tilt_adjust:.2f})", 
                    (20, 90),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 255), 1)
    else:
        # No target found - update target file
        try:
            with open("mxet300_labpantilt_has_target.txt", "w") as f:
                f.write("0")
        except Exception as e:
            print(f"Error writing target file: {e}")
        
        # Check if we should center servos after 5 seconds of no target
        if servo_available and (time.time() - last_target_time) > 10 and not has_centered_after_target:
            L1_Servo_Controller.center_servos()
            last_target_time = time.time()  # Reset timer after centering
            has_centered_after_target = False  # Reset the flag for future targets
            offset_applied = False  # Reset the offset flag for future targets
        
        cv2.putText(frame, "No target", (20, 40),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)