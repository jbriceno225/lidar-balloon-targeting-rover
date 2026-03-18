"""
Servo controller module for pan-tilt mechanism
Handles servo initialization and control functions
"""

import board
import busio
from adafruit_pca9685 import PCA9685
from adafruit_motor import servo
import L2_Tracking_Config as config
import time
from threading import Thread

# Global variables to store servo objects
pan_servo = None
tilt_servo = None
laser_servo = None
pca = None

# Laser control variables
last_laser_activation = 0
laser_cooldown = 15  # seconds between activations
laser_active = False
laser_thread = None

def initialize():
    """
    Initialize the PCA9685 controller and servo objects
    """
    global pan_servo, tilt_servo, laser_servo, pca
    
    # Initialize the PCA9685 servo controller with address 0x43
    i2c = busio.I2C(board.SCL, board.SDA)
    pca = PCA9685(i2c, address=0x42)
    pca.frequency = 50  # Standard servo frequency is 50Hz

    # Create servo objects for pan, tilt and laser
    pan_servo = servo.Servo(pca.channels[0])
    tilt_servo = servo.Servo(pca.channels[1])
    laser_servo = servo.Servo(pca.channels[2])

    # Initialize servo positions to center
    center_servos()
    # Initialize laser servo to off position (0 degrees)
    laser_servo.angle = 0

def set_servo_angles(pan_angle, tilt_angle):
    """
    Set the servo angles for pan and tilt
    
    Args:
        pan_angle: Pan servo angle (0-180 degrees)
        tilt_angle: Tilt servo angle (0-180 degrees)
    """
    global pan_servo, tilt_servo
    
    if pan_servo is None or tilt_servo is None:
        # Initialize if not already done
        initialize()
    
    # Set servo angles
    pan_servo.angle = pan_angle
    tilt_servo.angle = tilt_angle

def center_servos():
    """
    Move servos to center position (90 degrees)
    """
    global pan_servo, tilt_servo
    
    if pan_servo is None or tilt_servo is None:
        # Initialize if not already done
        initialize()
    
    # Set servos to 90 degrees (center)
    pan_servo.angle = 90
    tilt_servo.angle = 90
    
    # Update config values
    config.pan_angle = 90
    config.tilt_angle = 90

def _laser_activation_thread():
    """
    Internal function to handle laser activation in a separate thread
    """
    global laser_servo, laser_active
    
    if laser_servo is None:
        # Initialize if not already done
        initialize()
    
    # Default position when no target is detected
    laser_servo.angle = 45
    time.sleep(2)
    # Move to -85 degrees
    laser_servo.angle = 75
    
    # Hold position for 8 seconds
    time.sleep(3)
    
    # Return to default position
    laser_servo.angle = 45
    
    laser_active = False

def activate_laser():
    """
    Activate the laser by moving the servo to 45 degrees and back
    Only activates if cooldown period has passed and laser is not already active
    """
    global last_laser_activation, laser_active, laser_thread
    
    current_time = time.time()
    
    # Check if we can activate the laser
    if (current_time - last_laser_activation >= laser_cooldown and 
        not laser_active and 
        laser_thread is None or not laser_thread.is_alive()):
        
        # Update activation time and status
        last_laser_activation = current_time
        laser_active = True
        
        # Start laser activation in a separate thread
        laser_thread = Thread(target=_laser_activation_thread)
        laser_thread.daemon = True  # Thread will exit when main program exits
        laser_thread.start()