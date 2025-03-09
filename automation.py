# Written by Garret Abboud
# This script integrates my previous collision detection code and MAVlink message tester into one script.
# This is intended to be setup to be automatically run upon boot of the RaspberryPi.
# NOTE: This script must be run within a virutal enviroment that is setup on the RaspberryPi beforehand.

### Make some functions for all the vision stuff to make the main loop cleaner, then we can integrate the mavlink stuff cuz its simple.

###### WORRY ABOUT THE AUTOMATION BEFORE COLLISION DETECTION

import cv2
import numpy as np
import subprocess
import time
import serial
#from pymavlink import mavutil

### Variables
# Predefined HSV values, to be determined using the PC program beforehand
HSV_LOWER = np.array([0, 0, 194])   # Hue Min, Sat Min, Val Min
HSV_UPPER = np.array([117, 254, 255]) # Hue Max, Sat Max, Val Max

# Minimum and maximum area for valid detection
MIN_AREA = 1000  
MAX_AREA = 1000000 
SIZE_THRESHOLD = 100000  # Defines 'close' vs. 'far' object, adjust as needed based on the objects you're detecting

# Servo id's
THROTTLE_ID = 3
STEERING_ID = 1

### Collision detection function
def collision_detection():
    ret, img = cap.read()
    if not ret:
        return None, None  # Return None if camera capture fails

    #imgHSV = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    # Apply HSV threshold
    mask = cv2.inRange(img, HSV_LOWER, HSV_UPPER)

    # Find contours
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if contours:
        largest_contour = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(largest_contour)

        if MIN_AREA < area < MAX_AREA:
            x, y, w, h = cv2.boundingRect(largest_contour)

            # Calculate the center of the detected object
            center_x = x + w // 2
            width = img.shape[1]
            image_center = width // 2

            # Determine movement logic
            if w * h > SIZE_THRESHOLD:
                throttle = 1200
            else:
                throttle = 1700

            # Determine steering direction
            if center_x < image_center:
                steering = 1700
            else:
                steering = 1300

            return throttle, steering

    return None, None  # No valid detection

### MAVLink interpretation function
# def request_servo_output():
#     print("Requesting SERVO_OUTPUT_RAW message...")
#     # Send a message interval request to the Pixhawk
#     connection.mav.command_long_send(
#         connection.target_system,    # Target system ID
#         connection.target_component, # Target component ID
#         mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, # Command ID
#         0,                           # Confirmation
#         mavutil.mavlink.MAVLINK_MSG_ID_SERVO_OUTPUT_RAW, # Message ID
#         10000,                     # Interval in microseconds (1 second)
#         0, 0, 0, 0, 0                # Unused parameters
#     )

### Initialization

# Connect to pixhawk
#connection = mavutil.mavlink_connection('/dev/serial/by-id/usb-ArduPilot_Pixhawk1_2B003D000F51383130333132-if00', baud=115200)
# Wait for a heartbeat from the Pixhawk
#connection.wait_heartbeat()
#print("Heartbeat received!")

# Initialize Camera
cap = cv2.VideoCapture(0)

# Request Message
#request_servo_output()

# Set up UART on Raspberry Pi UPDATE THIS SERIAL VALUE
#Disable for debugging
ser = serial.Serial('/dev/serial0', baudrate=115200, timeout=1)

# Initialize collision_avoidance
collision_avoidance = False

### Main loop
while True:
    # Pull message from MAVLink
    #mav_message = connection.recv_match(type='SERVO_OUTPUT_RAW', blocking=True) #Should blocking equal true?
    servo_throttle = f"servo{THROTTLE_ID}_raw"
    servo_steering = f"servo{STEERING_ID}_raw"
    #mav_throttle = getattr(mav_message, servo_throttle)
    #mav_steering = getattr(mav_message, servo_steering)

    cd_throttle, cd_steering = collision_detection()
    print(f"Throttle: {cd_throttle}, Steering: {cd_steering}")
    if cd_throttle is not None and cd_steering is not None:
        collision_avoidance = True
    else:
        collision_avoidance = False

    # Prepare the message
    if collision_avoidance:
        throttle = cd_throttle
        steering = cd_steering
        source = "CD"  # Collision Detection
    else:
        #throttle = mav_throttle if mav_throttle is not None else 1500  # Default neutral
        #steering = mav_steering if mav_steering is not None else 1500  # Default neutral
        source = "MV"  # MAVLink
        
    # Send UART message
    #message = f"{source} T:{throttle} S:{steering}\n"
    
    #Debug message
    # Set debug throttle and steering values
    throttle = 1 
    steering = 1

    # Create a bytes object with two uint8_t values
    message = bytes([throttle, steering])

    #Disable for debugging
    ser.write(message)
        
    # stupid ass debug message ong
    print(f"Sent: Throttle={throttle}, Steering={steering}")

    time.sleep(0.1)  # Small delay to reduce CPU usage?
# cap.released necessary here?
