# Written by Garret Abboud
# This script integrates my previous collision detection code and MAVlink message tester into one script.
# This is intended to be setup to be automatically run upon boot of the RaspberryPi.
# NOTE: This script must be run within a virutal enviroment that is setup on the RaspberryPi beforehand.

### Make some functions for all the vision stuff to make the main loop cleaner, then we can integrate the mavlink stuff cuz its simple.

import cv2
import numpy as np
import subprocess
import time
import serial
from pymavlink import mavutil

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
        return None, None

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
                throttle = 20
            else:
                throttle = 70

            # Determine steering direction
            if center_x < image_center:
                steering = 10
            else:
                steering = 90

            return throttle, steering

    return None, None  # No valid detection

### Initialization

# Connect to pixhawk
connection = mavutil.mavlink_connection('/dev/serial/by-id/usb-ArduPilot_Pixhawk1_2B003D000F51383130333132-if00', baud=115200)
# Wait for a heartbeat from the Pixhawk
connection.wait_heartbeat()
print("Heartbeat received!")

# Initialize Camera
cap = cv2.VideoCapture(0)

# Set up UART on Raspberry Pi UPDATE THIS SERIAL VALUE
#Disable for debugging
ser = serial.Serial('/dev/serial0', baudrate=9600, timeout=5)

# Initialize collision_avoidance
collision_avoidance = False

### Main loop
while True:
    # Pull message from MAVLink
    mav_message = connection.recv_match(type='SERVO_OUTPUT_RAW', blocking=True) #Should blocking equal true?
    servo_throttle = f"servo{THROTTLE_ID}_raw"
    servo_steering = f"servo{STEERING_ID}_raw"
    mav_throttle = getattr(mav_message, servo_throttle)
    mav_steering = getattr(mav_message, servo_steering)

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
    else:
        mav_throttle = mav_throttle if mav_throttle is not None else 1500
        mav_steering = mav_steering if mav_steering is not None else 1500
        throttle = max(0, min(100, int((mav_throttle - 1000) / 10)))
        steering = max(0, min(100, int((mav_steering - 1000) / 10)))

    # Create a bytes object with two uint8_t values for UART
    message = bytes([throttle, steering])

    #Disable for debugging
    ser.write(message)

    # Debug sent message print
    print(f"Sent: Throttle={message[0]}, Steering={message[1]}")

    time.sleep(0.001)  # Small delay to reduce CPU usage?
# cap.released necessary here?
