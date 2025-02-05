# Written by Garret Abboud
# This script integrates my previous collision detection code and MAVlink message tester into one script.
# This is intended to be setup to be automatically run upon boot of the RaspberryPi.
# NOTE: This script must be run within a virutal enviroment that is setup on the RaspberryPi beforehand.

### Make some functions for all the vision stuff to make the main loop cleaner, then we can integrate the mavlink stuff cuz its simple.


import cv2
import numpy as np
import RPi.GPIO as GPIO
import subprocess
import time

# Image capture variables
# Predefined HSV values, to be determined using the PC program beforehand
HSV_LOWER = np.array([0, 0, 194])   # Hue Min, Sat Min, Val Min
HSV_UPPER = np.array([117, 254, 255]) # Hue Max, Sat Max, Val Max

# Minimum and maximum area for valid detection
MIN_AREA = 1000  
MAX_AREA = 1000000 
SIZE_THRESHOLD = 100000  # Defines 'close' vs. 'far' object, adjust as needed based on the objects you're detecting

# Initialzie Camera
cap = cv2.VideoCapture(0)

def collision_detection():
    ret, img = cap.read()
    if not ret:
        return None, None  # Return None if camera capture fails

    imgHSV = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    # Apply HSV threshold
    mask = cv2.inRange(imgHSV, HSV_LOWER, HSV_UPPER)

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
                throttle = 1200  # Reduce throttle when object is close
            else:
                throttle = 1700  # Increase throttle when object is far

            # Determine steering direction
            if center_x < image_center:
                steering = 1700  # Object on left -> Turn right
            else:
                steering = 1300  # Object on right -> Turn left

            return throttle, steering

    return None, None  # No valid detection

# Main loop
while True:
    throttle, steering = collision_detection()
    if throttle is not None and steering is not None:
        print(f"Throttle: {throttle}, Steering: {steering}")
    else
        print("ERROR returning none ")

    time.sleep(0.1)  # Small delay to reduce CPU usage
# cap.released necessary here?
