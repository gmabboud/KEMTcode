### Written by Garret Abboud based on code by Jiaming Cai
# This code comments out all Pi functionalities for testing on other computers.
import cv2
import numpy as np
#import RPi.GPIO as GPIO
import subprocess
import time

# Code to be used with Pi
#GPIO.setmode(GPIO.BCM)
#GPIO.setwarnings(False)
#set 18 to low, when detecting orangepixels too many, set to high
#run_pin=18
#GPIO.setup(run_pin, GPIO.OUT, initial=GPIO.LOW)
#This is to detect if this script is running under crontab
#pin=22
#GPIO.setup(pin, GPIO.OUT, initial=GPIO.HIGH)

# Used for the trackbars update function
def empty(a):
    pass
    
# Resizes and stacks images horizontally and vertically
def stackImages(scale,imgArray):
    rows = len(imgArray)
    cols = len(imgArray[0])
    rowsAvailable = isinstance(imgArray[0], list)
    width = imgArray[0][0].shape[1]
    height = imgArray[0][0].shape[0]
    if rowsAvailable:
        for x in range ( 0, rows):
            for y in range(0, cols):
                if imgArray[x][y].shape[:2] == imgArray[0][0].shape [:2]:
                    imgArray[x][y] = cv2.resize(imgArray[x][y], (0, 0), None, scale, scale)
                else:
                    imgArray[x][y] = cv2.resize(imgArray[x][y], (imgArray[0][0].shape[1], imgArray[0][0].shape[0]), None, scale, scale)
                if len(imgArray[x][y].shape) == 2: imgArray[x][y]= cv2.cvtColor( imgArray[x][y], cv2.COLOR_GRAY2BGR)
        imageBlank = np.zeros((height, width, 3), np.uint8)
        hor = [imageBlank]*rows
        hor_con = [imageBlank]*rows
        for x in range(0, rows):
            hor[x] = np.hstack(imgArray[x])
        ver = np.vstack(hor)
    else:
        for x in range(0, rows):
            if imgArray[x].shape[:2] == imgArray[0].shape[:2]:
                imgArray[x] = cv2.resize(imgArray[x], (0, 0), None, scale, scale)
            else:
                imgArray[x] = cv2.resize(imgArray[x], (imgArray[0].shape[1], imgArray[0].shape[0]), None,scale, scale)
            if len(imgArray[x].shape) == 2: imgArray[x] = cv2.cvtColor(imgArray[x], cv2.COLOR_GRAY2BGR)
        hor= np.hstack(imgArray)
        ver = hor
    return ver

cv2.namedWindow("TrackBars")
cv2.resizeWindow("TrackBars",800,400)

# Argument 3 is the default value for the track bars
# Default values from previous code:
cv2.createTrackbar("Hue Min","TrackBars",0,179,empty)
cv2.createTrackbar("Hue Max","TrackBars",117,179,empty)
cv2.createTrackbar("Sat Min","TrackBars",0,255,empty)
cv2.createTrackbar("Sat Max","TrackBars",254,255,empty)
cv2.createTrackbar("Val Min","TrackBars",194,255,empty)
cv2.createTrackbar("Val Max","TrackBars",255,255,empty)

cap = cv2.VideoCapture(0)

while 1:
    ret, img = cap.read()
    imgHSV= cv2.cvtColor(img,cv2.COLOR_BGR2HSV)
    h_min = cv2.getTrackbarPos("Hue Min","TrackBars")
    h_max = cv2.getTrackbarPos("Hue Max", "TrackBars")
    s_min = cv2.getTrackbarPos("Sat Min", "TrackBars")
    s_max = cv2.getTrackbarPos("Sat Max", "TrackBars")
    v_min = cv2.getTrackbarPos("Val Min", "TrackBars")
    v_max = cv2.getTrackbarPos("Val Max", "TrackBars")

    # Define HSV arrays
    lower = np.array([h_min,s_min,v_min])
    upper = np.array([h_max,s_max,v_max])

    #print(h_min,h_max,s_min,s_max,v_min,v_max)
    mask = cv2.inRange(img,lower,upper)
    imgResult = cv2.bitwise_and(img,img,mask=mask)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    # Will need to hone these in with testing
    MIN_AREA = 1000  # Minimum area of the buoy in pixels
    MAX_AREA = 1000000 # Maximum area of the buoy in pixels

    # Draw contours on result image
    largest_contour = max(contours, key=cv2.contourArea)
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if MIN_AREA < area < MAX_AREA:
            cv2.drawContours(imgResult, [cnt], -1, (255, 0, 0), 3)  # Draw contour in blue

            # Check where the largest contour is on the screen
            if cnt is largest_contour:
                x, y, w, h = cv2.boundingRect(largest_contour)

                # Calculate the center of the contour
                center_x = x + w // 2 
                center_y = y + h // 2

                # DEBUG
                # Draw the bounding box and centroid
                cv2.rectangle(imgResult, (x, y), (x + w, y + h), (0, 255, 0), 2)
                cv2.circle(imgResult, (center_x, center_y), 5, (0, 0, 255), -1) 

                # Check the size of the bounding box
                bounding_box_area = w * h
                size_threshold = 20000  # Adjust as needed based on the objects you're detecting
                if bounding_box_area > size_threshold:
                    print(f"Object is close\n    Send LOW THROTTLE")
                    # Check if the object center is on the left or right side
                    width = img.shape[1]
                    image_center = width // 2
                    if center_x < image_center:
                        print("Object is on LEFT side\n    Send RIGHT TURN")
                    else:
                        print("Object is on RIGHT side\n    Send LEFT TURN")
                else:
                    print("Object is far\n    Send HIGH THROTTLE")



    # Save image to disk
    #cv2.imwrite('masked.png', mask)
    
    # Display image stack
    imgStack = stackImages(0.6,([img,imgHSV],[mask,imgResult]))
    cv2.imshow("Image Stack", imgStack)

    # Display and wait x milliseconds 
    # This cannot be 0 or a very low number for the sliders to work
    cv2.waitKey(100)
cap.released()

cv2.destroyAllWindows()
