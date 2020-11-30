#! /usr/bin/env python3

# Ioannis Broumas
# ioabro17@student.hh.se
# Nov 2020
# Camera Coordinates
# Remember OpenCV is BGR

import sys
import cv2 as cv
import numpy as np

lower_red = np.array([0,25,25])
upper_red = np.array([2,255,255])
element = np.ones((5,5),np.uint8)

# Read the video stream
cap = cv.VideoCapture(0)

HorizontalPixels = cap.get(3)
# print(cap.get(3)) # Width - Horizontal
VerticalPixels = cap.get(4)
# print(cap.get(4)) # Height - Vertical
# Resolution = HorizontalPixels * VerticalPixels
# VerticalDistance = 10
# cm_to_pixel = VerticalDistance / VerticalPixels

if not cap.isOpened:
    print('--(!)Error opening video capture')
    exit(0)
    
while True:

    ret, frame = cap.read()

    if frame is None:
        print('--(!) No captured frame -- Break!')
        break

    hsv_im = cv.cvtColor(frame,cv.COLOR_BGR2HSV)
    
    mask = cv.inRange(hsv_im,lower_red,upper_red)
    mask = cv.erode(mask,element,iterations = 2)
    
    rows, cols = np.where(mask == 255)

    check = np.median(cols)
    
    if(check > 3/4*HorizontalPixels):
        print('Object is on the right')
    elif(1/4*HorizontalPixels<check<3/4*HorizontalPixels):
        print('Object is in the center')
    elif(check <1/4*HorizontalPixels):
        print('Object is on the left')
    elif(math.isnan(check)):
        print('No object')

    red_frame = frame[:,:,2]
    blue_frame = frame[:,:,0]
    green_frame = frame[:,:,1]

    cv.imshow("Text", frame)
    
    if cv.waitKey(10) == 27: # Wait for 10ms, if key == 27 (esc char) break
        break

cv.destroyAllWindows()