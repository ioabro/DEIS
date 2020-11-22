#! /usr/bin/env python3

# Ioannis Broumas
# ioabro17@student.hh.se
# Nov 2020
# Camera Coordinates
# Remember OpenCV is BGR

import sys
import cv2 as cv
import numpy as np

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

    red_frame = frame[:,:,2]
    blue_frame = frame[:,:,0]
    green_frame = frame[:,:,1]

    cv.imshow("Text", frame)
    
    if cv.waitKey(10) == 27: # Wait for 10ms, if key == 27 (esc char) break
        break

cv.destroyAllWindows()