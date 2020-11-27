#! /usr/bin/env python3

# Ioannis Broumas
# ioabro17@student.hh.se
# Nov 2020
# Remember OpenCV is BGR

# Resources: 
# - OpenCV-Python tutorial for calibration: http://opencv-python-tutroals.readthedocs.io/en/latest/py_tutorials/py_calib3d/py_calibration/py_calibration.html

import os
import numpy as np
import cv2 as cv
import glob
import pickle

measuredLength = 23 # Measured length of one side of the square in the printed chessboard mm 

# termination criteria
criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
objp = np.zeros((6*9,3), np.float32)
objp[:,:2] = np.mgrid[0:9,0:6].T.reshape(-1,2)*measuredLength

# Arrays to store object points and image points from all the images.
objpoints = [] # 3d point in real world space
imgpoints = [] # 2d points in image plane.

cam = cv.VideoCapture(0)

count = 0

while count < 51:

    # Capturing each frame of our video stream
    ret, QueryImg = cam.read()
    gray = cv.cvtColor(QueryImg, cv.COLOR_BGR2GRAY)

    # Find the chess board corners
    ret, corners = cv.findChessboardCorners(gray, (9,6), None)

    # If found, add object points, image points (after refining them)
    if ret == True:
        objpoints.append(objp)
        corners2 = cv.cornerSubPix(gray,corners, (11,11), (-1,-1), criteria)
        imgpoints.append(corners)
        count += 1
        # Draw and display the corners
        cv.drawChessboardCorners(QueryImg, (9,6), corners2, ret)
        cv.imshow('img', QueryImg)
        if cv.waitKey(500) == 27: # Wait for 10ms, if key == 27 (esc char) break
            break

cv.destroyAllWindows()

print("Done taking images!")
print("Calculating...")

ret, mtx, dist, rvecs, tvecs = cv.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)

# Print matrix and distortion coefficient to the console
print(mtx)
print(dist)

# Output values to be used where matrix+dist is required
f = open('calibration.pckl', 'wb')
pickle.dump((mtx, dist), f)
f.close()

print(tvecs)
