#! /usr/bin/env python3

# Ioannis Broumas
# ioabro17@student.hh.se
# Nov 2020
# Remember OpenCV is BGR

import os
import sys
import cv2 as cv
import numpy as np
from cv2 import aruco
import glob
import pickle

#ROS 2
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

# Check for camera calibration data
if not os.path.exists('/home/ubuntu/dev_ws/src/deis_py_dev/cameraCalibrations/sparkieCalibration.pckl'):
    print("You need to calibrate the camera you'll be using. See calibration project directory for details.")
    exit()
else:
    f = open('/home/ubuntu/dev_ws/src/deis_py_dev/cameraCalibrations/sparkieCalibration.pckl', 'rb')
    (cameraMatrix, distCoeffs) = pickle.load(f)
    f.close()
    if cameraMatrix is None or distCoeffs is None:
        print("Calibration issue. Remove ./calibration.pckl and recalibrate your camera with CalibrateCamera.py.")
        exit()

# Constant parameters used in Aruco methods
ARUCO_PARAMETERS = aruco.DetectorParameters_create()
ARUCO_DICT = aruco.Dictionary_get(aruco.DICT_6X6_250)

measuredLength = 3.2 # Measured length of one side of the printed marker cm

# Create vectors we'll be using for rotations and translations for postures
rvecs, tvecs = None, None

class Scanner(Node):

    def __init__(self):
        super().__init__('scanner')
        self.publisher_ = self.create_publisher(String, 'sparkieScanner', 10)
        self.get_logger().info("Node SparkieScanner initialized!")

def main(args=None):
    rclpy.init(args=args)

    s = Scanner()
    msg = String()

    ID = []
    R = []

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
        s.destroy_node()
        rclpy.shutdown()
    
    while True:

        # Capturing each frame of our video stream
        ret, frame = cap.read()

        if frame is None:
            print('--(!) No captured frame -- Break!')
            break

        # grayscale image
        gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
        # Detect Aruco markers
        corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, ARUCO_DICT, parameters=ARUCO_PARAMETERS)
        
        if len(ids) >= 3:
            k = 0
            # Print corners and ids to the console
            for i, corner in zip(ids, corners):
                ID[k] = i
                # Outline the detected marker in our image
                frame = aruco.drawDetectedMarkers(frame, corners, borderColor=(0, 0, 255))
                rvecs, tvecs, _objPoints = aruco.estimatePoseSingleMarkers(corner, measuredLength, cameraMatrix, distCoeffs)
            	distance = cv.norm(tvecs)
                R[k] = distance
                k += 1
                if k == 3:
                    break

            msg.data = ID[0] + "_" + R[0] + "_" + ID[1] + "_" + R[1] + "_" + ID[2] + "_" + R[2]
            s.publisher_.publish(msg)
            s.get_logger().info('Publishing: "%s"' % msg.data)

        frame_resized = cv.resize(frame, (640,360))    
        cv.imshow("Snapshot", frame_resized)
        
        # cv.imshow("Snapshot", frame)
            
        if cv.waitKey(10) == 27: # Wait for 10ms, if key == 27 (esc char) break
            break

    cv.destroyAllWindows()
    
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    f.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
