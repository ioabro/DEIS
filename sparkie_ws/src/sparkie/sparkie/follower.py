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
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Range

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
ARUCO_DICT = aruco.Dictionary_get(aruco.DICT_6X6_1000)

# # Create grid board object we're using in our stream
# board = aruco.GridBoard_create(
#         markersX=2,
#         markersY=2,
#         markerLength=0.09,
#         markerSeparation=0.01,
#         dictionary=ARUCO_DICT)

# Create vectors we'll be using for rotations and translations for postures
rvecs, tvecs = None, None

class Follower(Node):

    def __init__(self):
        super().__init__('follower')
        self.publisher_ = self.create_publisher(String, 'cam_follower', 10)
        self.get_logger().info("It's rabbit season!")

def main(args=None):
    rclpy.init(args=args)

    f = Follower()
    msg = String()

    base_speed = 70

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
        f.destroy_node()
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
        
        if ids is not None:
            # # Print corners and ids to the console
            # for i, corner in zip(ids, corners):
            #     print('ID: {}; Corners: {}'.format(i, corner))
            
            # Outline all of the markers detected in our image
            frame = aruco.drawDetectedMarkers(frame, corners, borderColor=(0, 0, 255))
            # Get center
            center_x = (corners[0][0][0][0] + corners[0][0][1][0] + corners[0][0][2][0] + corners[0][0][3][0]) / 4
            center_y = (corners[0][0][0][1] + corners[0][0][1][1] + corners[0][0][2][1] + corners[0][0][3][1]) / 4
            print("Center X: %d" %center_x)
            print("Center Y: %d" %center_y)

            rvecs, tvecs, _objPoints = aruco.estimatePoseSingleMarkers(corners, 3.2, cameraMatrix, distCoeffs)
            distance = cv.norm(tvecs)
            print(cv.norm(tvecs)) 

            # Find center
            cam_center = HorizontalPixels / 2

            if distance < 17.0:
                msg.data = '0 0\n'
                f.publisher_.publish(msg)
                f.get_logger().info('Publishing: "%s"' % msg.data)
            elif center_x < cam_center:
                gain = int((cam_center - center_x) / 10)
                LM = base_speed 
                RM = base_speed + gain
                msg.data = '%d %d\n' %(LM, RM)
                f.publisher_.publish(msg)
                f.get_logger().info('Publishing: "%s"' % msg.data)
            elif center_x > cam_center:
                gain = int((center_x - cam_center) / 10)
                LM = base_speed + gain
                RM = base_speed 
                msg.data = '%d %d\n' %(LM, RM)
                f.publisher_.publish(msg)
                f.get_logger().info('Publishing: "%s"' % msg.data)
            else:
                LM = base_speed
                RM = base_speed 
                msg.data = '%d %d\n' %(LM, RM)
                f.publisher_.publish(msg)
                f.get_logger().info('Publishing: "%s"' % msg.data)

 
        f.get_logger().info('Publishing: "%s"' % msg.data)
            
        cv.imshow("Snapshot", frame)
            
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
