#! /usr/bin/env python3

# Ioannis Broumas
# ioabro17@student.hh.se
# Dec 2020
# Remember OpenCV is BGR

import numpy as np
import cv2 as cv
from cv2 import aruco

#ROS 2
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PointStamped

Sparkie_ID = 1 # Change accordingly
Drone_ID = 2 # Change accordingly

# Constant parameters used in Aruco methods
ARUCO_PARAMETERS = aruco.DetectorParameters_create()
ARUCO_DICT = aruco.Dictionary_get(aruco.DICT_6X6_250)

class Roof(Node):

    def __init__(self):
        super().__init__('roof')
        self.sparkie_publisher_ = self.create_publisher(PointStamped, 'sparkieGPS', 10)
        self.drone_publisher_ = self.create_publisher(PointStamped, 'droneGPS', 10)
        self.get_logger().info("Node Roof initialized!")

def main(args=None):
    rclpy.init(args=args)

    r = Roof()
    point = PointStamped()

    # IP camera on the ceiling
    cam = cv.VideoCapture('rtsp://192.168.1.2:554/axis.media/media.amp')

    if not cam.isOpened():
        print('--(!)Error opening video capture')
        r.destroy_node()
        rclpy.shutdown()

    HorizontalPixels = cam.get(3)
    # print(cam.get(3)) # Width - Horizontal
    VerticalPixels = cam.get(4)
    # print(cam.get(4)) # Height - Vertical
    # Resolution = HorizontalPixels * VerticalPixels
    VerticalDistance = 2425
    HorizontalDistance = 3635
    # Calculate mm to pixels
    y_mm_to_pixel = VerticalDistance / VerticalPixels
    x_mm_to_pixel = HorizontalDistance / HorizontalPixels

    while True:

        # Capturing each frame of our video stream
        ret, frame = cam.read()

        # If found, add object points, image points (after refining them)
        if ret == True:
            # grayscale image
            gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
        
            # Detect Aruco markers
            corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, ARUCO_DICT, parameters=ARUCO_PARAMETERS)

            if ids is not None:
                # Print corners and ids to the console
                for i, corner in zip(ids, corners):
                    if i == Sparkie_ID:
                        # Find X, Y of the center pixel
                        cx = (corner[0][0][0] + corner[0][1][0] + corner[0][2][0] + corner[0][3][0]) / 4
                        cy = (corner[0][0][1] + corner[0][1][1] + corner[0][2][1] + corner[0][3][1]) / 4
                        # Convert pixels to mm
                        X = x_mm_to_pixel * cx
                        Y = y_mm_to_pixel * cy
                        # Prepare and publish the point
                        point.header.frame_id = "sparkieGPS"
                        point.header.stamp = r.get_clock().now().to_msg()
                        point.x = X
                        point.y = Y
                        point.z = 0
                        r.sparkie_publisher_(point)
                        # Outline the detected marker in our image
                        frame = aruco.drawDetectedMarkers(frame, corner, borderColor=(0, 0, 255))
                    if i == Drone_ID:
                        # Find X, Y of the center pixel
                        cx = (corner[0][0][0] + corner[0][1][0] + corner[0][2][0] + corner[0][3][0]) / 4
                        cy = (corner[0][0][1] + corner[0][1][1] + corner[0][2][1] + corner[0][3][1]) / 4
                        # Convert pixels to mm
                        X = x_mm_to_pixel * cx
                        Y = y_mm_to_pixel * cy
                        # Prepare and publish the point
                        point.header.frame_id = "droneGPS"
                        point.header.stamp = r.get_clock().now().to_msg()
                        point.x = X
                        point.y = Y
                        point.z = 0
                        r.drone_publisher_(point)
                        # Outline the detected marker in our image
                        frame = aruco.drawDetectedMarkers(frame, corner, borderColor=(0, 0, 255))
            # Show the frame
            cv.imshow('A-GPS', frame)

            if cv.waitKey(100) == 27: # Wait for 10ms, if key == 27 (esc char) break
                break
                
    cam.release()
    cv.destroyAllWindows()
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    r.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
