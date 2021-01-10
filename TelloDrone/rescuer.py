#! /usr/bin/env python3

# Ioannis Broumas
# ioabro17@student.hh.se
# Dec 2020
# Remember OpenCV is BGR
# Go to sparkies last published position and try to locate him

from math import atan2, degrees
import math
import numpy as np
from tello import Tello
import socket
import time
import cv2 as cv
from cv2 import aruco

#ROS 2
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

# ros2 topic pub 'missing' 'std_msgs/String' '{data: 700;700} -1'

Base = [ 1800, 1700 ] # mm

Sparkie_ID = 15 # Change accordingly

# Constant parameters used in Aruco methods
ARUCO_PARAMETERS = aruco.DetectorParameters_create()
ARUCO_DICT = aruco.Dictionary_get(aruco.DICT_6X6_250)

def angle_between(p1, p2, p3):
    x1, y1 = p1
    x2, y2 = p2
    x3, y3 = p3
    deg1 = (360 + degrees(atan2(x1 - x2, y1 - y2))) % 360
    deg2 = (360 + degrees(atan2(x3 - x2, y3 - y2))) % 360
    if deg1 <= deg2:
        return deg2 - deg1 
    else:
        return 360 - (deg1 - deg2)

# The orientation of the drone maybe different
# Find quickest way to achieve desired yaw angle
def shortestTurn(start, to):
    if start < to: # from < to
        dif1 = to - start
        dif2 = (360 - to) + start
        if dif1 <= dif2:
            return dif1
        else:
            return -dif2
    else: # from > to
        dif1 = start - to
        dif2 = (360 - start) + to
        if dif1 < dif2:
            return -dif1
        else:
            return dif2

# Calculate distance to be travelled and desired yaw
# Inverse kinematics for a 2-joint robot arm using geometry
def getRoute(sx, sy, gx, gy):
    R = math.sqrt(gx**2 + gy**2)
    R_theta = math.degrees( math.acos(gx/R) )

    a1 = math.sqrt( (sx**2) + (sy**2) )
    a1_theta = math.degrees( math.acos(sx/a1) )

    dX = (gx - sx)
    dY = (gy - sy)
    a2 = math.sqrt( (dX**2) + (dY**2) )
    a2_theta = math.degrees( math.acos(dX/a2) )

    if a1 == 0:
        return R, int(R_theta)

    # Same line
    if a1_theta == a2_theta:
        # going back
        if gx <= sx:
            return a2, int(360 - a2_theta)
        # going forward
        else:
            return a2, int(a2_theta)

    # The cosine rule
    q2 = math.degrees( math.acos( (gx**2 + gy**2 - a1**2 - a2**2) / (2*a1*a2) ) )
    # q2 = int(q2)

    # Check which solution applies
    if a1_theta > R_theta:
        return a2, int((360 + a1_theta - q2) % 360)
    else:
        return a2, int((a1_theta + q2))


class SuperDrone(Node):

    def __init__(self):
        super().__init__('drone')
        self.subscription = self.create_subscription(
            String,
            'missing',
            self.listener_callback,
            10)
        self.drone = Tello('', 8889)
        self.X = Base[0]
        self.Y = Base[1]
        self.Z = 0
        self.yaw = 0
        self.found = False
        self.xoffset = 0
        self.yoffset = 0
        self.get_logger().info('Node Rescuer initialized!')
    
    def Turn(self, gx, gy, theta):

        if gx == self.X and gy == self.Y:
            pass

        elif gx == self.X and gy > self.Y:
            angle = shortestTurn(self.yaw, 90)
            if angle >= 0:
                angle = int(math.fabs(angle))
                self.drone.rotate_ccw(angle)
            else:
                angle = math.fabs(angle)
                self.drone.rotate_cw(angle)

        elif gx == self.X and gy < self.Y:
            angle = shortestTurn(self.yaw, 270)
            if angle >= 0:
                angle = int(math.fabs(angle))
                self.drone.rotate_ccw(angle)
            else:
                angle = int(math.fabs(angle))
                self.drone.rotate_cw(angle)

        elif gy == self.Y and gx > self.X:
            angle = shortestTurn(self.yaw, 0)
            if angle >= 0:
                angle = int(math.fabs(angle))
                self.drone.rotate_ccw(angle)
            else:
                angle = int(math.fabs(angle))
                self.drone.rotate_cw(angle)

        elif gy == self.Y and gx < self.X:
            angle = shortestTurn(self.yaw, 180)
            if angle >= 0:
                angle = int(math.fabs(angle))
                self.drone.rotate_ccw(angle)
            else:
                angle = int(math.fabs(angle))
                self.drone.rotate_cw(angle)

        else:
            angle = shortestTurn(self.yaw, theta)
            if angle >= 0:
                angle = int(math.fabs(angle))
                self.drone.rotate_ccw(angle)
            else:
                angle = int(math.fabs(angle))
                self.drone.rotate_cw(angle)


    def GoToPoint(self, gx, gy):
        distance, q2 = getRoute(self.X, self.Y, gx, gy)
        distance = distance/1000
        print("Distance", distance)
        self.Turn(gx, gy, q2)
        time.sleep(5)
        self.yaw = q2
        # Go to point
        self.drone.move_forward(distance)
        time.sleep(5)
        self.X = gx
        self.Y = gy
        print('SuperDrone @ x: %f y: %f' % (self.X, self.Y))

    def detect(self):
        self.found = False
        frame = None
        # Capture a frame of our video stream
        while frame is None:
            frame = self.drone.read()
        # frame = self.drone.read()
        height, width, layers =  frame.shape
        cam_center_x = width / 2
        cam_center_y = height / 2
        # If found, add object points, image points
        if frame is not None:
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
                        self.xoffset = int(cx - cam_center_x)
                        self.yoffset = int(cam_center_y - cy)
                        self.found = True                             
                        frame = aruco.drawDetectedMarkers(frame, corners, borderColor=(0, 0, 255))
            # Show the frame
            cv.imshow('Tello', frame)
            cv.waitKey(10)

    def approach(self):
        distance = 100
        while self.found:
            self.detect()
            if self.xoffset < -distance:
                # cmd = "counter_clockwise"
                self.drone.move_left(0.2)
                self.update_pos_b(- 200)
                time.sleep(0.5)
            elif self.xoffset > distance:
                # cmd = "clockwise"
                self.drone.move_right(0.2)
                self.update_pos_b(200)
                time.sleep(0.5)
            elif self.yoffset < -distance:
                # cmd = "down"
                self.drone.move_backward(0.2)
                self.update_pos(- 200)
                time.sleep(0.5)
            elif self.yoffset > distance:
                # cmd = "up"
                self.drone.move_forward(0.2)
                self.update_pos(200)
                time.sleep(0.5)
            else:
                break
        if self.found:
            print("Sparkie Found!")
            print("Location")
            print("X: ", self.X)
            print("Y: ", self.Y)
            # self.GoToPoint(Base[0], Base[1])
            self.drone.land()
            time.sleep(3)
            cv.destroyAllWindows()
            exit(0)
        else:
            print("Fix approach!")
            self.drone.land()
            time.sleep(3)
            cv.destroyAllWindows()
            exit(0)
    
    def update_pos(self, distance):
        if self.yaw == 90:
            self.Y = self.Y + distance
        elif self.yaw == 270:
            self.Y = self.Y - distance
        elif self.yaw == 180:
            self.X = self.X - distance
        else:
            self.X = self.X + distance

    def update_pos_b(self, distance):
        if self.yaw == 90:
            self.X = self.X + distance
        elif self.yaw == 270:
            self.X = self.X - distance
        elif self.yaw == 180:
            self.Y = self.Y + distance
        else:
            self.Y = self.Y - distance
        
    def listener_callback(self, msg):
        data = msg.data.split(sep=";")
        # Validate data
        if len(data) != 2:
            print("Invalid data!")
            return
        if not data[0].isdigit() or not data[1].isdigit():
            print("Invalid data!")
            return

        x = int(data[0])
        y = int(data[1])

        if x > 3635 or y > 2425:
            print("Invalid input data!")
            return

        # Check remaining battery
        battery = self.drone.get_battery()
        if 20 >= battery:
            print("Low Battery -%d-" %battery)
            print("Rescuer Drone Shutting Down!")
            cv.destroyAllWindows()
            self.drone.land()
            self.drone.shutdown()
            self.destroy_node()
            rclpy.shutdown()

        distance, q2 = getRoute(self.X, self.Y, x, y)

        # Takeoff
        res = self.drone.takeoff()
        time.sleep(7)
        self.Z = self.drone.get_height() / 100
        time.sleep(1)
        # print(self.Z)
        print('SuperDrone on air!')
        print('SuperDrone @ x: %f y: %f' % (self.X, self.Y))

        # Go to point
        self.GoToPoint(x, y)
        time.sleep(5)
        self.detect()
        time.sleep(1)
        if self.found:
            self.approach()

        # Go 0.5 m previous to point
        self.GoToPoint(x - 500, y)
        time.sleep(3)
        self.detect()
        time.sleep(1)
        if self.found:
            self.approach()

        # Go to top left corner of the square
        self.drone.rotate_cw(90)
        time.sleep(3)
        self.yaw = (360 + self.yaw - 90) % 360
        self.detect()
        time.sleep(1)
        if self.found:
            self.approach()
        self.drone.move_forward(0.5)
        time.sleep(3)
        self.update_pos(500)
        self.detect()
        time.sleep(1)
        if self.found:
            self.approach()
        self.drone.rotate_cw(90)
        time.sleep(3)
        self.yaw = (360 + self.yaw - 90) % 360
        self.detect()
        time.sleep(1)
        if self.found:
            self.approach()

        # Do a square
        for i in range(4):
            for j in range(2):
                self.drone.move_forward(0.5)
                time.sleep(3)
                self.update_pos(500)
                self.detect()
                time.sleep(1)
                if self.found:
                    self.approach()    
            self.drone.rotate_cw(90)
            time.sleep(3)
            self.yaw = (360 + self.yaw - 90) % 360
            self.detect()
            time.sleep(1)
            if self.found:
                self.approach()

        self.drone.land()
        print('SuperDrone has landed!')



def main(args=None):
    rclpy.init(args=args)
    d = SuperDrone()
    msg = "900;1115"
    d.drone.send_command('command')
    time.sleep(1)

    d.drone.stream_on()
    time.sleep(1)

    cv.namedWindow('Tello', cv.WINDOW_NORMAL)

    rclpy.spin(d)
    # Destroy the node explicitly
    cv.destroyAllWindows()
    d.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
