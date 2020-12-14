#! /usr/bin/env python3

# Ioannis Broumas
# ioabro17@student.hh.se
# Dec 2020
# Remember OpenCV is BGR

import math
import numpy as np
import time
from tello import Tello
import socket

#ROS 2
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

# ros2 topic pub 'emergency' 'std_msgs/String' '{data:700;700}'

Base = [ 1800, 1700 ] # mm
Akut = [ 2900, 600 ] # mm

def angle_between(p1, p2, p3):
    x1, y1 = p1
    x2, y2 = p2
    x3, y3 = p3
    deg1 = (360 + math.degrees(math.atan2(x1 - x2, y1 - y2))) % 360
    deg2 = (360 + math.degrees(math.atan2(x3 - x2, y3 - y2))) % 360
    if deg1 <= deg2:
        return deg2 - deg1 
    else:
        return 360 - (deg1 - deg2)


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
        return a2, int(a1_theta + q2)


class SuperDrone(Node):

    def __init__(self):
        super().__init__('drone')
        self.subscription = self.create_subscription(
            String,
            'emergency',
            self.listener_callback,
            10)

        self.drone = Tello('', 8889)

        self.X = Base[0]
        self.Y = Base[1]
        self.Z = 0
        self.yaw = 0

        self.get_logger().info('Node SuperDrone initialized!')
    
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
    
    def listener_callback(self, msg):
        timestamp = self.get_clock().now().to_msg()
        data = msg.data.strip("[']")
        data = data.split(sep=";")
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
            print("Invalid data!")
            return

        x = int(data[0])
        y = int(data[1])

        distance, q2 = getRoute(self.X, self.Y, x, y)

        # Takeoff
        res = self.drone.takeoff()
        time.sleep(7)
        self.Z = self.drone.get_height() / 100
        time.sleep(3)
        # print(self.Z)
        print('SuperDrone on air!')
        print('SuperDrone @ x: %f y: %f' % (self.X, self.Y))

        # Go to point
        self.GoToPoint(x, y)
        time.sleep(5)
        # Lower down - Load the first one
        self.drone.move_down(self.Z / 2)
        self.Z = self.drone.get_height() / 100
        print('SuperDrone loading!')
        time.sleep(3)
        # Fly up again
        self.drone.move_up(self.Z / 2)
        time.sleep(3)
        self.Z = self.drone.get_height() / 100
        time.sleep(3)
        # Go to Akut
        self.GoToPoint(Akut[0], Akut[1])
        time.sleep(3)
        # Lower down - Unload the first one
        self.drone.move_down(self.Z / 2)
        time.sleep(3)
        self.Z = self.drone.get_height() / 100
        print('SuperDrone unloading!')
        time.sleep(3)
        # Fly up again
        self.drone.move_up(self.Z / 2)
        time.sleep(3)
        self.Z = self.drone.get_height() / 100
        time.sleep(3)
        # # Go to point
        # self.GoToPoint(x, y)
        # time.sleep(3)
        # # Lower down - Load the second
        # self.drone.move_down(self.Z / 2)
        # time.sleep(3)
        # self.Z = self.drone.get_height() / 100
        # print('SuperDrone loading!')
        # time.sleep(3)
        # # Fly up again
        # self.drone.move_up(self.Z / 2)
        # time.sleep(3)
        # self.Z = self.drone.get_height() / 100
        # time.sleep(3)
        # # Go to Akut
        # self.GoToPoint(Akut[0], Akut[1])
        # time.sleep(3)
        # # Lower down - Unload the second
        # self.drone.move_down(self.Z / 2)
        # time.sleep(3)
        # self.Z = self.drone.get_height() / 100
        # print('SuperDrone unloading!')
        # time.sleep(3)
        # # Fly up again
        # self.drone.move_up(self.Z / 2)
        # time.sleep(3)
        # self.Z = self.drone.get_height() / 100
        # time.sleep(3)
        # Return to Base
        self.GoToPoint(Base[0], Base[1])
        time.sleep(3)
        # Land
        self.drone.land()
        print('SuperDrone has landed!')



def main(args=None):
    rclpy.init(args=args)
    d = SuperDrone()
    rclpy.spin(d)
    # Destroy the node explicitly
    d.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
