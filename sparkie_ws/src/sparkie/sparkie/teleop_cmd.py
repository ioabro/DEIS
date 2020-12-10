#! usr/bin/env/python3

# Ioannis Broumas
# ioabro17@student.hh.se
# Dec 2020

import time
import datetime
from math import cos, sin, pi
import numpy as np

#ROS 2
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist, Vector3


class CMD(Node):

    def __init__(self):
        super().__init__('cmd') # name
        self.publisher_ = self.create_publisher(String, 'cmd', 10)
        self._cmd_vel_subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self._cmd_vel_callback,
            10)

        # Robot initialization
        self.WHEEL_BASE = 0.12
        self.WHEEL_DIAMETER = 0.065 # (m)
        self.PULSES_PER_REVOLUTION = 192 # ticks per wheel revolution
        self.MM_PER_PULSE = pi*self.WHEEL_DIAMETER / self.PULSES_PER_REVOLUTION

        self.speed = 0.0
        self.spin = 0.0

        self._left_max_rpm = 250 # The number of revolutions per minute made by the left motor when running at 100% power
        self._right_max_rpm = 250

        self.get_logger().info('Node CMD initialized!')


    def _cmd_vel_callback(self, msg):
        self.speed = msg.linear.x
        self.spin = msg.angular.z
        self._set_motor_speeds()

    def _set_motor_speeds(self):
        msg = String()
        # First figure out the speed of each wheel based on spin: each wheel covers 
        # self._wheel_base meters in one radian, so the target speed for each wheel
        # in meters per sec is spin (radians/sec) times wheel_base divided by
        # wheel_diameter
        #
        right_twist_mps = self.spin * self.WHEEL_BASE / self.WHEEL_DIAMETER
        left_twist_mps = -1.0 * self.spin * self.WHEEL_BASE / self.WHEEL_DIAMETER
        #
        # Now add in forward motion.
        # 
        left_mps = self.speed + left_twist_mps
        right_mps = self.speed + right_twist_mps
        #
        # Convert meters/sec into RPM: for each revolution, a wheel travels pi * diameter
        # meters, and each minute has 60 seconds.
        #
        left_target_rpm = (left_mps * 60.0) / (pi * self.WHEEL_DIAMETER)
        right_target_rpm = (right_mps * 60.0) / (pi * self.WHEEL_DIAMETER)
        #
        left_percentage = (left_target_rpm / self._left_max_rpm) * 100.0
        right_percentage = (right_target_rpm / self._right_max_rpm) * 100.0
        #
        # clip to +- 100%
        left_percentage = max (min (left_percentage, 100.0), -100.0)
        left_percentage = -int(round(left_percentage)) # added minus since motor cables are switched
        right_percentage = max (min (right_percentage, 100.0), -100.0)
        right_percentage = int(round(right_percentage))
        #
        msg.data = '%s %s \n' % (left_percentage, right_percentage)
        self.get_logger().info('Sending %s %s' % (left_percentage, right_percentage))
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    c = CMD()
    rclpy.spin(c)
    c.destroy_node()
    rclpy.shutdown()
    

if __name__ == '__main__':
    main()


    
