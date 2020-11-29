#! usr/bin/env/python3

# Ioannis Broumas
# ioabro17@student.hh.se
# Nov 2020

import time
import datetime
from math import cos, sin, pi
import numpy as np

#ROS 2
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Vector3, Quaternion, TransformStamped
import tf2_ros
from rclpy.qos import QoSProfile

def euler_to_quaternion(roll, pitch, yaw):
  qx = sin(roll/2) * cos(pitch/2) * cos(yaw/2) - cos(roll/2) * sin(pitch/2) * sin(yaw/2)
  qy = cos(roll/2) * sin(pitch/2) * cos(yaw/2) + sin(roll/2) * cos(pitch/2) * sin(yaw/2)
  qz = cos(roll/2) * cos(pitch/2) * sin(yaw/2) - sin(roll/2) * sin(pitch/2) * cos(yaw/2)
  qw = cos(roll/2) * cos(pitch/2) * cos(yaw/2) + sin(roll/2) * sin(pitch/2) * sin(yaw/2)
  return Quaternion(x=qx, y=qy, z=qz, w=qw)


class Odometer(Node):

    def __init__(self):
        super().__init__('odometer') # name
        self.publisher_odom = self.create_publisher(Odometry, '/odom', 10)
        self.subscription = self.create_subscription(
            String,
            'odom_raw',
            self.listener_callback,
            10)
        # self.odom_broadcaster = tf2_ros.TransformBroadcaster()
        qos_profile = QoSProfile(depth=50)
        self.odom_broadcaster = tf2_ros.TransformBroadcaster(self,qos=qos_profile)

        self.current_time = self.get_clock().now().to_msg()
        self.last_time = None
        self.get_logger().info('Node Odometer initialized!')

        # Robot initialization
        self.WHEEL_BASE = 0.12
        self.WHEEL_DIAMETER = 0.065 # (m)
        self.PULSES_PER_REVOLUTION = 192 # ticks per wheel revolution
        self.MM_PER_PULSE = pi*self.WHEEL_DIAMETER / self.PULSES_PER_REVOLUTION; 
        self.SIGMA_WHEEL_ENCODER = 0.5/12;   # The error in the encoder is 0.5mm / 12mm travelled
        # Use the same uncertainty in both of the wheel encoders
        self.SIGMAl = self.SIGMA_WHEEL_ENCODER
        self.SIGMAr = self.SIGMA_WHEEL_ENCODER
        # calculate the variances
        self.SIGMAD = (self.SIGMAr**2 + self.SIGMAl**2) / 4
        self.SIGMAdA = (self.SIGMAr**2 + self.SIGMAl**2) / self.WHEEL_BASE**2
        # Init Robot Position, i.e. (0, 0, 90*pi/180) and the Robots Uncertainty
        self.X = 0
        self.Y = 0
        self.A = 0 # 90*pi / 180
        # Uncertainty in state variables [3x3]
        self.P = [ [1, 0, 0], [0, 1, 0], [0, 0, (1*pi/180)**2] ]

    # Do KF also
    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)
        data = msg.data.split(sep="_")
        # Validate data
        if len(data) != 2:
            self.get_logger().info('Missed Data!')
            return
        if not data[0].lstrip('-').isdigit() and not data[1].lstrip('-').isdigit():
            self.get_logger().info('Missed Data!')
            return
        left_ticks = int(data[0])
        right_ticks = -int(data[1])
        # Transform encoder values (pulses) into distance travelled by the wheels (mm)
        # Change of wheel displacements, i.e displacement of left and right wheels
        dDr = left_ticks * self.MM_PER_PULSE
        dDl = right_ticks * self.MM_PER_PULSE
        if dDl < 300 and dDr < 300: # Threshold reject outliers
            self.current_time = self.get_clock().now().to_msg()
            if self.last_time == None:
                dt = 0.1
            else:
                dt = self.current_time - self.last_time
            self.last_time = self.current_time
            # The changes in the forward direction (δd) and heading (δθ)
            dD = (dDr + dDl) / 2 
            dA = (dDr - dDl) / self.WHEEL_BASE
            # Calculate the change in X and Y (World co-ordinates)    
            dX = dD*cos( self.A + dA/2 )*dt
            dY = dD*sin( self.A + dA/2 )*dt
            # Predict the new state variables (World co-ordinates)
            self.X = self.X + dX
            self.Y = self.Y + dY
            self.A = (self.A + dA) % (2*pi)
            
            # Predict the new uncertainty in the state variables (Error prediction)
            # Cxya_old = [ P[0], P[1], P[2] ]   # Uncertainty in state variables at time k-1 [3x3]
            # Uncertainty in the input variables [2x2]
            Cu = [ [(self.SIGMAl**2 + self.SIGMAr**2)/4, 0 ],
                [0, ((self.SIGMAl**2 + self.SIGMAr**2)/(self.WHEEL_BASE**2))] ]
            # Jacobian matrix w.r.t. X, Y and A [3x3]
            Axya = [ [1, 0, -dD*sin( self.A + dA/2 )],
                    [0, 1, dD*cos( self.A + dA/2 )],
                    [0, 0, 1] ] 
            # Jacobian matrix w.r.t. dD and dA [3x2]
            Au = [ [cos( self.A + dA/2 ), -dD*sin( self.A + dA/2 )/2],
                [sin( self.A + dA/2 ), dD*cos( self.A + dA/2 )/2],
                [0, 1] ]   
            # Use the law of error predictions, which gives the new uncertainty
            # Cxya_new = Axya*Cxya_old*Axya' + Au*Cu*Au'
            #self.P = np.matmul(np.matmul(Axya,self.P), np.inv(P)) + np.matmul(np.matmul(Au, Cu), np.inv(Au))
            # Store the new co-variance matrix
            #P(kk,1:9) = [Cxya_new(1,1:3) Cxya_new(2,1:3) Cxya_new(3,1:3)]

            # since all odometry is 6DOF we'll need a quaternion created from yaw
            # Quaternion odom_quat = euler_to_quaternion(0, 0, self.A)

            # first, we'll publish the transform over tf
            t = TransformStamped() # create a TransformStamped message that we will send out over tf
            t.header.stamp = current_time
            t.header.frame_id = "odom"
            t.child_frame_id = "base_link"

            t.transform.translation.x = self.X
            t.transform.translation.y = self.Y
            t.transform.translation.z = 0.0
            t.transform.rotation = euler_to_quaternion(0, 0, self.A)

            # send the transform
            self.odom_broadcaster.sendTransform(t)

            # next, we'll publish the odometry message over ROS
            odom = Odometry()
            odom.header.frame_id = "odom"
            odom.header.stamp = current_time

            # set the position
            odom.pose.pose.position.x = self.X
            odom.pose.pose.position.y = self.Y
            odom.pose.pose.position.z = 0.0
            
            odom.pose.pose.orientation = euler_to_quaternion(0, 0, self.A)
            
            # set the velocity
            odom.child_frame_id = "base_link"
            odom.twist.twist.linear.x = dD
            odom.twist.twist.linear.y = .0
            odom.twist.twist.angular.z = dA

            # publish the message
            self.publisher_odom.publish(odom)


def main(args=None):
    rclpy.init(args=args)
    o = Odometer()
    rclpy.spin(o)
    o.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
