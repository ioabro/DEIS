#! usr/bin/env/python3

# Ioannis Broumas
# ioabro17@student.hh.se
# Nov 2020

import time
import datetime
from math import cos, sin, pi

#ROS 2
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist, Vector3, Quaternion
import tf2_ros

def euler_to_quaternion(roll, pitch, yaw):
  qx = sin(roll/2) * cos(pitch/2) * cos(yaw/2) - cos(roll/2) * sin(pitch/2) * sin(yaw/2)
  qy = cos(roll/2) * sin(pitch/2) * cos(yaw/2) + sin(roll/2) * cos(pitch/2) * sin(yaw/2)
  qz = cos(roll/2) * cos(pitch/2) * sin(yaw/2) - sin(roll/2) * sin(pitch/2) * cos(yaw/2)
  qw = cos(roll/2) * cos(pitch/2) * cos(yaw/2) + sin(roll/2) * sin(pitch/2) * sin(yaw/2)
  return Quaternion(x=qx, y=qy, z=qz, w=qw)

class IMU(Node):

    def __init__(self):
        super().__init__('imu') # name
        self.publisher_imu = self.create_publisher(Imu, 'imu_dcm', 10)
        self.subscription = self.create_subscription(
            String,
            'imu_r',
            self.listener_callback,
            10)
        self.get_logger().info('Node IMU initialized!')
    

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)
        data = msg.data.split(sep="_")
        current_time = self.get_clock().now().to_msg()
        #current_time = data[0]

        imu = Imu()
        imu.header.frame_id = "imu"
        imu.header.stamp = current_time

        imu.linear_acceleration.x = float(data[0])
        imu.linear_acceleration.y = float(data[1])
        imu.linear_acceleration.z = float(data[2])

        imu.angular_velocity.x = float(data[3])
        imu.angular_velocity.y = float(data[4])
        imu.angular_velocity.z = float(data[5])

        imu.orientation = euler_to_quaternion(float(data[8]), float(data[7]), float(data[6])) # Or euler_to_quaternion(0, 0, data[7]) ?

        self.publisher_imu.publish(imu)
        

def main(args=None):
    rclpy.init(args=args)
    i = IMU()
    rclpy.spin(i)
    i.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
