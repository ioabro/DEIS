#! /usr/bin/env python3

# Ioannis Broumas
# ioabro17@student.hh.se
# Dec 2020
# Keep track of sparkie's position
# notify if no position received for > 3 sec

import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Twist

class Notifier(Node):

    def __init__(self):
        super().__init__('notifier')
        self.publisher_ = self.create_publisher(String, 'missing', 10)
        timer_period = 3.0 # seconds
        self.timer = self.create_timer(timer_period, self.notify_callback)
        self.subscription_positions = self.create_subscription(
            Odometry,
            '/odom',
            self.position_callback,
            10)
        self.last_time = None
        self.last_X = None
        self.last_Y = None
        self.get_logger().info('Node Notifier initialized!')
        
    # Receive position
    def position_callback(self, msg):
        self.last_time = time.time()
        self.last_X = msg.pose.pose.position.x
        self.last_Y = msg.pose.pose.position.y
        self.get_logger().info("Got cordinates %f %f" %(self.last_X, self.last_Y))

    # Notify drone    
    def notify_callback(self):
        timestamp = time.time()
        # If no position has been received
        if self.last_X == None:
            return
        # If no position has been received for > 3 sec
        if timestamp - self.last_time >= 3.0:
            msg = String()
            # Convert to mm
            msg.data = str(int(self.last_X*1000)) + ";" + str(int(self.last_Y*1000))
            self.publisher_.publish(msg)
            self.get_logger().info("Notified drone to fly to %f %f" %(self.last_X, self.last_Y))
            self.destroy_node()
            rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    n = Notifier()
    rclpy.spin(n)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    n.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()