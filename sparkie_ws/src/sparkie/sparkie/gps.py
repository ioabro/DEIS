#! usr/bin/env/python3

# Ioannis Broumas
# ioabro17@student.hh.se
# Nov 2020

from math import floor

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Point

class GPS(Node):

    def __init__(self):
        super().__init__('gps')
        self.publisher_ = self.create_publisher(Point, 'GPS', 10)
        self.subscription = self.create_subscription(
            String,
            '/robotPositions',
            self.listener_callback,
            10)
        self.get_logger().info('Node GPS initialized!')

    def listener_callback(self, msg):
        timestamp = self.get_clock().now().to_msg()
        data = msg.data.strip("[']")
        data = data.split(sep=";")
        data = data[0] # Group1 should be first, change accordingly
        point = Point()
        point.x = X
        point.y = Y
        point.z = 0
        self.publisher_(point)



def main(args=None):
    rclpy.init(args=args)
    g = GPS()
    rclpy.spin(g)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    g.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
