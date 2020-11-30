#! usr/bin/env/python3

# Ioannis Broumas
# ioabro17@student.hh.se
# Nov 2020

import math

# ROS2
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PointStamped

Landmarks = [ (0, 0), (0, 0), (0, 0) ]

def circleCircle( z, id1, R1, id2, R2, id3, R3):
    # Maybe should add validation if z > R
    r1 =  math.sqrt(R1**2 - z**2)
    r2 =  math.sqrt(R2**2 - z**2)
    r3 =  math.sqrt(R3**2 - z**2)

    # dx and dy are the vertical and horizontal distances between
    # the circle centers.
    dx = Landmarks[id1][0] - Landmarks[id2][0]
    dy = Landmarks[id1][1] - Landmarks[id2][1]

    # Straight line distance between the centers.
    d = math.sqrt((dx**2) + (dy**2))

    # Check if we have gone totally wrong.
    if d == 0:
        # concentric circles
        return None

    if (d > (r1 + r2)):
        # no solution. circles do not intersect.
        return None

    if (d < math.abs(r1 - r2)):
        # no solution. one circle is contained in the other.
        return None

    # 'P' is the point where the line joining the circle
    # intersection points crosses the line joining the circle centers.

    # Distance from point 0 to P.
    a = ((r1*r1) - (r2*r2) + (d*d)) / (2.0 * d)

    # Coordinates of P.
    P_x = Landmarks[id1][0] + (dx * a/d)
    P_y = Landmarks[id1][1] + (dy * a/d)

    # Distance from P to either of the intersection points.
    h = math.sqrt((r1*r1) - (a*a))

    if h == 0:
        # only one intersection point
        return [P_x, P_y, z]

    # Offsets of the intersection points from P.
    rx = -dy * (h/d)
    ry = dx * (h/d)

    # Absolute intersection points.
    intersectionPoint1_x = P_x + rx
    intersectionPoint2_x = P_x - rx
    intersectionPoint1_y = P_y + ry
    intersectionPoint2_y = P_y - ry
    
    # Lets determine if circle 3 intersects at either of the above intersection points.
    dx = intersectionPoint1_x - Landmarks[id3][0] 
    dy = intersectionPoint1_y - Landmarks[id3][1]
    d1 = math.sqrt((dy*dy) + (dx*dx))

    if d1 == r3: # math.abs(d1 - r3) <= 3
        return [intersectionPoint1_x, intersectionPoint1_y, z]
    # dx = intersectionPoint2_x - x2
    # dy = intersectionPoint2_y - y2
    # d2 = math.sqrt((dy*dy) + (dx*dx))
    else:
        return [intersectionPoint2_x, intersectionPoint2_y, z]


class Triangulation(Node):

    def __init__(self):
        super().__init__('triangulation')
        self.publisher_ = self.create_publisher(String, 'telloXYZ', 10)
        self.subscription = self.create_subscription(
            String,
            'detected_ArUcos',
            self.listener_callback,
            10)
        self.get_logger().info('Node Triangulation initialized!')
    
    def listener_callback(self, msg):
        timestamp = self.get_clock().now().to_msg()
        data = data.split(sep="_")
        Location = circleCircle(data[0], data[1], data[2], data[3], data[4], data[5], data[6])
        if Location == None:
            # Something went wrong don't publish
            pass
        else:
            point = PointStamped()
            point.header.frame_id = "TelloXYZ"
            point.header.stamp = timestamp
            point.x = Location[0]
            point.y = Location[1]
            point.z = Location[2]
            self.publisher_(point)

def main(args=None):
    rclpy.init(args=args)
    t = Triangulation()
    rclpy.spin(t)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    t.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()