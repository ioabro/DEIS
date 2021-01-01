#! usr/bin/env/python3

# Ioannis Broumas
# ioabro17@student.hh.se
# Nov 2020
# Trilateration with circle intersection method,
# algebraic solution and least squares solution

import math
import itertools
import numpy as np
from numpy.linalg import lstsq

# import scipy.optimize

# ROS2
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Point

VerticalDistance = 242.5 # cm
HorizontalDistance = 363.5 # cm

Landmarks = { 1 : (360.5, 121.5), 2 : (138.5, 143.0), 91 : (221.5, 143.0), 92 : (338.5, 143.0),
              3 : (91.5, 96.5), 4 : (138.5, 96.5), 93 : (221.5, 96.5), 94 : (338.5, 96.5) }

# Least Square
def lsq(o1, r1, o2, r2, o3, r3):
    # 2(xn − x1) 2(yn − y1)
    A = [ [ 2*(o2[0] - o1[0] ), 2*(o2[1] - o1[1]) ],
          [ 2*(o3[0] - o1[0] ), 2*(o3[1] - o1[1]) ] ]
    # (r12 − rn2 − x12 − y12+ xn2+ yn2)
    b = [ [r1**2 -  r2**2 - o1[0]**2 - o1[1]**2 + o2[0]**2 + o2[1]**2],
          [r1**2 -  r3**2 - o1[0]**2 - o1[1]**2 + o3[0]**2 + o3[1]**2] ]
    solution = lstsq(A, b, rcond=None)[0]
    # print(solution)
    # x = (ATA)-1ATb
    # A = np.array(A)
    # b = np.array(b)
    # x = np.linalg.inv(np.matmul(np.transpose(A),A))
    # y = np.matmul(np.transpose(A),b)
    # x = np.matmul(x,y)
    # print(x)
    x = solution[0]
    y = solution[1]
    return (x,y)

# Perpendicular point
def perp(t):
    x, y = t[0], t[1]
    return -y, x

# Power of 2
def sq(p):
    return p[0]*p[0] + p[1]*p[1]  

# Circle circle intersection
def cci(o1, r1, o2, r2, o3, r3):
    d = o2[0]-o1[0], o2[1]-o1[1]
    d2 = sq(d)
    if (d2 == 0):
        # {assert(r1 != r2); 
        return None # concentric circles
    pd = (d2 + r1*r1 - r2*r2)/2 # = |O_1P| * d
    h2 = r1*r1 - pd*pd/d2 # = hˆ2
    if (h2 >= 0):
        p = o1[0] + d[0]*pd/d2, o1[0] + d[1]*pd/d2
        h = perp(d)[0]*math.sqrt(h2/d2), perp(d)[1]*math.sqrt(h2/d2)
        out = p[0]-h[0], p[1]-h[1], p[0]+h[0], p[1]+h[1]
        p1 = (p[0]-h[0], p[1]-h[1])
        p2 = (p[0]+h[0], p[1]+h[1])
        # Lets determine if circle 3 intersects at either of the above intersection points.
        dx = p1[0] - o3[0] 
        dy = p1[1] - o3[1]
        d1 = math.sqrt((dy*dy) + (dx*dx))

        dx = p2[0] - o3[0] 
        dy = p2[1] - o3[1]
        d2 = math.sqrt((dy*dy) + (dx*dx))

        Dd1 = math.fabs(d1 - r3)
        Dd2 = math.fabs(d2 - r3)
        if Dd1 < Dd2:
            return p1
        else:
            return p2

# substitution solution
def substitution(o1, r1, o2, r2, o3, r3):
    A = -2*o1[0] + 2*o2[0] # (−2x1+2x2) 
    B = -2*o1[1] + 2*o2[1] # (−2y1+2y2)
    C = r1**2 -r2**2 - o2[0]**2 + o2[0]**2 - o1[1]**2 + o2[1]**2 # r21−r22−x21+x22−y21+y22
    D = -2*o2[0] + 2*o3[0] # (−2x2+2x3)
    E = -2*o2[1] + 2*o3[1] # (−2y2+2y3)
    F = r2**2 -r3**2 - o2[0]**2 + o3[0]**2 - o2[1]**2 + o3[1]**2 # r22−r23−x22+x23−y22+y23
    x = (C*E - F*B) / (E*A - B*D)
    y = (C*D - A*F) / (B*D - A*E)
    # print("A", x, y)
    return (x, y)

# Mean Squarred Error
def mse(pred, positions, R):
    # Return high error if pred out of bounds
    # Comment out this if statement for sub and cci
    if pred[0] < 0 or pred[1] < 0 or pred[0] > 363.5 or pred[1] > 242.5:
        return 9999999
    mse = 0.0
    for pos, r in zip(positions, R):
        distance = math.hypot( pred[0] - pos[0], pred[1] - pos[1] )
        mse += math.pow(distance - r, 2.0)
    return mse/3


class Trilateration(Node):

    def __init__(self):
        super().__init__('trilateration')
        self.publisher_ = self.create_publisher(Point, 'sparkieXY', 10)
        self.subscription = self.create_subscription(
            String,
            'sparkieScanner',
            self.listener_callback,
            10)
        self.get_logger().info('Node Trilateration initialized!')
    
    def listener_callback(self, msg):
        data = msg.data.replace("[","")
        data = data.replace("]","")
        data = data.split(sep="_")

        if len(data) < 6:
            print("Wrong Data!")
            return

        o1 = Landmarks[int(data[0])]
        o2 = Landmarks[int(data[2])]
        o3 = Landmarks[int(data[4])]
        r1 = float(data[1])
        r2 = float(data[3])
        r3 = float(data[5])

        Location = None

        # Least Squares
        pred = lsq( o1, r1, o2, r2, o3, r3)

        # Minimize mse
        # positions = [ o1, o2, o3 ]
        # R = [ r1, r2, r3 ]
        # pred = scipy.optimize.minimize(mse, [0,0], args=(positions, R), method='Nelder-Mead')
        
        # # Uncommnet and choose cci or substitution
        # j = list(itertools.permutations([(o1, r1), (o2, r2), (o3, r3)],3))
        # min_error = 1000000
        # for c in j:
        #     pred = substitution(Landmarks[c[0][0]], c[0][1], Landmarks[c[1][0]], c[1][1], Landmarks[c[2][0]], c[2][1])
        #     if pred == None:
        #         continue
        #     if pred[0] < 0 or pred[1] < 0:
        #         continue
        #     if pred[0] > 363.5 or pred[1] > 242.5:
        #         continue
        #     positions = [ Landmarks[c[0][0]], Landmarks[c[1][0]], Landmarks[c[2][0]] ]
        #     R = [ c[0][1], c[1][1], c[2][1] ]
        #     error = mse(pred, positions, R)
        #     if error < min_error:
        #         Location = pred
        #         min_error = error

        if Location == None:
            print("Something went wrong don't publish.")
        else:
            # Convert to meters and publish
            point = Point()
            point.x = round(Location[0] / 100, 2)
            point.y = round(Location[1] / 100, 2)
            point.z = 0.0
            self.publisher_.publish(point)

def main(args=None):
    rclpy.init(args=args)
    t = Trilateration()
    rclpy.spin(t)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    t.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()