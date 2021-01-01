#! /bin/python3

# Ioannis Broumas
# ioabro17@student.hh.se
# Christo George
# christogeorge@live.in
# Nov 2020

import time
import datetime
import math

import sys, select, termios, tty, os

#ROS 2
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

def getKey(key_timeout):
	#store the old terminal settings
    old_settings = termios.tcgetattr(sys.stdin)
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], key_timeout)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
    return key
    
#can use this method also for getting the pressed key
'''
def getKey():
	old_settings = termios.tcgetattr(sys.stdin)
	tty.setcbreak(sys.stdin.fileno())
	try:
		while True:
			b = os.read(sys.stdin.fileno(), 3).decode()
			if len(b) == 3:
				k = ord(b[2])
			else:
				k = ord(b)
			return k
	finally:
		termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
'''	
class Tele(Node):

    def __init__(self):
        super().__init__('tele') # name
        self.publisher_ = self.create_publisher(String, 'teleop', 10)
        self.get_logger().info('Node Teleop initialized!')

def main(args=None):
    rclpy.init(args=args)
    t = Tele()
    msg = String()
    while True:
        key = getKey(0.3) # 0.3 time between
        print(key)
        if key == "8": # Forward
            # motors.drive(255); # forward
            msg.data = '70 70\n'
        elif key == "2": # Reverse
            # motors.drive(-255); 
            msg.data = '-80 -80\n'
        elif key == "6": # Right
            # motors.leftMotor(-200) 
            # motors.rightMotor(100) 
            msg.data = '120 60\n'
        elif key == "4": # Left
            # motors.leftMotor(-100) 
            # motors.rightMotor(200) 
            msg.data = '60 120\n'
        elif key == "9": # Spin Right
            msg.data = '-90 90\n'
        elif key == "7": # Spin Left
            msg.data = '90 -90\n'
        elif key == "Q" or key == "q": #exit key 
            break
        else:
            msg.data = '0 0\n'
        t.publisher_.publish(msg)
        t.get_logger().info('Publishing: "%s"' % msg.data)

    t.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
