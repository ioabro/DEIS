#! /bin/python3

# Christo George
# christogeorge@live.in
# Nov 2020

import time
import datetime
import math
import threading
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
    

class Tele(Node):
    def __init__(self):
        super().__init__('teleop_drone') # name
        self.publisher_ = self.create_publisher(String, 'DRONE_ACTIONS', 10)
        #self.subscription  # prevent unused variable warning
        self.get_logger().info('Node init finished')

def main(args=None):
    rclpy.init(args=args)
    t = Tele()
    old_settings = termios.tcgetattr(sys.stdin)
    command = 'command'
    keyTimeout = 2

    msg = String()
    
    try:
        while True:
            key = getKey(keyTimeout)
            print(key)
            if key == "w": # Forward
                command = 'forward 40'
            elif key == "s": # Reverse
                command = 'back 40'
            elif key == "d": # Right 
                command = 'right 40' 
            elif key == "a": # Left
                command = 'left 40'
            elif key == "r": # flip Right
                command = 'flip r'
            elif key == "q": # flip Left
                command = 'flip l'
            elif key == "t":
                command = 'takeoff'
            elif key == "l":
                command = 'land'
            elif key == "v":
                command = "view"
            elif key == "b":
                command = "stop_view"
            elif key == '\x03': #exit key 
                break
            else:
                command = ''
            
            if('' != command):
                msg.data = command
                t.publisher_.publish(msg)
                t.get_logger().info('Publishing: "%s"' % msg.data)
    
    except KeyboardInterrupt:
        print("KeyboardInterrupt")
    
    finally:
        t.destroy_node()
        rclpy.shutdown()
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)

if __name__ == '__main__':
    main()

