#! /bin/python3

# Ioannis Broumas
# ioabro17@student.hh.se
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
        super().__init__('tele') # name
        self.publisher_ = self.create_publisher(String, 'teleop', 10)
        self.get_logger().info('Node Tele_C initialized!')

class PublishThread(threading.Thread):
    def __init__(self, rate):
        super(PublishThread, self).__init__()
        #create a Tele node object. this thread uses this node to publish
        self.t = Tele()
        self.leftSpeed = 0
        self.rightSpeed = 0
        self.condition = threading.Condition()
        self.done = False

        if rate != 0.0:
            self.timeout = 1.0/rate
        else:
            self.timeout = None
        
        self.start()

    def update(self, leftSpeed, rightSpeed):
        self.condition.acquire()
        self.leftSpeed = leftSpeed
        self.rightSpeed = rightSpeed
        self.condition.notify()
        self.condition.release()
    
    def stop(self):
        self.update(0,0)
        self.t.destroy_node()
        rclpy.shutdown()
        self.join()

    def run(self):
        msg = String()
        while not self.done:
            self.condition.acquire()
            self.condition.wait(self.timeout)
            msg.data = '%s %s \n' % (self.leftSpeed, self.rightSpeed)
            self.condition.release()
            self.t.publisher_.publish(msg)
        
        msg.data = '0 0 \n'
        self.t.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    old_settings = termios.tcgetattr(sys.stdin)
    repeat = 3
    leftSpeed = 0
    rightSpeed = 0
    keyTimeout = 2
    pub_thread = PublishThread(repeat)

    try:
        pub_thread.update(leftSpeed, rightSpeed)
        while True:
            key = getKey(keyTimeout)
            print(key)
            if key == "8": # Forward
                leftSpeed = 255
                rightSpeed = 255
            elif key == "2": # Reverse
                leftSpeed = -255 
                rightSpeed = -255
            elif key == "6": # Right 
                leftSpeed  = 200 
                rightSpeed = -100
            elif key == "4": # Left
                leftSpeed = -100 
                rightSpeed = 200
            elif key == "9": # Spin Right
                leftSpeed = 90 
                rightSpeed = -90
            elif key == "7": # Spin Left
                leftSpeed = -90 
                rightSpeed = 90
            elif key == '\x03': #exit key 
                break
            else:
                leftSpeed = 0
                rightSpeed = 0
            pub_thread.update(leftSpeed, rightSpeed)
    
    except KeyboardInterrupt:
        print("KeyboardInterrupt")
    
    finally:
        pub_thread.stop()
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)

if __name__ == '__main__':
    main()

