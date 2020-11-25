# -*- coding: utf-8 -*-
"""
Created on Mon Nov 16 10:03:30 2020
@author: Meerashine Joe
@ Christo George christogeorge@live.in
"""

import rclpy
import threading
import cv2
from rclpy.node import Node
from deis_py_dev.tello import Tello
from std_msgs.msg import String
from PIL import Image

class droneActor(Node):

    def __init__(self):
        super().__init__('drone_actor')
        self.subscription = self.create_subscription(String, 'DRONE_ACTIONS', self.drone_actor_callback, 10)
        self.subscription  # prevent unused variable warning
        self.timer = self.create_timer(20, self.check_drone_battery)
        self.drone = Tello('', 8889)
        res = self.drone.send_command('command')
        self.get_logger().info("Init done")
        self.frame = None 
        self.stream_state = False
        #self.opencv_streamon()

    def __del__(self):
        if(self.stream_state == True):
            self.opencv_streamoff()
            cv2.destroyAllWindows()

    def drone_actor_callback(self, msg):
        self.get_logger().info('Got command: "%s"' % msg.data)
        if('' != msg.data):
            if('view' == msg_data):
                self.opencv_streamon()
            elif('stop_view' == msg.data):
                self.opencv_streamoff()
            else:
                self.drone.send_command(msg.data)
    
    def check_drone_battery(self):
        battery_level = self.drone.get_battery()
        self.get_logger().info("Battery level %s" % battery_level)
        if(battery_level == 'none_response' or battery_level == 'ok' or battery_level == 'error'):
            return
        if(int(battery_level) < 10):
            self.get_logger().error("Low battery landing")
            self.drone.send_command('land')
    
    def opencv_video_thread(self):
        self.get_logger().info("Opencv video thread started")
        # Runs while 'stream_state' is True
        while self.stream_state:
                        # read the frame for GUI show
                self.frame = self.drone.read()
                if self.frame is None or self.frame.size == 0:
                    continue 
                # transfer the format from frame to image         
                image = Image.fromarray(self.frame)
                cv2.imshow('Group 1 - Air surveillance', self.frame)

                # Video Stream is closed if escape key is pressed
                k = cv2.waitKey(1) & 0xFF
                if k == 27:
                    break
        cv2.destroyAllWindows()


    def opencv_streamon(self):
        self.stream_state = True
        self.video_thread = threading.Thread(target=self.opencv_video_thread)
        self.video_thread.daemon = True
        self.video_thread.start()
   
    def opencv_streamoff(self):
        self.stream_state = False

def main(args=None):
    rclpy.init(args=args)
    drone_ = droneActor()
    rclpy.spin(drone_)

    drone_.destroy_node()
    rclpy.shutdown()
    
    
    
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    
    

if __name__ == '__main__':
    main()

   