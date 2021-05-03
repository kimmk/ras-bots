"corresponds to a main file calling the detector and processing the controls"


#! /usr/bin/python
import sys
import rospy

# ROS Image message -> OpenCV2 image converter
from cv_bridge import CvBridge, CvBridgeError
# OpenCV2 for saving an image
import cv2

import time
import numpy as np
import signal

from geometry_msgs.msg import Point
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Image
from std_msgs.msg import Float64
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty

import platform_detector
import landing_detector
import controls

bridge = CvBridge()
devel_mode = 0
debug_mode = 1

class landing_process:
    def init(self):
        self.platformDetector = platform_detector.platformDetector()
        self.landingDetector = landing_detector.droneDetector()
        
        self.cmd_pub = rospy.Publisher('/tello/cmd_vel', Twist, queue_size=1, latch = True)
        
    
    def pos_callback(pos):
    
        dronecontrol.fly_to_platform(pos.data)


def handle_exit(signum, frame):
#def handle_exit():
    
    cmd_pub = rospy.Publisher('/tello/cmd_vel', Twist, queue_size=1, latch = True)
    cmd_land = rospy.Publisher('/tello/land', Empty, queue_size=1)
    rospy.sleep(0.5)
    print("set to neutral + landing")
    cmd_land.publish(Empty())
    cmd_pub.publish(controls.hold())
    sys.exit(0)

if __name__ == '__main__':
    rospy.init_node('landing_run') 
    # Instantiate CvBridge
    bridge = CvBridge()
    landingprocess = landing_process()
    
    rospy.Subscriber("platform_x", Float64, landingprocess.pos_callback)

    signal.signal(signal.SIGINT, handle_exit)
    
    
    rospy.spin()