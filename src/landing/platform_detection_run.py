"corresponds to a main file calling the detector and processing the controls"


#! /usr/bin/python
import sys
import rospy
# ROS Image message
from sensor_msgs.msg import Image
# ROS Image message -> OpenCV2 image converter
from cv_bridge import CvBridge, CvBridgeError
# OpenCV2 for saving an image
import cv2
import imutils
from std_msgs.msg import Empty
from std_msgs.msg import String
import time
from geometry_msgs.msg import Point
from sensor_msgs.msg import LaserScan
import numpy as np
import controls
import signal
from std_msgs.msg import Float64


from geometry_msgs.msg import Twist



class fly_to_platform:
    def __init__(self):
        self.cmd_pub = rospy.Publisher('/tello/cmd_vel', Twist, queue_size=1, latch=True)
        self.searctime = None

    ### search pattern: rotate in place, if didn't help jetbot is prolly under the drone, so fly backwards
    def search(self):
        if self.searctime is None:
            self.searctime = time.time()
        if time.time()-self.searctime < 15:
            self.cmd_pub.publish(controls.control(az = 0.8))
        else:
            self.cmd_pub.publish(controls.control(y = -0.4))

    ##### return 1 if alignment succeeded, 0 if still ongoing
    def align_to_platform(self, platform_pos):
        self.searctime = None
        platform_x, platform_y, platform_size = platform_pos
        saz = platform_x/2.0
        sz = -platform_y/2.0

        # if size small go down, if y is offset then forward or backwards, if x is offset then rotate
        #if size small but platform is relatively centered, fly forward
        #if size small and platform is below threshold, fly down

        self.cmd_pub.publish(controls.control(az=saz, z = sz))
        
        if abs(platform_x) < 0.1 and abs(platform_y) < 0.3:
            self.cmd_pub.publish(controls.hold())
            return 1
        return 0


    #### platform_x = % of platform-x center off of image center
    #### return 1 if flying towards, 0 if still aligning
    def fly_to_platform(self, platform_pos):
        self.searctime = None
        if platform_pos is None:
            return 0
        platform_x, platform_y = platform_pos
        if abs(platform_x) > 0.1:
            self.align_to_platform(platform_pos)
            return 0

        self.cmd_pub.publish(controls.hold())
        time.sleep(0.2)
        sx = 1
        self.cmd_pub.publish(controls.control(y=sx))
        return 1



def handle_exit(signum, frame):
#def handle_exit():
    
    cmd_pub = rospy.Publisher('/tello/cmd_vel', Twist, queue_size=1, latch = True)
    cmd_land = rospy.Publisher('/tello/land', Empty, queue_size=1)
    rospy.sleep(1)
    print("set to neutral + landing")
    cmd_land.publish(Empty())
    cmd_pub.publish(controls.hold())
    sys.exit(0)
#signal.signal(signal.SIGINT, handle_exit)


#if called directly as main, process drone image and perform run
def drone_camera_callback(msg):
    img = bridge.imgmsg_to_cv2(msg, "bgr8")
    img = imutils.resize(img, width=500)
    platform_x = platformDetector.give_platform_x(img)
    #platform_run.fly_to_platform(platform_x)
    print("platform_x: ", platform_x)

if __name__ == '__main__':
    rospy.init_node('platform_detection_run')
    import platform_detector
    # Instantiate CvBridge
    bridge = CvBridge()
    platformDetector = platform_detector.platformDetector()
    platform_run = fly_to_platform()
    rospy.Subscriber("/image", Image, drone_camera_callback)

    signal.signal(signal.SIGINT, handle_exit)

    rospy.spin()