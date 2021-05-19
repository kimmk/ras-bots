#! /usr/bin/python

import base64

import numpy as np 
import matplotlib.pyplot as plt

import cv2
import rospy
import actionlib
from cv_bridge import CvBridge
import imutils

import sys
import signal
import time

from std_msgs.msg import Empty
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from std_msgs.msg import Float64

import controls
import landing_recon



# cap = cv2.VideoCapture('nvarguscamerasrc ! video/x-raw(memory:NVMM), width=3280, height=2464, format=(string)NV12,framerate=(fraction)20/1 ! nvvidconv ! video/x-raw, format=(string)BGRx ! videoconvert !video/x-raw, format=(string)BGR ! appsink' , cv2.CAP_GSTREAMER)

cap = cv2.VideoCapture('nvarguscamerasrc ! video/x-raw(memory:NVMM), width=1280, height=720, format=(string)NV12,framerate=(fraction)20/1 ! nvvidconv ! video/x-raw, format=(string)BGRx ! videoconvert !video/x-raw, format=(string)BGR ! appsink' , cv2.CAP_GSTREAMER)
bridge = CvBridge()





class LandingTest(object):
    def __init__(self):
        #rospy.Subscriber("/image", Image, self.camera_callback)  # Tello camera

        #this should be done with a ros action
        rospy.Subscriber("/do_landing", Float64 , self.landing_command_callback)
        self.landing_command = 0

        self.debug_img = rospy.Publisher("/debug_img", Image, queue_size=1)
        self.bg_img = rospy.Publisher("/bg_img", Image, queue_size=1)
        self.cmd_vel = rospy.Publisher('/tello/cmd_vel', Twist, queue_size=1)
        self.vel_timeout = None
        self.lander = landing_recon.Lander()
        self.led_red_lower = [[0, 50, 100], [15, 255, 255]]
        self.led_red_higher = [[170, 50, 100], [179, 255, 255]]
        self.led_blue = [[110, 80, 100], [130, 255, 255]]
        self.target_hovertimer = None

    def read_camera(self):
        while (not rospy.is_shutdown()):
            if not self.landing_command:
                return

            _, frame = cap.read()
            try:
                width = 1280
                height = 720
                # cv2.imwrite("Stream_pub{}.jpg".format(cnt), frame)

                x = (width - height) / 2

                frame = frame[0:height, x:x + height]
                frame = imutils.resize(frame, width=300)

                self.image_processing(frame)
            except():
                cap.release()
                print("Wrong with camera intial!")
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    def reset_vel(self):
        self.cmd_vel.publish(controls.hold())

    def landingtimer(self, reset=False):
        if reset:
            self.target_hovertimer = None
            return 0

        if self.target_hovertimer is None:
            self.target_hovertimer = time.time()
            return 0

        if time.time() - self.target_hovertimer < 1:
            return 0

        return 1

    def landing_command_callback(self, msg):
        self.landing_command = msg.data
        self.lander.reset_background()

    def image_processing(self, img):
        #img = bridge.imgmsg_to_cv2(msg, "bgr8")
        debugimg = img
        img_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        if not self.lander.init_background(img):

            return

        # Filter background
        dfilter = self.lander.deriv_filter(img)
        img_filtered = cv2.bitwise_and(img_hsv, img_hsv, mask=dfilter)

        # img_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        # img_filtered = cv2.bitwise_and(img, img, mask=dfilter)

        # Find craft position
        # craft_pos = self.lander.find_craft_leds(img_hsv, self.led_blue, self.led_red_lower, self.led_red_higher, debug_img=img)
        craft_pos = self.lander.find_craft_frame(dfilter, debug_img=debugimg)
        # Update craft velocity
        vx, vy, vz = 0, 0, 0

        if craft_pos is not None:
            x, y, a, h = craft_pos

            land_pos = (0.5, 0.5)
            vx, vy = self.lander.land_update(img_filtered, x, y, a, h, land_pos, debug_img=debugimg)
            ih, iw, _ = img.shape
            target = (iw * land_pos[0], ih * land_pos[1])
            if abs(x - target[0]) < 0.05 * iw and abs(y - target[1] < 0.05 * ih):
                vz = - 0.4 * self.landingtimer()

                if h < 0.62:
                    self.lander.land()

            else:
                self.landingtimer(reset=True)
        # Publish velocity command
        self.cmd_vel.publish(controls.control(x=vx, y=vy, z=vz))

        # Setup velocity timeout if set
        # if self.vel_timeout is not None:
        #    self.vel_timeout.shutdown()
        # self.vel_timeout = rospy.Timer(rospy.Duration(1), self.reset_vel(), oneshot=True)
        # """
        self.bg_img.publish(bridge.cv2_to_imgmsg(dfilter))
        self.debug_img.publish(bridge.cv2_to_imgmsg(debugimg))


def handle_exit(signum, frame):
    cmd_vel = rospy.Publisher('/tello/cmd_vel', Twist, queue_size=1)
    cmd_land = rospy.Publisher('/tello/land', Empty, queue_size=1)
    rospy.sleep(0.5)
    print("Landing and reseting velocity")
    cmd_land.publish(Empty())
    cmd_vel.publish(controls.hold())
    rospy.signal_shutdown("Landing and reseting velocity")
    cap.release()
    sys.exit(0)


if __name__ == '__main__':
    rospy.init_node('landing_test', anonymous=True)
    signal.signal(signal.SIGINT, handle_exit)
    landingTest = LandingTest()
    rospy.spin()



        


#if __name__ == '__main__':
#    rospy.init_node('image_pub')
#    pub = rospy.Publisher('/jetbot_image', Image, queue_size=1)