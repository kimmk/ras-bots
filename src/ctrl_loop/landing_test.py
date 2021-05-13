import sys
import signal
import numpy as np
import cv2
import imutils
from cv_bridge import CvBridge, CvBridgeError
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Empty
from geometry_msgs.msg import Twist
import controls
import lander
import matplotlib.pyplot as plt
import time

bridge = CvBridge()

class LandingTest(object):
    def __init__(self):
        rospy.Subscriber("/image", Image, self.camera_callback)  # Tello camera
        self.debug_img = rospy.Publisher("/debug_img", Image, queue_size=1)
        self.bg_img = rospy.Publisher("/bg_img", Image, queue_size=1)
        self.cmd_vel = rospy.Publisher('/tello/cmd_vel', Twist, queue_size=1)
        self.vel_timeout = None
        self.lander = lander.Lander()
        self.led_red_lower = [[0,50,100],[15,255,255]]
        self.led_red_higher = [[170,50,100],[179,255,255]]
        self.led_blue = [[110,80,100],[130,255,255]]
        self.target_hovertimer = None

    def reset_vel(self):
        self.cmd_vel.publish(controls.hold())

    def landingtimer(self, reset = False):
        if reset:
            self.target_hovertimer = None
            return 0

        if self.target_hovertimer is None:
            self.target_hovertimer = time.time()
            return 0

        if time.time() - self.target_hovertimer < 1:
            return 0

        return 1

    def camera_callback(self, msg):
        img = bridge.imgmsg_to_cv2(msg, "bgr8")
        debugimg = img.copy()
        #pltimg = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        
        #plt.imshow(pltimg)
        #plt.show()
        
        #image is already w = 300
        #img = imutils.resize(img, width=300)
        img_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        if not self.lander.init_background(img):
            return

        # Filter background
        dfilter = self.lander.deriv_filter(img)
        img_filtered = cv2.bitwise_and(img_hsv, img_hsv, mask=dfilter)
        
        #img_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        #img_filtered = cv2.bitwise_and(img, img, mask=dfilter)

        # Find craft position
        #craft_pos = self.lander.find_craft_leds(img_hsv, self.led_blue, self.led_red_lower, self.led_red_higher, debug_img=img)
        craft_pos = self.lander.find_craft_frame(dfilter, debug_img = debugimg)
        # Update craft velocity
        vx,vy,vz = 0,0,0

        if craft_pos is not None:
            x, y, a, h = craft_pos

            land_pos = (0.5, 0.5)
            vx,vy = self.lander.land_update(img_filtered, x, y, a, h, land_pos, debug_img=debugimg)
            ih, iw, _ = img.shape
            target = (iw*land_pos[0], ih*land_pos[1])
            if abs(x-target[0]) < 0.05*iw and abs(y-target[1] < 0.05*ih):
                vz = - 0.2*self.landingtimer()
                print("ontarget")
                if h < 0.8:
                    self.lander.land()

            else:
                self.landingtimer(reset = True)
        # Publish velocity command
        self.cmd_vel.publish(controls.control(x = vx, y = vy, z = vz))
        
        # Setup velocity timeout if set
        #if self.vel_timeout is not None:
        #    self.vel_timeout.shutdown()
        #self.vel_timeout = rospy.Timer(rospy.Duration(1), self.reset_vel(), oneshot=True)
        #"""
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
    sys.exit(0)

if __name__ == '__main__':
    rospy.init_node('landing_test', anonymous=True)
    signal.signal(signal.SIGINT, handle_exit)
    landingTest = LandingTest()
    rospy.spin()