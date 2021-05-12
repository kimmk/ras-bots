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

bridge = CvBridge()

class LandingTest(object):
    def __init__(self):
        rospy.Subscriber("/image", Image, self.camera_callback)  # Tello camera
        self.debug_img = rospy.Publisher("/debug_img", Image, queue_size=10)
        self.bg_img = rospy.Publisher("/bg_img", Image, queue_size=10)
        self.cmd_vel = rospy.Publisher('/tello/cmd_vel', Twist, queue_size=1)
        self.vel_timeout = None
        self.lander = lander.Lander()
        self.led_a = [[0,80,150],[15,255,255]]
        self.led_b = [[115,100,200],[125,255,255]]

    def reset_vel(self):
        self.cmd_vel.publish(controls.hold())

    def camera_callback(self, msg):
        img = bridge.imgmsg_to_cv2(msg, "bgr8")
        img = imutils.resize(img, width=300)
        img_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        if not self.lander.init_background(img_hsv):
            return

        # Filter background
        dfilter = self.lander.deriv_filter(img)
        img_hsv = cv2.bitwise_and(img_hsv, img_hsv, mask=dfilter)

        # Find craft position
        craft_pos = self.lander.find_craft_leds(img_hsv, self.led_a, self.led_b, debug_img=img)

        # Update craft velocity
        vel = [0,0]
        if craft_pos is not None:
            x, y, a, h = craft_pos
            vel = self.lander.land_update(img_hsv, x, y, a, h, debug_img=img)

        # Publish velocity command
        self.cmd_vel.publish(controls.control(vel[0], vel[1]))
        
        # Setup velocity timeout if set
        if self.vel_timeout is not None:
            self.vel_timeout.shutdown()
        self.vel_timeout = rospy.Timer(rospy.Duration(0.5), self.reset_vel, oneshot=True)

        self.bg_img.publish(bridge.cv2_to_imgmsg(dfilter))
        self.debug_img.publish(bridge.cv2_to_imgmsg(img))

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