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
        self.cmd_vel = rospy.Publisher('/tello/cmd_vel', Twist, queue_size=1)
        self.vel_timeout = None
        self.lander = lander.Lander()
        self.led_a = [[0,0,0], [0,0,0]]
        self.led_b = [[0,0,0], [0,0,0]]

    def reset_vel(self):
        self.cmd_vel.publish(controls.hold())

    def camera_callback(self, msg):
        img = bridge.imgmsg_to_cv2(msg, "hsv8")
        debug_img = img.copy()
        v = self.lander.land_update(img, self.led_a, self.led_b, debug_img=debug_img)

        # Publish velocity command
        self.cmd_vel.publish(controls.control(v[0], v[1]))
        
        # Setup velocity timeout if set
        if self.vel_timeout is not None:
            self.vel_timeout.shutdown()
        self.vel_timeout = rospy.Timer(rospy.Duration(0.5), self.reset_vel, oneshot=True)

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