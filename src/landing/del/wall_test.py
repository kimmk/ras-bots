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
import time
import socket
import threading
import random

bridge = CvBridge()
stateFindWall = 0
stateScanWall = 1

class LandingTest(object):
    def __init__(self):
        rospy.Subscriber("/image", Image, self.camera_callback)  # Tello camera
        self.debug_img = rospy.Publisher("/debug_img", Image, queue_size=1)
        self.bg_img = rospy.Publisher("/bg_img", Image, queue_size=1)
        self.cmd_vel = rospy.Publisher('/tello/cmd_vel', Twist, queue_size=1)
        self.vel_timeout = None
        self.lander = lander.Lander()
        self.close_threads = False
        self.recv_range_th = threading.Thread(target=self.recv_range)
        self.recv_range_th.start()
        self.state = stateFindWall

    def recv_range(self):
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.bind(("192.168.255.255", 7171))
        sock.settimeout(0.1)
        lastPrint = 0
        while not self.close_threads:
            try:
                self.last_range = (time.time(), int(sock.recv(1024)))
                if time.time()-lastPrint > 0.1:
                    print("range: {:.2f} {}".format(self.last_range[0],self.last_range[1]))
                    lastPrint = time.time()
            except socket.timeout:
                pass

    def reset_vel(self):
        print("reset vel")
        self.cmd_vel.publish(controls.hold())

    def update_state(self, vel, range):
        if self.state == stateFindWall:
            if range > 1500:
                vel[1] = 0.35
            else:
                self.state = stateScanWall
        elif self.state == stateScanWall:
            if range < 2500:
                vel[3] = 0.5
            else:
                self.state = stateFindWall

    def camera_callback(self, msg):
        img = bridge.imgmsg_to_cv2(msg, "bgr8")
        img = imutils.resize(img, width=300)
        debug_img = img.copy()
        img_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        if not self.lander.init_background(img):
            return

        # Filter background
        dfilter = self.lander.deriv_filter(img)
        img_filtered = cv2.bitwise_and(img_hsv, img_hsv, mask=dfilter)

        vel = [0,0,0,0]
        if self.last_range is not None:
            range_time, range = self.last_range
            if time.time()-range_time < 0.5:
                #self.update_state(vel, range)
                if range < 1500:
                    self.cmd_vel.publish(controls.control(az=0.7))
                    rospy.sleep(4)
                else:
                    vel[1] = 0.5

        self.cmd_vel.publish(controls.control(x=vel[0], y=vel[1], z=vel[2], az=vel[3]))
        
        # Setup velocity timeout if set
        #if self.vel_timeout is not None:
        #    self.vel_timeout.shutdown()
        #self.vel_timeout = rospy.Timer(rospy.Duration(1), self.reset_vel(), oneshot=True)
        
        self.bg_img.publish(bridge.cv2_to_imgmsg(dfilter))
        self.debug_img.publish(bridge.cv2_to_imgmsg(debug_img))

    def handle_exit(self, signum, frame):
        self.close_threads = True
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
    if len(sys.argv) < 2 or sys.argv[1] != "noto":
        cmd_takeoff = rospy.Publisher('/tello/takeoff', Empty, queue_size=1)
        time.sleep(1)
        cmd_takeoff.publish(Empty())
    landingTest = LandingTest()
    signal.signal(signal.SIGINT, landingTest.handle_exit)
    rospy.spin()