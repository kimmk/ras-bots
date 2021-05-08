import numpy as np
import cv2
import imutils
import matplotlib.pyplot as plt
from simple_pid import PID

# For debugging
def imshow_grayscale(img):    
    plt.imshow(img, cmap='gray', vmin=0, vmax=255)
    plt.show()

def imshow_hsv(img):
    img = cv2.cvtColor(img, cv2.COLOR_HSV2RGB)
    plt.imshow(img)
    plt.show()

def box_center(box):
    return (box[0]+box[2]//2, box[1]+box[3]//2)

def angle_between(p1, p2):
    return np.arctan2(p1[0]-p2[0], p1[1]-p2[1])

def rot_vel(vel, a):
    c, s = np.cos(a), np.sin(a)
    R = np.array(((c, -s), (s, c)))
    return np.dot(vel, R)

class Lander(object):
    def __init__(self):
        self.pid_x = PID(10.0, 0, 3.0)
        self.pid_y = PID(10.0, 0, 3.0)

    def find_leds(self, img, hsv_min, hsv_max, debug_img=None):

        # Filter LED color only
        lower = np.array(hsv_min, dtype="uint8")
        upper = np.array(hsv_max, dtype="uint8")
        mask = cv2.inRange(img, lower, upper)
        img_masked = cv2.bitwise_and(img, img, mask=mask)

        # Take grayscale, apply blur and threshold
        gray = cv2.split(img_masked)[2]
        blurred = cv2.medianBlur(gray, ksize=5)
        thresh = cv2.threshold(blurred, 5, 255, cv2.THRESH_BINARY)[1]

        # Find contours
        cnts = cv2.findContours(thresh, cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
        im_cnts = imutils.grab_contours(cnts)

        # Return features that are large enough to be candidates
        leds = []
        for points in im_cnts:
            led_box = cv2.boundingRect(points)
            x,y,w,h = led_box
            if w*h > 20:
                leds.append(led_box)
                if debug_img is not None:
                    cv2.rectangle(debug_img, (x,y), (x+w,y+h), (255,0,255), 2)

        # Sort by size
        leds.sort(key=lambda box: box[2]*box[3], reverse=True)

        return leds

    # img:              landing camera image in cv2 HSV format
    # led_a, led_b:     HSV color ranges for front led (led_a) and back led (led_b)
    # land_pos:         target landing position in image, in percentages of X and Y
    # debug_img:        debug image in BGR8 format. if set, debug info will be drawn to this image
    def land_update(self, img, led_a, led_b, land_pos=(0.5,0.5), debug_img=None):

        # Find LED candidate positions
        leds_a = self.find_leds(img, led_a[0], led_a[1], debug_img)
        leds_b = self.find_leds(img, led_b[0], led_b[1], debug_img)
        if len(leds_a) == 0 or len(leds_b) == 0:
            return np.array([0,0])

        # Right now, just pick the largest leds. Maybe could do some fancy size matching here?
        a_box = leds_a[0]
        b_box = leds_b[0]
        a_pos = box_center(a_box)
        b_pos = box_center(b_box)

        # Estimate craft position, height, angle
        ih, iw, _ = img.shape
        idim = np.array([float(iw),float(ih)])
        x = (a_pos[0] + b_pos[0]) // 2
        y = (a_pos[1] + b_pos[1]) // 2
        ab_dist = np.linalg.norm(a_pos/idim-b_pos/idim)
        h = 1.0 / ab_dist
        a = angle_between(a_pos, b_pos)

        # Calculate initial velocity vector
        self.pid_x.setpoint = iw * land_pos[0]
        self.pid_y.setpoint = ih * land_pos[1]
      
        dx = self.pid_x(x)
        dy = self.pid_x(y)
        v = np.array([dx,dy])

        # Normalize velocity vector
        norm = np.linalg.norm(v)
        v = v / norm

        # Apply height modifier and generic velocity multiplier
        v = (1.0-(1.0/(h+1.0))) * 1.0 * v

        # Rotate velocity vector according to the craft angle
        vr = rot_vel(v, a)

        # Debug drawing
        if debug_img is not None:
            dh,dw,_ = debug_img.shape
            cv2.circle(debug_img, (x,y), 5, (255,255,80), 2)
            cv2.circle(debug_img, a_pos, 3, (0,255,0), -1)
            cv2.circle(debug_img, b_pos, 3, (255,0,0), -1)
            cv2.putText(debug_img, "xy: {:.2f} {:.2f} h: {:.2f} a: {:.2f}".format(float(x)/iw, float(y)/ih, h, np.rad2deg(a)), (dw*1/8, dh*1/10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 255), 2)

        return [vr[0], vr[1]]

def handle_exit(signum, frame):
    import sys
    cmd_vel = rospy.Publisher('/tello/cmd_vel', Twist, queue_size=1)
    cmd_land = rospy.Publisher('/tello/land', Empty, queue_size=1)
    rospy.sleep(0.5)
    print("Landing and reseting velocity")
    cmd_land.publish(Empty())
    cmd_vel.publish(controls.hold())
    rospy.signal_shutdown("Landing and reseting velocity")
    sys.exit(0)

def test_file():
    lander = Lander()
    img = cv2.imread("test.png")
    img_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    debug_img = img_hsv.copy()
    led_a = [[0,240,110],[10,255,255]]
    led_b = [[110,240,110],[130,255,255]]
    vel = lander.land_update(img_hsv, led_a, led_b, debug_img=debug_img)
    imshow_hsv(debug_img)
    print(vel)

import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from std_msgs.msg import Empty
from geometry_msgs.msg import Twist
import controls
bridge = CvBridge()

class UsbCamTest:
    def __init__(self):
        rospy.Subscriber("/usb_cam/image_raw", Image, self.usb_cam_callback)
        self.debug_img = rospy.Publisher("/debug_img", Image, queue_size=10) 
        self.cmd_vel = rospy.Publisher('/tello/cmd_vel', Twist, queue_size=1)
        self.lander = Lander()
        self.led_a = [[0,80,150],[15,255,255]]
        self.led_b = [[115,100,200],[125,255,255]]

    def usb_cam_callback(self, msg):
        img = bridge.imgmsg_to_cv2(msg, "bgr8")
        img = imutils.resize(img, width=300)
        img_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        vel = self.lander.land_update(img_hsv, self.led_a, self.led_b, debug_img=img)
        print(vel)
        self.debug_img.publish(bridge.cv2_to_imgmsg(img))

def test_usb_cam():
    rospy.init_node('lander_test', anonymous=True)
    test = UsbCamTest()
    rospy.spin()

if __name__ == '__main__':
    import signal
    signal.signal(signal.SIGINT, handle_exit)
    #test_file()
    test_usb_cam()