import numpy as np
import cv2
import imutils
import matplotlib.pyplot as plt
from simple_pid import PID
import time


# For debugging
def imshow_grayscale(img):
    plt.imshow(img, cmap='gray', vmin=0, vmax=255)
    plt.show()


def imshow_hsv(img):
    img = cv2.cvtColor(img, cv2.COLOR_HSV2RGB)
    plt.imshow(img)
    plt.show()


def box_center(box):
    return (box[0] + box[2] // 2, box[1] + box[3] // 2)


def angle_between(p1, p2):
    return np.arctan2(p1[0] - p2[0], p1[1] - p2[1])


def rot_vel(vel, a):
    c, s = np.cos(a), np.sin(a)
    R = np.array(((c, -s), (s, c)))
    return np.dot(R, vel)


class Lander(object):
    def __init__(self):
        P, I, D = 0.8, 0.0, 0.8
        self.pid_x = PID(P, I, D)
        self.pid_y = PID(P, I, D)
        self.pid_x.output_limits = (-0.4, 0.4)
        self.pid_y.output_limits = (-0.4, 0.4)

        self.cmd_vel = rospy.Publisher('/tello/cmd_vel', Twist, queue_size=1)
        self.cmd_land = rospy.Publisher('/tello/land', Empty, queue_size=1)
        self.cmd_takeoff = rospy.Publisher('/tello/takeoff', Empty, queue_size=1)
        self.reset_background()
        
        self.debug_img = rospy.Publisher("/color_filter", Image, queue_size=10)
        
        self.backSub = cv2.createBackgroundSubtractorMOG2()
        self.craft_entry_angle = None

    def find_leds(self, img, hsv1_min, hsv1_max, hsv2_min = None, hsv2_max = None, debug_img=None):

        # Filter LED color only
        lower1 = np.array(hsv1_min, dtype="uint8")
        upper1 = np.array(hsv1_max, dtype="uint8")
        mask = cv2.inRange(img, lower1, upper1)
        if hsv2_min is not None and hsv2_max is not None:
            lower2 = np.array(hsv1_min, dtype="uint8")
            upper2 = np.array(hsv1_max, dtype="uint8")
            mask2 = cv2.inRange(img, lower2, upper2)
            mask = mask + mask2
            
        img_masked = cv2.bitwise_and(img, img, mask=mask)
        self.debug_img.publish(bridge.cv2_to_imgmsg(img_masked))
        # Take grayscale, apply blur and threshold
        gray = cv2.split(img_masked)[2]
        blurred = cv2.medianBlur(gray, ksize=5)
        thresh = cv2.threshold(blurred, 5, 255, cv2.THRESH_BINARY)[1]

        # Find contours
        cnts = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        im_cnts = imutils.grab_contours(cnts)

        # Return features that are large enough to be candidates
        leds = []
        for points in im_cnts:
            led_box = cv2.boundingRect(points)
            x, y, w, h = led_box
            c = (0, 0, 255)
            if w * h > 4:
                c = (255, 0, 0)
                leds.append(led_box)
            if debug_img is not None:
                cv2.rectangle(debug_img, (x, y), (x + w, y + h), c, 2)

        # Sort by size
        leds.sort(key=lambda box: box[2] * box[3], reverse=True)

        return leds

    # Finds candidates for leds A and B from image and returns approx craft position.
    # img:              landing camera image in cv2 HSV format
    # led_a, led_b:     HSV color ranges for front led (led_a) and back led (led_b)
    # debug_img:        debug image in BGR8 format. if set, debug info will be drawn to this image
    # returns:          None if no leds were found OR
    #                   Craft position (x, y, a, h)
    def find_craft_leds(self, img, led_blue, led_red_lower, led_red_higher, debug_img=None):

        # Find LED candidate positions
        leds_a = self.find_leds(img, led_red_lower[0], led_red_lower[1],led_red_higher[0], led_red_higher[1], debug_img=debug_img)
        leds_b = []
        #leds_b = self.find_leds(img, led_blue[0], led_blue[1], debug_img=debug_img)
        if len(leds_a) == 0 or len(leds_b) == 0:
            return None

        # Right now, just pick the largest leds. Maybe could do some fancy size matching here?
        a_box = leds_a[0]
        b_box = leds_b[0]
        a_pos = box_center(a_box)
        b_pos = box_center(b_box)

        # Estimate craft position, height, angle
        ih, iw, _ = img.shape
        idim = np.array([float(iw), float(ih)])
        x = (a_pos[0] + b_pos[0]) // 2
        y = (a_pos[1] + b_pos[1]) // 2
        ab_dist = np.linalg.norm(a_pos / idim - b_pos / idim)
        h = 1.0 / ab_dist
        a = ((angle_between(a_pos, b_pos) + np.pi * 1.5) % (np.pi * 2)) - np.pi

        # Debug drawing
        if debug_img is not None:
            cv2.circle(debug_img, a_pos, iw//80, (0, 255, 0), -1)
            cv2.circle(debug_img, b_pos, iw//80, (255, 0, 0), -1)

        return (x, y, a, h)

    #find frame center just from the filtered image
    #in: filtered image
    #out: x,y of biggest contour center
    def find_craft_frame(self, img, debug_img = None):
        #img = cv2.medianBlur(img, ksize=3)
        img = cv2.boxFilter(img, -1, ksize=(5,5), normalize = False)
        img = cv2.erode(img, kernel= cv2.getStructuringElement(shape = cv2.MORPH_RECT, ksize=(5,5)))
        self.debug_img.publish(bridge.cv2_to_imgmsg(img))
        cnts = cv2.findContours(img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        im_cnts = imutils.grab_contours(cnts)
        if len(im_cnts)==0:
            return None
        # find the biggest countour (c) by the area
        c = max(im_cnts, key=cv2.contourArea)

        x, y, w, h = cv2.boundingRect(c)
        if w < 10 or h < 10:
            return None
        center_x, center_y = x+w/2, y+h/2
        if debug_img is not None:
            # draw the biggest contour (c) in green
            cv2.rectangle(debug_img, (x, y), (x + w, y + h), (0 , 255, 0), 2)
            cv2.rectangle(debug_img, (center_x-1, center_y-1), (center_x+1, center_y+1), (0, 255, 0), 2)

        if self.craft_entry_angle is None:

            ih, iw= img.shape
            entry_x, entry_y = center_x-iw/2, center_y-ih/2
            self.craft_entry_angle = np.arctan2(entry_y, -entry_x)


        #approximate height by contour size
        z = 100.0/w




        return center_x,center_y, self.craft_entry_angle, z

    # Calculates new velocity vector for craft according to input parameters.
    # img:              landing camera image in cv2 HSV format
    # x, y:             craft position in image
    # a:                craft angle
    # land_pos:         target landing position in image, in percentages of image width and height
    # debug_img:        debug image in BGR8 format. if set, debug info will be drawn to this image
    # returns:          new craft velocity (vel_x, vel_y)
    def land_update(self, img, x, y, a, h, land_pos=(0.5, 0.5), debug_img=None):

        # Calculate initial velocity vector
        ih, iw, _ = img.shape

        self.pid_x.setpoint = iw * land_pos[0]
        self.pid_y.setpoint = ih * land_pos[1]

        dx = self.pid_x(x)
        dy = self.pid_y(y)
        v = np.array([-dy,dx])

        # Normalize velocity vector
        norm = np.linalg.norm(v)
        v = v / norm

        # Apply height modifier and generic velocity multiplier
        v = (1.0 - (1.0 / (h + 1.0))) * 1.0 * v

        # Rotate velocity vector according to the craft angle
        vr = rot_vel(v, a)

        # Debug drawing
        if debug_img is not None:
            dh, dw, _ = debug_img.shape
            #draw center and drone pos
            cv2.circle(debug_img, (int(self.pid_x.setpoint), int(self.pid_y.setpoint)), 5, (80, 255, 255), 2)
            cv2.circle(debug_img, (x, y), dw//80, (255, 255, 80), 2)
            cv2.putText(debug_img,
                        "xy: {:.2f} {:.2f}".format(float(x) / iw, float(y) / ih),
                        (dw * 1 / 10, dh * 1 / 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 255), 2)
            cv2.putText(debug_img, "h: {:.2f} a: {:.2f}".format(h, np.rad2deg(a)),
                        (dw * 1 / 10, dh * 2 / 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 255), 2)

            cv2.putText(debug_img, "vel: {:.2f} {:.2f}".format(vr[0], vr[1]),
                        (dw * 1 / 10, dh * 9 / 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 255), 2)

            fva = (dw // 2, dh * 4 // 5)
            fvb = (fva[0] + int(vr[0] * 20), fva[1] + int(-vr[1] * 20))
            cv2.line(debug_img, fva, fvb, (255, 255, 0), 2)
            cv2.circle(debug_img, fva, 2, (255, 255, 255), -1)
            cv2.line(debug_img, fva, (fva[0],fva[1]-15), (0,0,255), 1)
            fva = (x, y)
            rot = rot_vel([20, 0], a)
            fvb = (x + int(rot[0]), y - int(rot[1]))
            cv2.line(debug_img, fva, fvb, (255, 255, 255), 2)

        return [vr[0], vr[1]]

    #filter based on change to background image
    #in: camera image (HSV)
    #out: background filter mask
    #needs set: self.background_img
    def deriv_filter(self, img, min_delta=40):
        img_channels = cv2.split(img)
        gray = img_channels[2]

        frameDelta = cv2.absdiff(self.background_img, gray)
        drone_area = cv2.threshold(frameDelta, min_delta, 255, cv2.THRESH_BINARY)[1]
        #fgMask = self.backSub.apply(img)
        return drone_area

    # land the drone, and when landed take an image, this is now the empty background
    # in: camera image (HSV)
    # out: 0 if process not complete
    #     1 if background set
    # set: self.background_img for further use
    def init_background(self, img):
        secs_to_land = 4
        if self.background_img is not None:
            return 1
        if self.background_init_timer is None:
            self.background_init_timer = time.time()
            self.land()
            return 0
        if time.time() - self.background_init_timer < secs_to_land:
            return 0

        #store as grayscale (HSV V channel)
        img_channels = cv2.split(img)
        self.background_img = img_channels[2]
        self.takeoff()
        time.sleep(2)
        print("bg image ready")
        return 1

    def reset_background(self):
        self.background_img = None
        self.background_init_timer = None

    def land(self):
        self.cmd_vel.publish(controls.hold())
        time.sleep(0.2)
        self.cmd_land.publish(Empty())
        self.cmd_vel.publish(controls.hold())

    def takeoff(self):
        self.cmd_takeoff.publish(Empty())


#########################################################################
### NOTE: Everything below from here is debug code and may be removed ###
#########################################################################
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
    led_a = [[0, 240, 110], [10, 255, 255]]
    led_b = [[110, 240, 110], [130, 255, 255]]
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

# Using USB camera in ROS:
# $ apt install ros-melodic-usb-cam 
# $ rosrun usb_cam usb_cam_node
class UsbCamTest:
    def __init__(self):
        rospy.Subscriber("/usb_cam/image_raw", Image, self.usb_cam_callback)
        #rospy.Subscriber("/usb_cam/image_raw", Image, self.usb_cam_led_calibration)
        self.debug_img = rospy.Publisher("/debug_img", Image, queue_size=10)
        self.bg_img = rospy.Publisher("/bg_img", Image, queue_size=10)
        self.led_img = rospy.Publisher("/led_img", Image, queue_size=10)
        self.cmd_vel = rospy.Publisher('/tello/cmd_vel', Twist, queue_size=1)
        self.lander = Lander()
        # calibrated with tool script
        self.led_a = [[116, 92, 111], [196, 150, 255]]
        self.led_b = [[106, 105, 178], [124, 221, 255]]

        # hand calibrated
        #self.led_a = [[0, 80, 240], [15, 130, 255]]
        #self.led_b = [[115, 100, 200], [125, 255, 255]]


    def usb_cam_led_calibration(self, msg):
        img = bridge.imgmsg_to_cv2(msg, "bgr8")
        img = imutils.resize(img, width=300)
        img_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        # Get HSV-pixels from inside a circle in the middle of the image
        h, w, _ = img.shape
        r = int(w * 0.11)
        r2 = r**2
        x0, y0 = w // 2, h // 2
        hue, sat, val = [], [], []
        for x in range(0, w):
            for y in range(0, h):
                if (x-x0)**2+(y-y0)**2 < r2:
                    p = img_hsv[y][x]
                    hue.append(p[0])
                    sat.append(p[1])
                    val.append(p[2])

        # Get mean and standard dev for hue, saturation, value
        hue_mean = int(np.mean(hue))
        hue_std = int(np.std(hue))
        print("hue: mean {}, std {}".format(hue_mean, hue_std))
        sat_mean = int(np.mean(sat))
        sat_std = int(np.std(sat))
        print("sat: mean {}, std {}".format(sat_mean, sat_std))
        val_mean = int(np.mean(val))
        val_std = int(np.std(val))
        print("val: mean {}, std {}".format(val_mean, val_std))
        print("filter: [[{},{},{}],[{},{},{}]]\n".format(hue_mean-hue_std,sat_mean-sat_std,val_mean-val_std,hue_mean+hue_std,sat_mean+sat_std,val_mean+val_std))

        cv2.circle(img, (x0, y0), r, (255, 255, 80), 2)
        self.led_img.publish(bridge.cv2_to_imgmsg(img))

    def usb_cam_callback(self, msg):
        img = bridge.imgmsg_to_cv2(msg, "bgr8")
        img = imutils.resize(img, width=450)
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

        print(vel)

        self.bg_img.publish(bridge.cv2_to_imgmsg(dfilter))
        self.debug_img.publish(bridge.cv2_to_imgmsg(img))

def test_usb_cam():
    rospy.init_node('lander_test', anonymous=True)
    test = UsbCamTest()
    rospy.spin()


if __name__ == '__main__':
    import signal

    signal.signal(signal.SIGINT, handle_exit)
    # test_file()
    test_usb_cam()
