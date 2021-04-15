#!/usr/bin/python3

import cv2
import imutils
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import matplotlib.pyplot as plt
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Float64

bridge = CvBridge()

class GateDetector:
    def __init__(self):
        self.gate_img = rospy.Publisher("gate_img", Image, queue_size=10)       # Debug image of gate vision
        self.gate_angle = rospy.Publisher("gate_angle", Float64, queue_size=10) # Estimated angle to gate
        self.gate_dist = rospy.Publisher("gate_dist", Float64, queue_size=10)   # Estimated distance to gate
        rospy.Subscriber("/image", Image, self.camera_callback)                 # Tello camera image

    def test_image(self, filepath):
        img = cv2.imread(filepath)
        img = imutils.resize(img, width=600)
        self.detect_gate(img)

    def camera_callback(self, img):
        try:
            cv2_img = bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            print(e)
        else:
            angle,dist = self.detect_gate(cv2_img)
            if angle != -1:
                self.gate_angle.publish(angle)
                self.gate_dist.publish(dist)

    def imshow_bgr(self, img):
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        plt.imshow(img)
        plt.show()

    def imshow_grayscale(self, img):    
        plt.imshow(img, cmap='gray', vmin=0, vmax=255)
        plt.show()

    def filter_gates(self, img):
        # Apply HSV filter for green gates
        img_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        clr_gate = np.array([38, 195, 143])
        clr_range = np.array([5, 60, 75])
        lower = np.array(clr_gate-clr_range, dtype="uint8")
        upper = np.array(clr_gate+clr_range, dtype="uint8")
        mask = cv2.inRange(img_hsv, lower, upper)
        img_masked = cv2.bitwise_and(img_hsv, img_hsv, mask=mask)

        # Take the grayscale image from value channel and threshold it
        gray = cv2.split(img_masked)[2]
        return cv2.threshold(gray, 10, 255, cv2.THRESH_BINARY)[1]

    def detect_gate(self, img):
        #debug_img = rospy.get_param("debug_img")
        debug_img = True

        # Filter 'gate' features from image
        gate_img = self.filter_gates(img)

        # Get gate contours
        cnts = cv2.findContours(gate_img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        im_cnts = imutils.grab_contours(cnts)
        if debug_img:
            cv2.drawContours(img, im_cnts, -1, (0, 255, 0), 2)

        # Select largest gate feature from image
        gate_idx = -1
        biggest_box = 0
        for idx, points in enumerate(im_cnts):
            x,y,w,h = cv2.boundingRect(points)
            box_sz = w*h
            if debug_img:
                cv2.rectangle(img, (x,y), (x+w,y+h), (0,0,255), 2)
            if box_sz > biggest_box:
                biggest_box = box_sz
                gate_idx = idx

        # If found, try to approximate gate as a 4-gon
        angle = -1
        dist = -1
        if gate_idx != -1:
            points = im_cnts[gate_idx]
            epsilon = 0.05*cv2.arcLength(points,True)
            approx = cv2.approxPolyDP(points,epsilon,True)
            if debug_img:
                cv2.drawContours(img,[approx],0,(255,0,0),2)
            if len(approx) == 4:
                box = [c[0] for c in approx] # resolve annoying lists within list
                box.sort(key=lambda p: p[0]) # sort points by x coordinate

                if debug_img:
                    for i, coord in enumerate(box):
                        p = tuple(coord)
                        cv2.putText(img, f"{i}", p, cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
                        #cv2.circle(img, p, 4, (255,255,255), -1)
                
                h0 = np.linalg.norm(np.array(box[0])-np.array(box[1]))
                h1 = np.linalg.norm(np.array(box[2])-np.array(box[3]))
                angle = h0/h1
                if debug_img:
                    p = (img.shape[0]*1//3,img.shape[1]*2//3)
                    cv2.putText(img, f"angle: {angle}", p, cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
                print("ratio: ", angle)
        
        if debug_img:
            self.imshow_bgr(img)
            self.gate_img.publish(bridge.cv2_to_imgmsg(img))

        return angle, dist

if __name__ == '__main__':
    rospy.init_node('gate_detector', anonymous=True)
    gate = GateDetector()
    #rospy.spin()
    gate.test_image("gate.png")
