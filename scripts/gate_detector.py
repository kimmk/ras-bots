#!/usr/bin/python3

import cv2
import imutils
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import matplotlib.pyplot as plt
import rospy

class GateDetector:
    def update(self, timer):
        img = cv2.imread("gate3.png")
        img = imutils.resize(img, width=600)
        self.detect_gate(img)

    def imshow_bgr(self, img):
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        plt.imshow(img)
        plt.show()

    def imshow_grayscale(self, img):    
        plt.imshow(img, cmap='gray', vmin=0, vmax=255)
        plt.show()

    def detect_gate(self, img):
        img_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        clr_gate = np.array([38, 195, 143])
        clr_range = np.array([5, 60, 75])
        lower = np.array(clr_gate-clr_range, dtype="uint8")
        upper = np.array(clr_gate+clr_range, dtype="uint8")
        mask = cv2.inRange(img_hsv, lower, upper)
        img_masked = cv2.bitwise_and(img_hsv, img_hsv, mask=mask)
        gray = cv2.cvtColor(cv2.cvtColor(img_masked, cv2.COLOR_HSV2BGR), cv2.COLOR_BGR2GRAY)
        thresh = cv2.threshold(gray, 10, 255, cv2.THRESH_BINARY)[1]
        cnts = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        im_cnts = imutils.grab_contours(cnts)

        for i, points in enumerate(im_cnts):
            epsilon = 0.05*cv2.arcLength(points,True)
            approx = cv2.approxPolyDP(points,epsilon,True)
            cv2.drawContours(img,[approx],0,(255,0,0),2)
        
        #cv2.drawContours(img, im_cnts, -1, (0, 255, 0), 2)
        self.imshow_bgr(img)

if __name__ == '__main__':
    rospy.init_node('gate_detector', anonymous=True)
    gate = GateDetector()
    #rospy.Timer(rospy.Duration(1), gate.update)
    #rospy.spin()
    gate.update(None)
