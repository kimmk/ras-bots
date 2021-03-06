#!/usr/bin/python3

import os
import sys
import signal
import cv2
import imutils
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import matplotlib.pyplot as plt
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Float64

bridge = CvBridge()

def is_within_bbox(p, bbox):
    x2 = bbox[0]+bbox[2]
    y2 = bbox[1]+bbox[3]
    return p[0] >= bbox[0] and p[1] >= bbox[1] and p[0] <= x2 and p[1] <= y2

def points_within_bbox(cnts, bbox):
    points = []
    for cnt in cnts:
        points.extend([[[p[0][0],p[0][1]]] for p in cnt if is_within_bbox(p[0], bbox)])
    return np.array(points, dtype=np.int32)

def add_bboxes(a, b):
    # x, y, w, h
    x = min(a[0], b[0])
    y = min(a[1], b[1])
    a_xmax = a[0] + a[2]
    a_ymax = a[1] + a[3]
    b_xmax = b[0] + b[2]
    b_ymax = b[1] + b[3]
    xmax = max(a_xmax, b_xmax)
    ymax = max(a_ymax, b_ymax)
    w = xmax-x
    h = ymax-y
    return (x,y,w,h)

class GateDetector:
    def __init__(self):
        self.gate_img = rospy.Publisher("gate_img", Image, queue_size=10)       # Debug image of gate vision
        self.gate_angle = rospy.Publisher("gate_angle", Float64, queue_size=10) # Estimated angle to gate
        self.gate_dist = rospy.Publisher("gate_dist", Float64, queue_size=10)   # Estimated distance to gate
        rospy.Subscriber("/image", Image, self.camera_callback)                 # Tello camera image

    def test_image(self, filepath):
        img = cv2.imread(filepath)
        if img is not None:
            img = imutils.resize(img, width=600)
            self.detect_gate_bbox(img)

    def test_images(self, path):
        for file in os.listdir(path):
            self.test_image(path+"/"+file)

    def camera_callback(self, img):
        try:
            cv2_img = bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            print(e)
        else:
            gate = self.detect_gate_bbox(cv2_img)
            if gate:
                angle,dist,center = gate
                self.gate_angle.publish(angle)
                self.gate_dist.publish(dist)

    def imshow_bgr(self, img):
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        plt.imshow(img)
        plt.show()

    def imshow_grayscale(self, img):    
        plt.imshow(img, cmap='gray', vmin=0, vmax=255)
        plt.show()

    def debug_draw_gate(self, gate, img):
        angle,dist,center = gate
        p = (img.shape[0]*1//3,img.shape[1]*2//3)
        cv2.putText(img, f"angle: {angle:.2f}", p, cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        cv2.circle(img, center, 4, (255,255,255), -1)

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

    def find_gate_4gon(self, cnts, debug_img=None):
        # Select largest contour by it's bounding box
        gate_points = None
        gate_center = None
        biggest_box = 0
        box_ratio = 0
        for points in cnts:
            x,y,w,h = cv2.boundingRect(points)
            box_sz = w*h
            if debug_img is not None:
                cv2.rectangle(debug_img, (x,y), (x+w,y+h), (0,0,255), 2)
            if box_sz > biggest_box:
                biggest_box = box_sz
                box_ratio = w/h
                gate_points = points
                gate_center = (x+w//2,y+h//2)

        # If found, try to approximate gate as a 4-gon
        if gate_points is not None and box_ratio > 0.3 and box_ratio < 3.0:
            epsilon = 0.05*cv2.arcLength(gate_points,True)
            approx = cv2.approxPolyDP(gate_points,epsilon,True)
            if debug_img is not None:
                cv2.drawContours(debug_img,[approx],0,(255,0,0),2)
            if len(approx) == 4:
                box = [c[0] for c in approx] # resolve annoying lists within list
                box.sort(key=lambda p: p[0]) # sort points by x coordinate

                h0 = np.linalg.norm(np.array(box[0])-np.array(box[1]))
                h1 = np.linalg.norm(np.array(box[2])-np.array(box[3]))
                gate_angle = h0/h1
                gate = (gate_angle, -1, gate_center)

                if debug_img is not None:
                    for i, coord in enumerate(box):
                        p = tuple(coord)
                        cv2.putText(debug_img, f"{i}", p, cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
                        #cv2.circle(img, p, 4, (255,255,255), -1)
                    self.debug_draw_gate(gate, debug_img)
        
                return gate
        return None

    def find_gate_5gon(self, cnts, debug_img):
        for points in cnts:
            epsilon = 0.05*cv2.arcLength(points,True)
            approx = cv2.approxPolyDP(points,epsilon,True)
            if debug_img is not None:
                print(approx[0][0])
                cv2.putText(debug_img, f"{len(approx)}", tuple(approx[0][0]), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
                clr = (255,255,255)
                if len(approx) == 5:
                    clr = (255,0,255)
                cv2.drawContours(debug_img,[approx],0,clr,2)

    def detect_gate(self, img):
        gate = None

        draw_debug_img = True #rospy.get_param("draw_debug_img")
        if draw_debug_img:
           debug_img = img

        # Filter 'gate' features from image
        gate_img = self.filter_gates(img)

        # Get gate contours
        cnts = cv2.findContours(gate_img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        im_cnts = imutils.grab_contours(cnts)
        if debug_img is not None:
            cv2.drawContours(debug_img, im_cnts, -1, (0, 255, 0), 2)

        # Find gates
        gate = self.find_gate_4gon(im_cnts, debug_img)
        if gate is None:
            gate = self.find_gate_5gon(im_cnts, debug_img)

        if debug_img is not None:
            self.imshow_bgr(debug_img)
            self.gate_img.publish(bridge.cv2_to_imgmsg(debug_img))

        return gate

    def detect_gate_bbox(self, img):
        draw_debug_img = True #rospy.get_param("draw_debug_img")
        if draw_debug_img:
           debug_img = img

        gate = None

        # Filter 'gate' features from image
        gate_img = self.filter_gates(img)

        # Get gate contours
        cnts = cv2.findContours(gate_img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        im_cnts = imutils.grab_contours(cnts)

        # Get gate contour bboxes
        bboxes = [cv2.boundingRect(points) for points in im_cnts]

        # Remove little bboxes
        bboxes = [b for b in bboxes if b[2] > 30 and b[3] > 30]

        # Combine close enough bboxes
        combi_bboxes = []
        combi_range = img.shape[0]*0.3
        #b = add_bboxes(bboxes[0], bboxes[1])
        #x,y,w,h = b

        # Get largest bbox
        bbox = (-1,-1,-1,-1)
        for b in bboxes:
            if b[2]*b[3] > bbox[2]*bbox[3]:
                bbox = b
        box_ratio = bbox[2]/bbox[3]

        if bbox[0] != -1 and box_ratio > 0.3 and box_ratio < 3.0:
            w_half = bbox[2]//2
            bbox_a = (bbox[0],bbox[1],w_half,bbox[3])
            bbox_b = (bbox[0]+w_half,bbox[1],w_half,bbox[3])

            points_a = points_within_bbox(im_cnts, bbox_a)
            points_b = points_within_bbox(im_cnts, bbox_b)

            bbox_a = cv2.boundingRect(points_a)
            bbox_b = cv2.boundingRect(points_b)

            # angle,dist,center
            angle = bbox_a[3]/bbox_b[3]
            center = (bbox[0]+bbox[2]//2,bbox[1]+bbox[3]//2)
            dist = -1 # todo
            gate = (angle,dist,center)

            if debug_img is not None:
                x,y,w,h = bbox_a
                cv2.rectangle(debug_img, (x,y), (x+w,y+h), (255,0,0), 2)
                x,y,w,h = bbox_b
                cv2.rectangle(debug_img, (x,y), (x+w,y+h), (0,0,255), 2)
                self.debug_draw_gate(gate, img)

        if debug_img is not None:
            cv2.drawContours(debug_img, im_cnts, -1, (0, 255, 0), 2)
            self.imshow_bgr(debug_img)

        return gate

def handle_exit(signum, frame):
    sys.exit(0)

if __name__ == '__main__':
    signal.signal(signal.SIGINT, handle_exit)
    rospy.init_node('gate_detector', anonymous=True)
    gate = GateDetector()
    #rospy.spin()
    #gate.test_image("gate.png")
    gate.test_images("../images/tello_gates")
