#!/usr/bin/python3

import os
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
        if img is not None:
            img = imutils.resize(img, width=600)
            self.detect_gate(img)

    def test_images(self, path):
        for i in range(0,100):
            
            
            file = "/picture" + str(i) + ".png"
            print(file)
            self.test_image(path+file)
            
    #        except:
     #           pass
            
        """
        for file in os.listdir(path):
            self.test_image(path+"/"+file)
            print(file)
        """
    def camera_callback(self, img):
        try:
            cv2_img = bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            print(e)
        else:
            angle,dist,center = self.detect_gate(cv2_img)
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
        #self.imshow_bgr(img_masked)
        # Take the grayscale image from value channel and threshold it
        gray = cv2.split(img_masked)[2]
        
        
        blurred = cv2.medianBlur(gray, ksize= 7)
        
        #laplacian = cv2.Laplacian(blurred,cv2.CV_32F, ksize =1)
        #self.imshow_bgr(laplacian)
        
        #thresh = cv2.threshold(blurred, 10, 255, cv2.THRESH_BINARY)[1]
        #laplacian = cv2.Laplacian(thresh,cv2.CV_32F)
        
        
        "creating long kernels type [1,1,1,1...] vert and hor"
        "kerneldim = 19 seems a good startingpoint"
        kerneldim = 19
        kernelH    =  (1/kerneldim)*np.ones((1,kerneldim))
        kernelV    =  (1/kerneldim)*np.ones((kerneldim, 1))
        
        
        dilateV = blurred
        "how many times to apply, more than 2 does veeeeeeery little"
        for i in range(1):
            erodeH=cv2.erode(dilateV, kernelH, iterations=1)
            dilateH=cv2.dilate(erodeH, kernelH, iterations=1)
            
            erodeV=cv2.erode(dilateH, kernelV, iterations=1)
            dilateV=cv2.dilate(erodeV, kernelV, iterations=1)
            #print(i)
            #self.imshow_bgr(dilateV)
        
        """
        self.imshow_bgr(gray)
        print("blurred")
        self.imshow_bgr(blurred)
        print("H")
        self.imshow_bgr(dilateH)
        print("V")
        self.imshow_bgr(dilateV)
        #filtered = cv2.filter2D(blurred, -1, hor_kernel) 
        #self.imshow_bgr(laplacian)
        #"""
        return dilateV


    def detect_gate(self, img):
        #debug_img = rospy.get_param("debug_img")
        debug_img = True

        # Filter 'gate' features from image
        gate_img = self.filter_gates(img)
        #self.imshow_bgr(gate_img)

        # Get gate contours
        cnts = cv2.findContours(gate_img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        im_cnts = imutils.grab_contours(cnts)
        
        if debug_img:
            cv2.drawContours(img, im_cnts, -1, (0, 255, 0), 2)
            #self.imshow_bgr(img)

        # Select largest gate feature from image
        gate_idx = 0
        biggest_box = 0
        box_ratio = 0
        gate_center = (0,0)
        for idx, points in enumerate(im_cnts):
            if len(points) < 20:
                continue
            
            #print(len(points))
            #print("points")
            #print(points)
            x,y,w,h = cv2.boundingRect(points)
            box_sz = w*h
            if debug_img:
                cv2.rectangle(img, (x,y), (x+w,y+h), (0,0,255), 2)
                
                perimeter = cv2.arcLength(points, closed = True)
                approx = cv2.approxPolyDP(points, 0.03 * perimeter, closed = True)
                cv2.drawContours(img,[approx],0,(255,0,0),2)
                #self.shapedetector(points,img)
                #self.imshow_bgr(img)
            if box_sz > biggest_box:
                biggest_box = box_sz
                gate_idx = idx
                box_ratio = w/h
                gate_center = (x+w//2,y+h//2)

        # If found, try to approximate gate as a 4-gon
        gate_angle = -1
        gate_dist = -1
        if gate_idx != -1 and box_ratio > 0.3 and box_ratio < 3.0:
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
                        cv2.putText(img, f"{i}", p, cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
                        #cv2.circle(img, p, 4, (255,255,255), -1)
                    cv2.circle(img, gate_center, 4, (255,255,255), -1)

                h0 = np.linalg.norm(np.array(box[0])-np.array(box[1]))
                h1 = np.linalg.norm(np.array(box[2])-np.array(box[3]))
                gate_angle = h0/h1
                if debug_img:
                    p = (img.shape[0]*1//3,img.shape[1]*2//3)
                    cv2.putText(img, f"angle: {gate_angle:.2f}", p, cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
                print("ratio: ", gate_angle)
        
        if debug_img:
            self.imshow_bgr(img)
            self.gate_img.publish(bridge.cv2_to_imgmsg(img))

        return gate_angle, gate_dist, gate_center
    
    
    "shapedetector from the very first preception-navigation lab"
    "truing to determine the corners vs full gates"
    def shapedetector(self,c,img):
        shape = "N.A"
        perimeter = cv2.arcLength(c, closed = True)
    
        approx = cv2.approxPolyDP(c, 0.035 * perimeter, closed = True)
        print("approx")
        print(approx)
        print(perimeter)
        cv2.drawContours(img,[approx],0,(255,0,0),2)
        self.imshow_bgr(img)
        #if len(approx) == 3:
        #    shape = "triangle"
        if len(approx) == 4:
            shape = "rectangle"
        
        elif len(approx) == 6:
            shape = "gatecorner"
        
        """
        
            (x, y, w, h) = cv2.boundingRect(approx)
            ar = w / float(h)
            print("x="+str(x)+ ", y="+str(y)+ ", w="+str(w)+", h="+str(h))
            if ar >= 0.95 and ar <= 1.05:
                shape = "square" 
            else:
                ux = approx.item(2) - approx.item(0)
                uy = approx.item(3) - approx.item(1)
                vx = approx.item(6) - approx.item(0)
                vy = approx.item(7) - approx.item(1)
                print("ux="+ str(ux) + " uy="+ str(uy) + " vx="+ str(vx) + " vy="+ str(vy) )
                vector_u = [ux, uy]
                vector_v = [vx, vy]
                unit_vector_u = vector_u / np.linalg.norm(vector_u)
                unit_vector_v = vector_v / np.linalg.norm(vector_v)
                dot_product = np.dot(unit_vector_u, unit_vector_v)
                angle = np.arccos(dot_product)
                angle = angle*180/np.pi

                if angle >= 88 and angle <= 92:
                    shape = "rectangle"
                else:
                #TODO (Homework): parallelogram, trapezoid and diamond
                
                # corners are as follows, coordinate= approx.item(index)
                # [01] v   [67]
                #   u
                # [23]    [45] w
                #           t
                
                tx = approx.item(4)-approx.item(6)
                ty = approx.item(5)-approx.item(7)
                wx = approx.item(4)-approx.item(2)
                wy = approx.item(5)-approx.item(3)
                
                vector_t = [tx,ty]
                vector_w = [wx,wy]
                len_u= np.linalg.norm(vector_u)
                len_v= np.linalg.norm(vector_v)
                len_t= np.linalg.norm(vector_t)
                len_w= np.linalg.norm(vector_w)
                
                unit_vector_t = vector_t/len_t
                unit_vector_w = vector_w/len_w
                
                udott = np.dot(unit_vector_u, unit_vector_t)
                vdotw = np.dot(unit_vector_v, unit_vector_w)
                
                angle_ut = np.arccos(udott) *180/np.pi
                angle_vw = np.arccos(vdotw) *180/np.pi
                
                if angle_ut <= 2 and angle_vw <= 2:
                    print(" len_u= " + str(len_u) +" len_v= " + str(len_v) +" len_t= " + str(len_t) +" len_w= " + str(len_w))
                    if max(len_u, len_v, len_t, len_w) - min(len_u, len_v, len_t, len_w) <= len_u/10:
                        # max 10% diff in sidelengths
                        shape = "diamond"
                    else:
                        shape = "parallelogram"
                else:
                    if (angle_ut <= 2 and angle_vw > 2) or ( angle_ut > 2 and angle_vw <= 2):
                        shape = "trapezoid"
                    else:
                        shape = "4SIDES" 
                        #"""
        print(shape)

if __name__ == '__main__':
    rospy.init_node('gate_detector', anonymous=True)
    gate = GateDetector()
    #rospy.spin()
    #gate.test_image("gate.png")
    gate.test_images("../images/tello_gates")
