
#!/usr/bin/python3

#"""
#this detects the platform by
#- grayscale + threshold, platform is very white
#- find contours, list as nested tree, pick only nested contour
#- platform should now be only contour
#"""

import os
import cv2
import imutils
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import matplotlib.pyplot as plt
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Float64

from sensor_msgs.msg import CompressedImage


bridge = CvBridge()
devel_mode = 0
debug_mode = 1


class platformDetector:
    def __init__(self):
        self.platform_brightness_threshold = 190
        #self.platform_brightness_threshold = 140
        self.platform_x = rospy.Publisher("platform_x", Float64, queue_size=1) # Estimated x coord of platform
        #self.platform_dist = rospy.Publisher("platform_dist", Float64, queue_size=10)   # Estimated distance to platform
        
        #rospy.Subscriber("/image", Image, self.camera_callback)                 # Tello camera image
        if devel_mode:
            print("init")
        
        if debug_mode:
            self.platform_img = rospy.Publisher("platform_img", Image, queue_size=10)       # Debug image of platform vision

    def test_image(self, filepath):
        cv2_img = cv2.imread(filepath)
        if cv2_img is not None:
            img = imutils.resize(cv2_img, width=600)
            self.image_processing(cv2_img)
            
        
    def test_images(self, path):
        #for i in range(0,100):
            
            
        #    file = "/picture" + str(i) + ".png"
        #    print(file)
        #    self.test_image(path+file)
            
    #        except:
     #           pass
            
        
        for file in os.listdir(path):
            self.test_image(path+"/"+file)
            print(file)
        #"""
        
    def image_processing(self, cv2_img):
        center_x = self.detect_platform(cv2_img)
        if center_x is not None:
            self.platform_x.publish(center_x)
        
    def camera_callback(self, img):
        if debug_mode: print("camera callback")
        
        cv2_img = bridge.imgmsg_to_cv2(img, "bgr8")
        if debug_mode: print("img recvd")
        self.image_processing(cv2_img)
        
        #    print("getting center")
            
            #putting this here is kinda spaghettiying
        #    if center is not None:
        #        dronecontrol.fly_to_platform(center)
            #if angle != -1:
            #    self.platform_angle.publish(angle)
            #    self.platform_dist.publish(dist)
        
        
        

    def imshow_bgr(self, img):
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        plt.imshow(img)
        plt.show()

    def imshow_grayscale(self, img):    
        plt.imshow(img, cmap='gray', vmin=0, vmax=255)
        plt.show()
        
        
    #in: cv2_image
    #out: cv2_image filtered and thresholded
    def filter_platforms(self, img):
        
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        
        blurred = cv2.medianBlur(gray, ksize= 3)
        
        thresh = cv2.threshold(blurred, self.platform_brightness_threshold, 255, cv2.THRESH_BINARY)[1]
        #laplacian = cv2.Laplacian(thresh,cv2.CV_32F)
        
        
        #kernelErode = np.ones((3,3))
        #eroded = cv2.erode(thresh, kernelErode)
        
        
        
        if devel_mode:
            self.imshow_bgr(gray)
            self.imshow_bgr(thresh)
            #self.imshow_bgr(laplacian)
            #self.imshow_bgr(eroded)
            """
            print("blurred")
            self.imshow_bgr(blurred)
            print("H")
            self.imshow_bgr(dilateH)
            print("V")
            self.imshow_bgr(dilateV)
            #filtered = cv2.filter2D(blurred, -1, hor_kernel) 
            #self.imshow_bgr(laplacian)
            #"""
        return thresh

    #in: cv2_image
    #out: % the platform x coord is off image center, 
    #       OR if no platform: None
    def detect_platform(self, img):
        #debug_img = rospy.get_param("debug_img")
        

        # Filter 'platform' features from image
        platform_img = self.filter_platforms(img)
        #
        if devel_mode:
            self.imshow_bgr(platform_img)
        
        # Get platform contours
        cnts = cv2.findContours(platform_img, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        if debug_mode:
            print("contours " + str(len(cnts[0])))
            
        
        im_cnts = []
        if len(cnts[0]):
            for i, line in enumerate(cnts[1][0]):
                if line[3]!=-1:
                    im_cnts.append(cnts[0][i])
        if debug_mode:
            print(len(im_cnts))
        if devel_mode:
            cv2.drawContours(img, im_cnts, -1, (0, 255, 0), 2)
            self.imshow_bgr(img)

        # Select largest platform feature from image
        platform_idx = 0
        biggest_box = 0
        box_ratio = 0
        platform_center = (0,0)
        for idx, points in enumerate(im_cnts):
            if len(points) < 20:
                continue
                
            x,y,w,h = cv2.boundingRect(points)
            box_sz = w*h
            if debug_mode:
                cv2.rectangle(img, (x,y), (x+w,y+h), (0,0,255), 2)
                
                perimeter = cv2.arcLength(points, closed = True)
                approx = cv2.approxPolyDP(points, 0.03 * perimeter, closed = True)
                cv2.drawContours(img,[approx],0,(255,0,0),2)
                #self.shapedetector(points,img)
                #self.imshow_bgr(img)
            if box_sz > biggest_box:
                biggest_box = box_sz
                platform_idx = idx+1
                box_ratio = w/h
                platform_center = (x+w//2,y+h//2)
        
        if platform_idx == 0:
            print("idx==none" + str(platform_idx))
            return None
        
        platform_center_x = platform_center[0]
        width = img.shape[1]
        if debug_mode:
            print("x-coord "+str(platform_center_x))
            print("image_width " + str(width))
        platform_center_x = (platform_center_x-(width/2.0))/(width/2.0)
        if debug_mode:
            print("x-% " + str(platform_center_x))
        
        if devel_mode:
            self.imshow_bgr(img)
        if debug_mode:
            pass
            #print("debug")
            #self.platform_img.publish(bridge.cv2_to_imgmsg(platform_img))
            #self.platform_img.publish(bridge.cv2_to_imgmsg(img))
            
        return platform_center_x
        
    
    

if __name__ == '__main__':
    rospy.init_node('platform_detector', anonymous=True)
    platform = platformDetector()
    #dronecontrol = drone_controller.control()
    
    
    #rospy.Subscriber("/tello/image_raw/h264", CompressedImage, platform.camera_callback)                 # Tello camera image
    if devel_mode:
        print("main")
        platform.test_images("../ras-bots/images/platform_pics/run2")
    else:
        rospy.spin()
    
    #platform.test_image("platform.png")
    #
