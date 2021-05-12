"""
this detects the drone by
- grayscale + threshold, drone is very white
- find contours, list as nested tree, pick only nested contour
- drone should now be only contour
#"""
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

class droneDetector:
    def __init__(self):
        self.drone_img = rospy.Publisher("drone_img", Image, queue_size=10)       # Debug image of drone vision
        self.drone_angle = rospy.Publisher("drone_angle", Float64, queue_size=10) # Estimated angle to drone
        self.drone_dist = rospy.Publisher("drone_dist", Float64, queue_size=10)   # Estimated distance to drone
        rospy.Subscriber("/image", Image, self.camera_callback)                 # Tello camera image
        self.initframe1 = None
        self.initframe2 = None
        self.initframe3 = None

    def test_image(self, filepath):
        img = cv2.imread(filepath)
        if img is not None:
            img = imutils.resize(img, width=600)
            if self.initframe3 is not None:
                self.detect_drone(img)
            else:
                self.set_initframe(img)


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
    def camera_callback(self, img):
        try:
            cv2_img = bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            print(e)
        else:
            if self.initframe3 is not None:
                pos = self.detect_drone(cv2_img)
                if angle != -1:
                    self.drone_angle.publish(angle)
                    self.drone_dist.publish(dist)
            else:
                self.set_initframe(cv2_img)

    def set_initframe(self,img):
        
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        #self.imshow_bgr(gray)
        self.initframe3 = self.initframe2
        self.initframe2 = self.initframe1
        self.initframe1 = gray
        
        if self.initframe3 is not None:
            self.filter_initframe()
    
    def filter_initframe(self):
        #cutting holes in img1 and patching it with pars from img3
        
        diff12 = cv2.absdiff(self.initframe1, self.initframe2)
        mask_patches = cv2.threshold(diff12, 25, 255, cv2.THRESH_BINARY)[1]
        mask_holes = cv2.bitwise_not(mask_patches)
        
        patches = cv2.bitwise_and(self.initframe3,self.initframe3, mask = mask_patches)
        holes = cv2.bitwise_and(self.initframe1,self.initframe1, mask = mask_holes)
        print("patches and holes")
        self.imshow_bgr(patches)
        self.imshow_bgr(holes)
        
        clear = cv2.bitwise_or(patches,holes)
        self.imshow_bgr(clear)
        self.initframe1 = clear
        
    def imshow_bgr(self, img):
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        plt.imshow(img)
        plt.show()

    def imshow_grayscale(self, img):    
        plt.imshow(img, cmap='gray', vmin=0, vmax=255)
        plt.show()

    def filter_drones(self, img):
        
        img_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        self.imshow_bgr(img_hsv)
        
        clr_gate = np.array([90, 127, 178])
        clr_range = np.array([5, 127, 77])
        lower = np.array(clr_gate-clr_range, dtype="uint8")
        upper = np.array(clr_gate+clr_range, dtype="uint8")
        
        #lower = np.array([90,0,100],dtype="uint8")
        #upper = np.array([98,255,255],dtype="uint8")
        mask = cv2.inRange(img_hsv, lower, upper)
        img_masked = cv2.bitwise_and(img_hsv, img_hsv, mask=mask)
        #self.imshow_bgr(img_masked)
        # Take the grayscale image from value channel and threshold it
        gray = cv2.split(img_masked)[2]

        #gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        
        blurred = cv2.medianBlur(gray, ksize= 1)
        
        thresh = cv2.threshold(blurred, 80, 255, cv2.THRESH_BINARY)[1]
        #laplacian = cv2.Laplacian(thresh,cv2.CV_32F)
        
        
        kernelErode = np.ones((3,3))
        eroded = cv2.erode(thresh, kernelErode)
        
        
        
        
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


    def detect_drone(self, img):
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        frameDelta = cv2.absdiff(self.initframe1, gray)
        thresh = cv2.threshold(frameDelta, 25, 255, cv2.THRESH_BINARY)[1]
        #self.imshow_bgr(thresh)
        
        
        
        
        #debug_img = rospy.get_param("debug_img")
        debug_img = True

        # Filter 'drone' features from image
        drone_img = thresh
        #self.filter_drones(img)
        print("drone")
        self.imshow_bgr(thresh)
        
        # Get drone contours
        cnts = cv2.findContours(drone_img, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        
        im_cnts = []
        
        #for i, line in enumerate(cnts[1][0]):
        #    if line[3]!=-1:
        #        im_cnts.append(cnts[0][i])
        im_cnts = imutils.grab_contours(cnts)
        print(im_cnts)
        
        if debug_img:
            cv2.drawContours(img, im_cnts, -1, (0, 255, 0), 2)
            self.imshow_bgr(img)

        # Select largest drone feature from image
        drone_idx = 0
        biggest_box = 0
        box_ratio = 0
        drone_center = (0,0)
        for idx, points in enumerate(im_cnts):
            #if len(points) < 20:
            #    continue
            
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
                
            if box_sz > biggest_box:
                biggest_box = box_sz
                drone_idx = idx
                box_ratio = w/h
                drone_center = (x+w//2,y+h//2)
        
        
        if debug_img:
            self.imshow_bgr(img)
            self.drone_img.publish(bridge.cv2_to_imgmsg(img))

        return drone_center
        #"""
    
    
    

if __name__ == '__main__':
    rospy.init_node('drone_detector', anonymous=True)
    drone = droneDetector()
    #rospy.spin()
    #drone.test_image("drone.png")
    drone.test_images("../../images/landing/flights")
