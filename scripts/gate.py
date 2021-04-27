#!/usr/bin/env python

import rospy
# ROS Image message
from sensor_msgs.msg import Image
# ROS Image message -> OpenCV2 image converter
from cv_bridge import CvBridge, CvBridgeError
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import numpy as np
import matplotlib.pyplot as plt
import cv2

# Instantiate CvBridge
bridge = CvBridge()
cnt = 0


def gatefilter(img):
    width = int(img.shape[1] * 50 / 100)
    height = int(img.shape[0] * 50 / 100)
    dim = (width, height)
    img1 = cv2.resize(img, dim, interpolation = cv2.INTER_AREA)
    img_hsv = cv2.cvtColor(img1, cv2.COLOR_BGR2HSV)
    #plt.imshow(img_hsv)
    lower_range = np.array([20, 100, 90])  # Set the Lower range value of color in BGR
    upper_range = np.array([70, 255,255])   # Set the Upper range value of color in BGR
    mask = cv2.inRange(img_hsv,lower_range,upper_range) # Create a mask with range
    result = cv2.bitwise_and(img_hsv,img_hsv,mask = mask)  # Performing bitwise and operation with mask in img variable
    gray = cv2.cvtColor(result,cv2.COLOR_BGR2GRAY)
    #gray = np.float32(gray)
    #plt.imshow(gray)
    (thresh, black) = cv2.threshold(gray, 127, 255, cv2.THRESH_BINARY)
    return black

def gatedetection(img_filtered, img_normal):
    width = int(img_normal.shape[1] * 50 / 100)
    height = int(img_normal.shape[0] * 50 / 100)
    dim = (width, height)
    img_normal = cv2.resize(img_normal, dim, interpolation = cv2.INTER_AREA)
    arr = np.uint8(img_filtered)
    ##### or we if we want the dilated version
    #arr = np.uint8(dilateV)
    _, contours, _ = cv2.findContours(arr, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
    print(len(contours))
    rects = []
    for c in contours:
        rect = cv2.boundingRect(c)
        contarea = cv2.contourArea(c)
        rects.append([rect, contarea])

    maximcontarea = 0.0
    for i in rects:

        if i[1] > maximcontarea:
            maximcontarea = i[1]
            k = i[0]
    print(maximcontarea)
    print(k)

    dummy2 = img_normal.copy()

    x,y,w,h = k
    cv2.rectangle(dummy2,(x,y),(x+w,y+h),(0,255,0),2)
    cv2.putText(dummy2,'Gate detected',(x+w+10,y+h),0,0.9,(0,255,0))
    #plt.imshow(cv2.cvtColor(dummy2, cv2.COLOR_BGR2RGB))
    cv2.imwrite('images/Gate detection {}.jpeg'.format(cnt), cv2.cvtColor(dummy2, cv2.COLOR_BGR2RGB))
    #cv2.imshow('ImageWindow', cv2.cvtColor(dummy2, cv2.COLOR_BGR2RGB))
    #cv2.waitKey()
    #calculate the center of the rectangle so we compare it with the center of the image
    
    dummy3 = dummy2.copy()
    centerx = int((x+x+w)/2)
    centery = int((y+y+h)/2)

    centerximage = int(width/2)
    centeryimage = int(height/2)

    image = cv2.circle(dummy3, (centerx,centery), 10, (0, 0, 255), -1)
    image = cv2.circle(dummy3, (centerximage,centeryimage), 10, (255, 0, 0), -1)
    #plt.imshow(cv2.cvtColor(image, cv2.COLOR_BGR2RGB))

    cv2.imwrite('images/Detected gate with centers {}.jpeg'.format(cnt), dummy3)
    if centerx > centerximage:
        print("Gate is on the right side ---> Drone should move to the right")
    elif centerx < centerximage:
        print("Gate is on the left side ---> Drone should move to the left")
    else:
        print("Drone is alligned with the gate")


def image_callback(msg):
    try:
        # Convert your ROS Image message to OpenCV2
        cv2_img = bridge.imgmsg_to_cv2(msg, "bgr8")
    except CvBridgeError,e:
        print(e)
    else:
        global cnt
        cnt = cnt + 1
        black = gatefilter(cv2_img)
        gatedetection(black, cv2_img)


def main():
    rospy.init_node('image_listener')
    
    # Define your image topic
    image_topic = "/image"
    # Set up your subscriber and define its callback
    rospy.Subscriber(image_topic, Image, image_callback)

    # Spin until ctrl + c
    rospy.spin()

if __name__ == '__main__':
    main()
