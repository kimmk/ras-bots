#!/usr/bin/python3
import sys, time
import numpy as np
import cv2
import roslib
import rospy
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import Image

class image_stream:
    def __init__(self):
        # topic where we publish
        #self.image_pub = rospy.Publisher("/output/image_raw/compressed", CompressedImage)
        # self.bridge = CvBridge()

        # subscribed Topic
        self.subscriber = rospy.Subscriber("/tello/image_raw", Image, self.callback,  queue_size = 1)
        print ("subscribed to /camera/image/compressed")

    def callback(self, message):
        print ('received image of type: "%s"' % ros_data.format)

def main(args):
    '''Initializes and cleanup ros node'''
    ic = image_stream()
    rospy.init_node('image_feature', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print ("Shutting down ROS Image feature detector module")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)