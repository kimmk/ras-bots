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
from geometry_msgs.msg import Pose
from img_recon import GateDetector
from geometry_msgs.msg import Twist
import controls
import time
from controls import *
import logging
import logging.config
from datetime import datetime

decisionNone = 0
decisionCorrection = 1
decisionGo = 2
votecount = 7
bridge = CvBridge()
heightcorrect = 0

################################### DEVEL VARS & DEFS
#devel mode = 1: picking single images from file to process
#devel mode = 0: picking images from subscriber image stream
devel_mode = 1
debug_mode = 1
dateTimeFormat = "%d-%m-%Y_%H:%M:%S"

class ControlState:
    def __init__(self):
        logging.config.fileConfig('logging.conf')
        mylogger = logging.getLogger('decisionControl')
        logging.info("----------------------")
        logging.info("Decision Control START")
        self.cmd_pub = rospy.Publisher('/tello/cmd_vel', Twist, queue_size=1)
        self.gateDetector = GateDetector()
        self.decisionTally = np.array([0,0,0])
        self.decisionTotals = 0
        self.lastGate = [0,0,(0,0)]
        self.target_dist = 0.8
        self.metronome = 0

        if devel_mode:
            logging.info("DEVEL MODE")
            #self.test_images("../../images/tello_gates")
            self.test_video("../../images/gate_vids/video.avi")
        else:
            rospy.Subscriber("/image", Image, self.camera_callback)  # Tello camera 
        

    def test_video(self, filepath):
        cap = cv2.VideoCapture(filepath)
        while(cap.isOpened()):
            ret, frame = cap.read()
            self.camera_callback(frame)
            if cv2.waitKey(100) & 0xFF == ord('q'):
                break
        cap.release()
        
    def test_image(self, filepath):
        cv2_img = cv2.imread(filepath)
        if cv2_img is not None:
            self.camera_callback(cv2_img)
            
    def test_images(self, path):
        for i in range(0,100):
            file = "/picture" + str(i) + ".png"
            self.test_image(path+file)

    def move_around_gate(self, alpha, gate_x, gate_z, gate_dist):
        logging.info("Decision Control - Angle Correction")
        #         y+
        #         ^
        #    x- <- ->  x+
        #         v
        #         y-
        #   . = z+
        #   + = z-
        #   clockwise = az+      ?
        #   cntrclockwise = az-  ?
        #
        
        
        #   looking from gate: if drone is left:  alpha < 0
        #                                  right: alpha > 0
        #looking from drone: if gate is left:  x < 0
        #                               right: x > 0
        #                               high:  z < 0
        #                               low:   z > 0
        #print("move")
        #print(alpha,gate_x,gate_z,gate_dist)
        #see what input is like, the convert alpha to deg, with 0 = center, +90 = left, -90 = right
        #convert x and z to % of image
        #set speeds to 0
        sx,sy,sz,saz = 0,0,0,0
        #TODO check signs for all
        #angle ranges from -0.1 to + 0.1
        sy += alpha *10.0 
        saz += -alpha *10.0
        print(alpha, gate_dist, gate_x, gate_z)
        
        sy += gate_x
        sz += -gate_z
        sx = min(gate_dist - self.target_dist, 1) #max speed =1
        
        self.cmd_pub.publish(controls.control(y = sx, x = sy, z = sz, az = saz))
        #self.cmd_pub.publish(controls.control(az = 1))
        #time.sleep(0.5)
        #self.cmd_pub.publish(controls.hold())

    def go_trough_gate(self):
        logging.info("Decision Control - Go Through Gate")
        sx = 1
        sz = 0.6
        self.cmd_pub.publish(controls.control(y=sx, z = -sz))
        time.sleep(1.5)
        self.cmd_pub.publish(controls.control(y=sx, z = sz))
        time.sleep(1.5)
        self.cmd_pub.publish(controls.hold())
        time.sleep(1.5)
        for i in range(10):
            print("GO")
        
        return 1

    def search(self):
        self.cmd_pub.publish(controls.control(az = np.sign(self.metronome)*0.6 ))
        time.sleep(0.5)
        
        self.metronome += 1
        if self.metronome > 10:
            self.metronome = -15
        
    def camera_callback(self, img):
        logging.info("Camera Callback")
        if devel_mode:
            cv2_img = img
        else:
            cv2_img = bridge.imgmsg_to_cv2(img, "bgr8")
        gate = self.gateDetector.image_processing(cv2_img)
        decision = decisionNone
        dWeight = 1

        if gate is None:
            pass
        else:
            self.lastGate = gate
            angle, dist, (x, y) = gate
            ######## Decider
            if abs(x)<0.05 and abs(y)<0.05 and abs(angle)<0.1 and abs(dist-self.target_dist)<0.1:
                # foo = self.go_trough_gate()
                # print("gogogo")
                decision = decisionGo
            else:
                # self.move_around_gate(angle,x,y,dist)
                # print("adjust")
                decision = decisionCorrection
                dWeight = 2
        self.decisionTotals += 1
        self.decisionTally[decision] += dWeight
        if self.decisionTotals < votecount:
            return

        logging.info("Total Decisions: "+str(self.decisionTotals))
        decision = np.argmax(self.decisionTally)
        logging.info("DecisionTally: "+np.array2string(self.decisionTally))
        logging.info("Decision: "+str(decision))
        if self.lastGate != None:
            logging.info(self.lastGate)

        self.decisionTally = np.array([0,0,0])
        self.decisionTotals = 0
        angle, dist, (x, y) = self.lastGate

        if decision == decisionNone:
            self.search()
        elif decision == decisionCorrection:
            self.move_around_gate(angle,dist,x,y)
        else: 
            foo = self.go_trough_gate()

def handle_exit(signum, frame):
    logging.info("Decision Control - Handle Exit")
    cmd_pub = rospy.Publisher('/tello/cmd_vel', Twist, queue_size=1, latch = True)
    cmd_land = rospy.Publisher('/tello/land', Empty, queue_size=1)
    rospy.sleep(0.5)
    cmd_land.publish(Empty())
    cmd_pub.publish(controls.hold())
    
    
    print("set to neutral + landing")
    rospy.sleep(1)
    sys.exit(0)

if __name__ == '__main__':
    rospy.init_node('gate_detector', anonymous=True)
    signal.signal(signal.SIGINT, handle_exit)
    StateMachine = ControlState()
    rospy.spin()
    
