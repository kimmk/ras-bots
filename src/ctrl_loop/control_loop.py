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
from geometry_msgs.msg import Pose.
from scripts.img_recon import GateDetector
from scripts.controls import *

decisionNone = 0
decisionCorrection = 1
decisionGo = 2
magicNumber = 10

class ControlState:
    def __init__():
        rospy.Subscriber("/image", Image, self.camera_callback)  # Tello camera 
        self.cmd_pub = rospy.Publisher('/tello/cmd_vel', Twist, queue_size=1)
        self.gateDetector = GateDetector()
        self.decisionTally = np.array([0,0,0])
        self.decisionTotals = 0
        self.lastGate = [0,0,(0,0)]

    def move_around_gate(self, alpha, gate_x, gate_z, gate_dist):
        #see what input is like, the convert alpha to deg, with 0 = center, +90 = left, -90 = right
        #convert x and z to % of image
        #set speeds to 0
        sx,sy,sz,saz = 0,0,0,0
        #TODO check signs for all
        sy += alpha  #at 90 deg, speed=max=pi/2
        saz += alpha
        sy += gate_x
        sz += -gate_z
        sx = min(gate_dist - self.target_dist, 1) #max speed =1
        self.cmd_pub.publish(controls.control(y = sx, x = sy, z = sz, az = saz))
        time.sleep(0.5)
        self.cmd_pub.publish(controls.hold())

    def go_trough_gate(self):
        sx = 1
        self.cmd_pub.publish(controls.control(y=sx))
        time.sleep(2)
        self.cmd_pub.publish(controls.hold())
        time.sleep(0.5)
        return 1
        
    def camera_callback(self, img):
        gate = self.gateDetector.image_processing(img)
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
        if self.decisionTotals < magicNumber
            return

        decision = np.argmax(self.decisionTally)
        self.decisionTally = np.array([0,0,0])
        self.decisionTotals = 0
        angle, dist, (x, y) = self.lastGate
        if decision == decisionNone:
            ##this is the part where we pretend that we know what we are doing
        elif decision == decisionCorrection:
            self.move_around_gate(angle,dist,x,y)
        else: 
            self.go_trough_gate()

if __name__ == '__main__':
    signal.signal(signal.SIGINT, handle_exit)
    rospy.init_node('gate_detector', anonymous=True)
    rospy.spin()
