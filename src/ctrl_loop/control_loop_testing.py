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
from std_msgs.msg import Empty
import controls
import time

decisionNone = 0
decisionCorrection = 1
decisionGo = 2
magicNumber = 4
bridge = CvBridge()
heightcorrect = 0.3

class ControlState:
    def __init__(self):
        rospy.Subscriber("/image", Image, self.camera_callback)  # Tello camera 
        self.cmd_pub = rospy.Publisher('/tello/cmd_vel', Twist, queue_size=1)
        self.gateDetector = GateDetector()
        self.decisionTally = np.array([0,0,0])
        self.decisionTotals = 0
        self.lastGate = [0,0,(0,0)]
        self.target_dist = 0.7
        


    def move_around_gate(self, alpha, gate_dist, gate_x, gate_z):
        #         y+
        #         ^
        #    x- <- ->  x+
        #         v
        #         y-
        #   . = z+
        #   + = z-
        #   clockwise = az-      ?
        #   cntrclockwise = az+  ?
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
        saz += alpha *10.0
        print(alpha, gate_dist, gate_x, gate_z)
        
        sy += gate_x
        sz += -gate_z
        sx = min(gate_dist - self.target_dist, 1) #max speed =1
        
        self.cmd_pub.publish(controls.control(y = sx, x = sy, z = sz, az = saz))
        #self.cmd_pub.publish(controls.control(az = 1))
        #time.sleep(0.5)
        #self.cmd_pub.publish(controls.hold())
        
        
        #y=+1 == forward with ok speed

    def go_trough_gate(self):
        sx = 1
        self.cmd_pub.publish(controls.control(y=sx))
        time.sleep(2)
        self.cmd_pub.publish(controls.hold())
        time.sleep(0.5)
        return 1
        
    def camera_callback(self, img):
        cv2_img = bridge.imgmsg_to_cv2(img, "bgr8")
        gate = self.gateDetector.image_processing(cv2_img)
        
        decision = decisionNone
        dWeight = 1
        
        if gate is None:
            pass
        
        else:
            #angle,dist,(x,y) = gate
            #self.move_around_gate(angle,dist,x,y)
            #angle ranges from -0.1 to + 0.1
            self.lastGate = gate
            angle, dist, (x, y) = gate
            
            y = y + heightcorrect
            #print(angle)
            ######## Decider
            if abs(x)<0.05 and abs(y)<0.05 and abs(angle)<0.01 and abs(dist-self.target_dist)<0.1:
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
        if self.decisionTotals < magicNumber:
            return

        decision = np.argmax(self.decisionTally)
        #print("decision: " + str(decision))
        self.decisionTally = np.array([0,0,0])
        self.decisionTotals = 0
        angle, dist, (x, y) = self.lastGate
        y=y+heightcorrect
        if decision == decisionNone:
            pass
            ##this is the part where we pretend that we know what we are doing
        elif decision == decisionCorrection:
            self.move_around_gate(angle,dist,x,y)
        else: 
            self.go_trough_gate()
        #"""
#def handle_exit(signum, frame):
def handle_exit():
    
    cmd_pub = rospy.Publisher('/tello/cmd_vel', Twist, queue_size=1, latch = True)
    cmd_land = rospy.Publisher('/tello/land', Empty, queue_size=1)
    rospy.sleep(0.5)
    print("set to neutral + landing")
    cmd_land.publish(Empty())
    cmd_pub.publish(controls.hold())
    #sys.exit(0)
if __name__ == '__main__':
    
    rospy.init_node('gate_detector', anonymous=True)
    #signal.signal(signal.SIGINT, handle_exit)
    StateMachine = ControlState()
    rospy.on_shutdown(handle_exit)
    
    rospy.spin()
    
