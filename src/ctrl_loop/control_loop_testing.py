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
votecount = 6
bridge = CvBridge()
sloperun = 0


class ControlState:
    def __init__(self):
        rospy.Subscriber("/image", Image, self.camera_callback)  # Tello camera 
        self.cmd_pub = rospy.Publisher('/tello/cmd_vel', Twist, queue_size=1)
        self.gateDetector = GateDetector()
        self.decisionTally = np.array([0,0,0])
        self.decisionTotals = 0
        self.lastGate = [0,0,(0,0)]
        self.target_dist = 0.7
        
        self.justWentTrough = 0 
        self.reset_metronome()
        
        self.lastdecision = decisionNone
        
        
        
    def reset_metronome(self):
        self.metronome_sweepsize=4
        self.metronome = self.metronome_sweepsize 
        self.metronomedir = 1
        self.anglediff = 0
        

    def move_around_gate(self, alpha, gate_dist, gate_x, gate_z):
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
        
        alpha = np.sign(alpha)*min(abs(alpha*10), 0.3)
        sy += alpha 
        saz += -alpha
        
        print("alpha,dist,x,y" ,(alpha, gate_dist, gate_x, gate_z))
        
        #if abs(gate_x < 0.1):
        #    gate_x = np.sign(gate_x)*0.1
        
        #if abs(gate_z < 0.1):
        #    gate_z = np.sign(gate_z)*0.1
        
        control_multiple = 1.5
        
        gate_x = gate_x*control_multiple
        gate_z = gate_z*control_multiple
        gate_x = np.sign(gate_x)*min(abs(gate_x), 0.3)
        gate_z = np.sign(gate_z)*min(abs(gate_z), 0.3)
        
        sy += gate_x
        sz += -gate_z
        sx = min(gate_dist - self.target_dist, 0.2) #max speed =1
        sx = sx*control_multiple
        self.cmd_pub.publish(controls.control(y = sx, x = sy, z = sz, az = saz))
        #self.cmd_pub.publish(controls.control(az = 1))
        #time.sleep(0.5)
        #self.cmd_pub.publish(controls.hold())
        #self.reset_metronome()
        
        

    def go_trough_gate(self):
        sx = 1.7
        sz = 0.6
        stop = 1
        for i in range(5):
            print("GO")
        self.cmd_pub.publish(controls.hold())
        time.sleep(0.3)
        
        self.cmd_pub.publish(controls.control(y=sx/4, z = -sz/4))
        time.sleep(0.5)
        self.cmd_pub.publish(controls.control(y=sx/2, z = -sz/2))
        time.sleep(0.5)
        self.cmd_pub.publish(controls.control(y=3*sx/4, z = -3*sz/4))
        time.sleep(0.5)
        
        self.cmd_pub.publish(controls.control(y=sx, z = -sz))
        time.sleep(1.5)
        self.cmd_pub.publish(controls.control(y=-stop))
        time.sleep(1)
        self.cmd_pub.publish(controls.control(z = sz))
        time.sleep(1)
        self.cmd_pub.publish(controls.hold())
        time.sleep(1)
        self.reset_metronome()
        
        return 1
    
    
    
    
    def search(self):
        self.cmd_pub.publish(controls.control(az = np.sign(self.metronome)*0.5 ))
        time.sleep(0.2)
        
        self.metronome += self.metronomedir
        self.anglediff += self.metronomedir
        
        print("angle,metronome", self.anglediff, self.metronome)
        
        if abs(self.metronome) > self.metronome_sweepsize:
            self.metronomedir = -self.metronomedir
            self.metronome_sweepsize +=1
            
        if self.anglediff == 0 and self.metronome_sweepsize > 5:
            self.cmd_pub.publish(controls.control(y=0.8))
            time.sleep(1)
            if sloperun:
                time.sleep(3)
            print("go forward")
        
        if abs(self.metronome_sweepsize) > 8:
            handle_exit(None,None)
    
    
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
            
            y = y 
            x = x
            #print(angle)
            ######## Decider
            if abs(x)<0.05 and abs(y)<0.07 and abs(angle)<0.01 and abs(dist-self.target_dist)<0.1:
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
        
        if self.lastdecision is decisionCorrection and gate is not None:
            angle, dist, (x, y) = self.lastGate
            self.move_around_gate(angle,dist,x,y)
        if self.decisionTotals < votecount:
            return

        decision = np.argmax(self.decisionTally)
        self.lastdecision = decision
        #print("decision: " + str(decision))
        self.decisionTally = np.array([0,0,0])
        self.decisionTotals = 0
        angle, dist, (x, y) = self.lastGate
        y=y
        x = x
        if decision == decisionNone:
            
            self.search()
            self.justWentTrough = 0
            
            ##this is the part where we pretend that we know what we are doing
        elif decision == decisionCorrection:
            self.move_around_gate(angle,dist,x,y)
            self.justWentTrough = 0
        else:
            if not self.justWentTrough:
                self.justWentTrough = 1
                foo = self.go_trough_gate()
        #"""
def handle_exit(signum, frame):
#def handle_exit():
    
    cmd_pub = rospy.Publisher('/tello/cmd_vel', Twist, queue_size=1, latch = True)
    cmd_land = rospy.Publisher('/tello/land', Empty, queue_size=1)
    rospy.sleep(0.5)
    print("set to neutral + landing")
    cmd_land.publish(Empty())
    cmd_pub.publish(controls.hold())
    rospy.signal_shutdown("set to neutral + landing")
    sys.exit(0)
if __name__ == '__main__':
    
    rospy.init_node('gate_detector', anonymous=True)
    signal.signal(signal.SIGINT, handle_exit)
    StateMachine = ControlState()
    #rospy.on_shutdown(handle_exit)
    
    rospy.spin()
    
