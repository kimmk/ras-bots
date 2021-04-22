import numpy as np
import controls
import rospy
import cv2
import time
from geometry_msgs.msg import Twist



#note on drone commands: y = front-back, x is side-side
#here used: sx = front-back, sy = side-side
class control:
    def __init__(self):
        #rospy.init_node('drone_control')
        self.cmd_pub = rospy.Publisher('/tello/cmd_vel', Twist, queue_size=1)
    
    #search pattern, currently rotate in place
    def search_pattern():
        self.cmd_pub.publish(controls.control(az =0.5))
    
    
    
    
    #in: angle_from_gate, image_center_of_gate_x, image_center_of_gate_z
    #out: publish translations and rotations to tello
    def move_around_gate(self, alpha, gate_x, gate_z, gate_dist):
        #see what input is like, the convert alpha to deg, with 0 = center, +90 = left, -90 = right
        #convert x and z to % of image
        
        target_dist = 1 #m
        
        #set speeds to 0
        sx,sy,sz,saz = 0,0,0,0
        
        #check signs for all
        sy += alpha/90.0  #at 90 deg, speed=max=1
        saz += alpha/90.0
        
        sy += gate_x
        sz += -gate_z
        
        sx = min(gate_dist - target_dist, 1) #max speed =1
        
        self.cmd_pub.publish(controls.control(y = sx, x = sy, z = sz, az = saz))
    
    def _align_to_platform(self, platform_x):
        saz = platform_x
        self.cmd_pub.publish(controls.control(az = saz))
        print("align" + str(saz))
    
    
    #platform_x = % of platform-x center off of image center
    def fly_to_platform(self, platform_x):
        
        if abs(platform_x) > 0.05:
            self._align_to_platform(platform_x)
            
        else:
            sx = 1
            saz = platform_x/3.0
            self.cmd_pub.publish(controls.control(y=sx, az = saz))
            print("fly" + str(saz))
            #time.sleep(3)
            #self.cmd_pub.publish(controls.hold())
            #return 1
        
        
        
        