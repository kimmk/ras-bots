import numpy as np
import controls
import rospy
import cv2
import time
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose
import signal
import sys



#note on drone commands: y = front-back, x is side-side
#here used: sx = front-back, sy = side-side
class control:
    def __init__(self):
        self.target_dist = 0.8
        rospy.init_node('drone_control')
        self.cmd_pub = rospy.Publisher('/tello/cmd_vel', Twist, queue_size=1)
        rospy.Subscriber("/gate_pose", Pose, self.detector_callback)
    
    
    def detector_callback(self,msg):
    
        x = msg.position.x
        y = msg.position.y = x
        dist = -msg.position.z
        angle = msg.orientation.w
    
        if abs(x)<0.05 and abs(y)<0.05 and abs(angle)<0.1 and abs(dist-self.target_dist)<0.1:
            foo = self.go_trough_gate()
            print("gogogo")
        else:
            self.move_around_gate(angle,x,y,dist)
            print("adjust")
    
    
    #search pattern, currently rotate in place
    def search_pattern(self):
        self.cmd_pub.publish(controls.control(az =0.5))
        
        
    #in: angle_from_gate, image_center_of_gate_x, image_center_of_gate_z
    #out: publish translations and rotations to tello
    
    
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
    
    def go_trough_gate(self):
        sx = 1
        self.cmd_pub.publish(controls.control(y=sx))
        time.sleep(2)
        self.cmd_pub.publish(controls.hold())
        return 1
        
def handle_exit(signum, frame):
    controls.hold()
    print("put to neutral")
    sys.exit(0)
    


if __name__ == '__main__':
    signal.signal(signal.SIGINT, handle_exit)
    controller = control()
    
    rospy.spin()
        
        