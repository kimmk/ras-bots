"corresponds to a main file calling the detector and processing the controls"


#! /usr/bin/python
import sys
import rospy
# ROS Image message
from sensor_msgs.msg import Image
# ROS Image message -> OpenCV2 image converter
from cv_bridge import CvBridge, CvBridgeError
# OpenCV2 for saving an image
import cv2
import imutils
from std_msgs.msg import Empty
from std_msgs.msg import String
import time
from geometry_msgs.msg import Point
from sensor_msgs.msg import LaserScan
import numpy as np
import controls
import signal
from std_msgs.msg import Float64
from simple_pid import PID
#from tello_driver import TelloStatus

from geometry_msgs.msg import Twist



class fly_to_platform:
    def __init__(self):
        self.cmd_pub = rospy.Publisher('/tello/cmd_vel', Twist, queue_size=1, latch=True)
        self.searctime = None
        self.reset_metronome()

        P, I, D = 8.0, 3.0, 10.0
        self.align_pid_angle = PID(P, I, D)
        self.align_pid_dist = PID(P, I, D)
        self.align_pid_height = PID(P+2.0, I, D)
        self.align_pid_angle.output_limits = (-0.4, 0.4)
        self.align_pid_dist.output_limits = (-0.4, 0.4)
        self.align_pid_height.output_limits = (-0.4, 0.4)

        #self.recv_range_th = threading.Thread(target=self.recv_range)
        #self.recv_range_th.start()

    ### search pattern: drone flies backwards, likely that the drone is over the platform
    def search(self):
        if self.searctime is None:
            self.searctime = time.time()
        if time.time()-self.searctime < 1:
            pass
            #self.cmd_pub.publish(controls.control(az = 0.5))
        else:
            self.cmd_pub.publish(controls.control(y = -0.4))
            #self.cmd_pub.publish(controls.control(az = 0.4))

    ## "metronome" variable reset for rotation search
    def reset_metronome(self):
        self.metronome_sweepsize=7
        self.metronome = 0
        self.metronomedir = 1
        self.anglediff = 0

    ### "metronome" search pattern: drone rotates to both sides looking for the platform and flies slowly forward.
    ### Since is is to be called during the pseudo creeping line, it's unlikely that the platform will be below it
    def rotation_search(self):
        self.cmd_pub.publish(controls.control(az = np.sign(self.metronomedir)*0.6 , y = -0.2) )
        time.sleep(0.3)
        
        self.metronome += self.metronomedir
        print("metronome, dir", self.metronome, self.metronomedir)
        
        if abs(self.metronome) > self.metronome_sweepsize:
            self.metronomedir = -self.metronomedir
            self.metronome_sweepsize +=1
            
        #if self.metronome == 0 and self.metronomedir == 1:
        #    self.cmd_pub.publish(controls.control(y= -0.3))
        #    print("go forward")
        
    ##### return 1 if alignment succeeded, 0 if still ongoing
    ##### in: platform position: [%x offset, %y offset, widht in px)
    ##### offsets from image center
    def align_to_platform(self, platform_pos):
        self.reset_metronome()
        self.searctime = None
        platform_x, platform_y, platform_size = platform_pos
        platform_size = (platform_size - 70.0)/70.0
        self.align_pid_angle.setpoint = 0
        self.align_pid_dist.setpoint = 0
        self.align_pid_height.setpoint = 0.7

        print("platform_pos: ", platform_pos)



        saz = platform_x/2.0
        sz = 0.75*(platform_size-70)/70.0
        sy = 1.0*(-1)*(platform_y - 0.7)

        saz = self.align_pid_angle(-platform_x)
        sz = self.align_pid_height(platform_y)
        sy = self.align_pid_dist(platform_size)

        self.cmd_pub.publish(controls.control(az=saz, y=sy, z=sz))

        if abs(platform_x) < 0.15 and abs(platform_y) < 0.8 and abs(platform_y) > 0.6 and abs(platform_size) < 0.15:
            return 1
        return 0


    #### platform_x = % of platform-x center off of image center
    #### return 1 if flying towards, 0 if still aligning
    def fly_over_platform(self, platform_pos):
        if platform_pos is None:
            return 0
        platform_x, platform_y, platform_size = platform_pos
        self.searctime = None
        if platform_pos is None:
            return 0
        elif abs(platform_x) > 0.1 and abs(platform_y) > 0.8 and abs(platform_y) < 0.6 and platform_size > 60 and platform_size < 80:
            self.align_to_platform(platform_pos)
            return 0

        self.cmd_pub.publish(controls.hold())
        time.sleep(0.2)
        sx = 1
        self.cmd_pub.publish(controls.control(y=sx))
        return 1

    ## Implemented in carrier_run.py
    # def pseudo_creeping_line_sp(self, platform_pos):
    #     platform_x, platform_y, platform_size = platform_pos
    #     find_platform_flag = False
    #     if platform_pos is None and find_platform_flag is True:
    #         return 0
    #     elif abs(platform_x) > 0.1 and abs(platform_y) > 0.8 and abs(platform_y) < 0.6 and platform_size > 60 and platform_size < 80 and find_platform_flag is True:
    #         self.align_to_platform(platform_pos)
    #         return 0
        
    #     find_platform_flag = False
    #     # once aligned, perform go forward, stop at wall using LIDAR and turn around, then find platform again
    #     # lidar magic goes here, remember to set find_platform_flag True after turning around from wall
    #     #sx = 1
    #     #self.cmd_pub.publish(controls.control(y=sx))
    #     # see_wall() => find_platform_flag = True



    #     # exit the loop on battery low
    #     # battery_low = rospy.Subscriber("/tello/status", Image, drone_status_callback)
    #     # if battery_low is True:
    #     #   return 1



def handle_exit(signum, frame):
#def handle_exit():
    
    cmd_pub = rospy.Publisher('/tello/cmd_vel', Twist, queue_size=1, latch = True)
    cmd_land = rospy.Publisher('/tello/land', Empty, queue_size=1)
    rospy.sleep(1)
    print("set to neutral + landing")
    cmd_land.publish(Empty())
    cmd_pub.publish(controls.hold())
    sys.exit(0)

#def drone_status_callback(msg):
#    if msg is None:
#        return True
#    else:
#        return msg.is_battery_low


#if called directly as main, process drone image and perform run
def drone_camera_callback(msg):
    img = bridge.imgmsg_to_cv2(msg, "bgr8")
    img = imutils.resize(img, width=500)
    platform_x = platformDetector.give_platform_x(img)
    platform_run.align_to_platform(platform_x)
    #print("platform_x: ", platform_x)

if __name__ == '__main__':
    rospy.init_node('platform_detection_run')
    signal.signal(signal.SIGINT, handle_exit)
    import platform_detector
    # Instantiate CvBridge
    bridge = CvBridge()
    platformDetector = platform_detector.platformDetector()
    platform_run = fly_to_platform()
    rospy.Subscriber("/image", Image, drone_camera_callback)



    rospy.spin()