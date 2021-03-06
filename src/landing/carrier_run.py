#######This is the main control loop on the pc.


import os
import time
import signal
import sys
import cv2
import imutils
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import matplotlib.pyplot as plt
import rospy

from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64
from std_msgs.msg import Int8
from std_msgs.msg import Empty
from std_msgs.msg import String
from geometry_msgs.msg import Point
from geometry_msgs.msg import Twist


import controls
import platform_detector
import platform_detection_run
import lander
import socket
import threading



bridge = CvBridge()


class MainControl:
    def __init__(self):
        rospy.Subscriber("/image", Image, self.drone_camera_callback)  # Tello camera
        #rospy.Subscriber("/jetbot_image", Image, self.jetbot_camera_callback)  # Jetbot camera, not in use like this

        #rospy.Subscriber("/land/done", Int8, self.landing_callback)
        self.do_landing_pub = rospy.Publisher("/land/execute", Int8,  queue_size=1)
        self.jetbot_move_pub = rospy.Publisher("/jetbot_move", String, queue_size=1)

        self.drone_image = None
        #self.jetbot_image = None


        self.platformDetector = platform_detector.platformDetector()
        self.platform_run = platform_detection_run.fly_to_platform()

        self.cmd_land = rospy.Publisher('/tello/land', Empty, queue_size=1)
        self.cmd_takeoff = rospy.Publisher('/tello/takeoff', Empty, queue_size=1)
        self.cmd_pub = rospy.Publisher('/tello/cmd_vel', Twist, queue_size=1)

        self.start_search_timer = None


        self.wall_count = 0
        self.max_wallcount = 3
        self.lidar_reads = 0
        self.Drone_lidar = DroneLidar()



    def run_start(self):
        time.sleep(2)
        self.jetbot_move_into_arena()

        #self.both_scramble()

        self.cmd_takeoff.publish(Empty())
        time.sleep(2)
        self.jetbot_move_pub.publish("patrol")

        while self.wall_count < self.max_wallcount:
            self.drone_creeping_line()
        self.jetbot_move_pub.publish("stop")
        while not self.drone_search_jetbot():
            pass

        landed = self.drone_land()

        if landed == 0:
            self.drone_new_approach()
            landed = self.drone_land()
        ##self.jetbot_move_trough_target()

    def jetbot_move_into_arena(self):
        self.jetbot_move_pub.publish("forward")
        time.sleep(5)
        return 1

    def both_scramble(self):
        self.cmd_takeoff.publish(Empty())
        time.sleep(4.0)
        self.jetbot_move_pub.publish("circle")
        self.cmd_pub.publish(controls.control(y = 0.7, az = 0.7))
        time.sleep(4.0)
        self.cmd_pub.publish(controls.hold())
        time.sleep(1.0)

    def drone_creeping_line(self):

        #if maybe wall
        if self.Drone_lidar.detect_wall() == 2:
            self.jetbot_move_pub.publish("stop")
            self.cmd_pub.publish(controls.hold())

        #if definitely wall:
        #        turn around and search and align to jetbot
        if self.Drone_lidar.detect_wall() == 1:
            print("turn around")
            self.jetbot_move_pub.publish("stop")
            if self.wall_count % 2 == 0:
                self.cmd_pub.publish(controls.control(az=1))
            else:
                self.cmd_pub.publish(controls.control(az=-1))
            time.sleep(3.5)
            self.wall_count += 1
            while not self.drone_search_jetbot(True):
                pass

            if not self.wall_count == self.max_wallcount:
                self.jetbot_move_pub.publish("patrol")
                self.cmd_pub.publish((controls.control(z=0.8)))
                time.sleep(1)

        #if no wall
        if self.Drone_lidar.detect_wall() == 0:
            self.cmd_pub.publish(controls.control(y=0.8))
            self.jetbot_move_pub.publish("patrol")
            print("no wall")

        """
        if self.last_range is not None:
            range_time, range = self.last_range
            if time.time()-range_time < 2:
                if range < 1700:
                    self.lidar_reads +=1
                    if self.lidar_reads > 15:
                        if self.wall_count%2==0:
                            self.cmd_pub.publish(controls.control(az=-1))
                        else:
                            self.cmd_pub.publish(controls.control(az=1))
                        time.sleep(3.5)

                        find_platform_flag = True
                        self.wall_count += 1
                        while not self.drone_search_jetbot(True):
                            self.jetbot_move_pub.publish("stop")
                            pass
                        if not self.wall_count ==self.max_wallcount:
                            self.jetbot_move_pub.publish("patrol")
                            self.cmd_pub.publish((controls.control(z=0.7)))
                            time.sleep(1)
                        self.cmd_pub.publish(controls.hold())
                        time.sleep(0.5)
                else:
                    self.cmd_pub.publish(controls.control(y=0.8))
                    self.lidar_reads = 0
                    #self.jetbot_move_pub.publish("patrol")
            else:
                self.cmd_pub.publish(controls.hold())
        #"""

    def drone_search_jetbot(self, creeping_line = False):
        while self.drone_image is None:
            time.sleep(1)
            print("no drone image")
        platform_pos = self.platformDetector.give_platform_x(self.drone_image)
        if platform_pos is None:
            if self.start_search_timer is None:
                self.start_search_timer = time.time()
            if time.time()- self.start_search_timer > 1:
                print("searching")
                if creeping_line is True:
                    self.platform_run.rotation_search()
                else:
                    self.platform_run.search()
            return 0
            #self.drone_search_jetbot()
            #platform_pos = self.platformDetector.give_platform_x(self.drone_image)
        elif not self.platform_run.align_to_platform(platform_pos):
            self.start_search_timer = None
            #self.drone_search_jetbot()
            print("aligning")
            return 0
        # while align returns 0 == not complete, keep aligning
        #while not self.platform_run.align_to_platform(platform_pos):
        #    platform_pos = self.platformDetector.give_platform_x(self.drone_image)
        #    if platform_pos is None:
        #        self.drone_search_jetbot()
        #    print("aligning")


        return 1

    ### land, wait till done, reset the command
    ### return 1 if landing succesfull, 0 otherwise

    ### first start the landing procedure, so creating background, then fly over jetbot, then jetbot catches drone in image
    def drone_land(self):
        print("start land")
        self.do_landing_pub.publish(1)
        time.sleep(1)

        platform_pos = self.platformDetector.give_platform_x(self.drone_image)
        if platform_pos is None:
            while not self.drone_search_jetbot(False):
                pass
        while not self.platform_run.fly_over_platform(platform_pos):
            platform_pos = self.platformDetector.give_platform_x(self.drone_image)
            if platform_pos is None:
                while not self.drone_search_jetbot(False):
                    pass
            print("realigning")

        try:
            print("jetbot catch")
            landed = rospy.wait_for_message("/land/done", Int8)
        except rospy.exceptions.ROSInterruptException:
            print("landing interrupted")
            handle_exit(0,0)
            landed = 0

        self.do_landing_pub.publish(0)
        print("landed= ", landed)
        return landed


    def drone_new_approach(self):
        print("new approach")
        self.cmd_takeoff.publish(Empty)
        time.sleep(2.0)
        while not self.drone_search_jetbot(False):
            pass
        return 1

    def jetbot_move_trough_target(self):
        return 1

    def drone_camera_callback(self, msg):
        img = bridge.imgmsg_to_cv2(msg, "bgr8")
        img = imutils.resize(img, width=500)
        self.drone_image = img

    #def jetbot_camera_callback(self, msg):
    #    img = bridge.imgmsg_to_cv2(msg, "bgr8")
    #    self.jetbot_image = img
class DroneLidar:
    def __init__(self):
        self.close_threads = False
        self.recv_range_th = threading.Thread(target=self.recv_range)
        self.recv_range_th.start()
        self.last_range = None
        self.walltime = None

    def recv_range(self):
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.bind(("192.168.50.255", 7171))
        sock.settimeout(0.1)
        lastPrint = 0
        while not self.close_threads:
            try:
                self.last_range = (time.time(), int(sock.recv(1024)))
                if time.time() - lastPrint > 0.1:
                    print("range: {:.2f} {}".format(self.last_range[0], self.last_range[1]))
                    lastPrint = time.time()
            except socket.timeout:
                pass
        print("bye bye lidar")
    # uses last_range to determine if its detecting a wall
    #return 0 if no wall
    #       1 if definitely a wall
    #       2 if maybe a wall
    def detect_wall(self):
        # if measurement doesnt work, unsure so maybe a wall
        if self.last_range is None:
            self.walltime = None
            return 1

        last_measurement, range = self.last_range

        #if measurement doesnt work, unsure so maybe a wall
        if time.time() - last_measurement > 1:
            self.walltime= None
            return 2

        if range < 1500:
            if self.walltime is None:
                self.walltime = time.time()

            time_elapsed = time.time()-self.walltime
            if time_elapsed > 1:
                return 1
            if time_elapsed > 0.1:
                return 2
        else:
            self.walltime = None
        return 0






def handle_exit(signum, frame):
    Main_control.Drone_lidar.close_threads = True
    #MainControl.close_threads = True
    cmd_pub = rospy.Publisher('/tello/cmd_vel', Twist, queue_size=1, latch=True)
    cmd_land = rospy.Publisher('/tello/land', Empty, queue_size=1)
    do_landing_pub = rospy.Publisher("/land/execute", Int8, queue_size=1)
    jetbot_move_pub = rospy.Publisher("/jetbot_move", String, queue_size=1)

    rospy.sleep(1)
    print("set to neutral + landing")
    cmd_land.publish(Empty())
    cmd_pub.publish(controls.hold())
    do_landing_pub.publish(0)
    jetbot_move_pub.publish("stop")
    rospy.sleep(1)
    sys.exit(0)


if __name__ == '__main__':
    rospy.init_node("carrier_run")
    Main_control = MainControl()

    signal.signal(signal.SIGINT,handle_exit)
    Main_control.run_start()

    #rospy.on_shutdown(handle_exit(0,0))

    rospy.spin()







