#!/usr/bin/python3
import time

""" Scripts """
from scripts import controls
""" ROS """
import rospy
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Empty
from std_msgs.msg import UInt8
from geometry_msgs.msg import Twist
""" Others """
from control_state import ControlState 
import cv2
import numpy as np

""" ROS TOPIC DEFINITIONS - PUBLISHERS """
cmd_vel = rospy.Publisher('/tello/cmd_vel', Twist, queue_size=2)
cmd_takeoff = rospy.Publisher('/tello/takeoff', Empty, queue_size=1)
cmd_land = rospy.Publisher('/tello/land', Empty, queue_size=1)

""" ROS TOPIC DEFINITIONS - SUBSCRIBERS """


""" CONTROL LOOP CODE """
def main():
    print('CTRL-LOOP INIT')
    ## THIS IS WHERE THE CODE FOR CONTROLLING THE STATES SHOULD BE, AND STATES SHOULD BE DEFINITIONS THAT CALL FUNCTIONS IN THE SCRIPT FOLDER

    ## Code is currently just for take off and landing
    time.sleep(5)
    controls.takeoff(cmd_takeoff)
    time.sleep(5)

    controls.control(cmd_vel, y=1)
    time.sleep(2)
    controls.control(cmd_vel)

    time.sleep(5)
    controls.land(cmd_land)

    


if __name__ == '__main__':
    try:
        # Initialize the ROS node in case the python is initialized from here
        rospy.init_node('jetbot_control', anonymous=True)
        main()
    except BaseException:
        traceback.print_exc()