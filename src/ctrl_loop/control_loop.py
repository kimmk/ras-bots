#!/usr/bin/python3
import time
from djitellopy import Tello


""" Scripts """
from scripts import controls
""" ROS """
import rospy
from std_msgs.msg import Empty
from geometry_msgs.msg import Twist
""" Others """
from control_state import ControlState 

"""
Main Control Machine State Loop

"""

def main():
    tello = Tello()
    tello.connect()
    print('CTRL-LOOP INIT')

    emptyMsg = Empty()

    cmd_vel = rospy.Publisher('/tello/cmd_vel', Twist, queue_size=1)
    cmd_takeoff = rospy.Publisher('/tello/takeoff', Empty, queue_size=1)
    cmd_land = rospy.Publisher('/tello/land', Empty, queue_size=1)
    
    time.sleep(5)
    
    cmd_takeoff.publish(emptyMsg)
    time.sleep(5)



    time.sleep(5)
    cmd_land.publish(emptyMsg)


if __name__ == '__main__':
    try:
        # Initialize the ROS node
        rospy.init_node('jetbot_control', anonymous=True)
        main()
    except BaseException:
        traceback.print_exc()