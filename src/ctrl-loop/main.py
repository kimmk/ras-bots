#!/usr/bin/python3
import sys
import rospy

""" Scripts """
from scrits import controls

""" ROS Messages """
from std_msgs.msg import Empty

"""
Main Control Machine State Loop
"""

def main():
    print('CTRL-LOOP INIT')


if __name__ == '__main__':
    try:
        main()
    except BaseException:
        traceback.print_exc()