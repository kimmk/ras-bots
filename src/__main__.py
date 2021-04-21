#!/usr/bin/python3
import rospy
from ctrl_loop import control_loop

def main():
    rospy.init_node('jetbot_control', anonymous=True)
    control_loop.main()


if __name__ == '__main__':
    main()