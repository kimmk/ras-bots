import rospy
from geometry_msgs.msg import Twist


"""
use case:
import controls
cmd_pub = rospy.Publisher('/tello/cmd_vel', Twist, queue_size=1)
cmd_pub.publish(controls.control(z=z_value, az=rot_value))
cmd_pub.publish(controls.hold())


can be useful to add in main command:

def ctrlc(signum, frame):
    print('setting drone to neutral')
    cmd_pub.publish(controls.hold())
    exit (0);


signal.signal(signal.SIGINT, ctrlc)
"""

def control(x=0,y=0,z=0,ax=0,ay=0,az=0):
    msg = Twist()
    
    msg.linear.x=x
    msg.linear.y=y
    msg.linear.z=z
    
    msg.angular.x=ax
    msg.angular.y=ay
    msg.angular.z=az
    return msg

def rotz(az):
    msg = Twist()
    
    msg.linear.x=0
    msg.linear.y=0
    msg.linear.z=0
    
    msg.angular.x=0
    msg.angular.y=0
    msg.angular.z=az
    return msg


#redundant but nice to have
def hold():
    msg = Twist()
    
    msg.linear.x=0
    msg.linear.y=0
    msg.linear.z=0
    
    msg.angular.x=0
    msg.angular.y=0
    msg.angular.z=0
    return msg