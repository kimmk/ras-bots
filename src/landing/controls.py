import rospy
from std_msgs.msg import Empty
from std_msgs.msg import UInt8
from geometry_msgs.msg import Twist

"""
use case:
import controls
# """

# def takeoff(cmd_takeoff):
#     print("TAKEOFF")
#     cmd_takeoff.publish(Empty())

# def land(cmd_land):
#     print("LAND")
#     cmd_land.publish(Empty())

def control(x=0,y=0,z=0,ax=0,ay=0,az=0):
    msg = Twist()
    msg.linear.x=x
    msg.linear.y=y
    msg.linear.z=z
    msg.angular.x=ax
    msg.angular.y=ay
    msg.angular.z=az
    return msg

# def rotz(cmd_vel,az):
#     cmd_vel = rospy.Publisher('/tello/cmd_vel', Twist, queue_size=1)
#     msg = Twist()
#     msg.linear.x=0
#     msg.linear.y=0
#     msg.linear.z=0
#     msg.angular.x=0
#     msg.angular.y=0
#     msg.angular.z=az
#     #sendVel(msg)
#     cmd_vel.publish(msg)

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
