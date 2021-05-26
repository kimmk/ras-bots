import rospy
from std_msgs.msg import Empty
from landing import service_land, service_landResponse

rospy.Service("testservice", Empty, handle_service)

def handle_service():
    print("handling")
    return service_landResponse
