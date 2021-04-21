import cv2
from djitellopy import Tello
import time

tello = Tello()
tello.connect()

tello.streamon()
frame_read = tello.get_frame_read()
#tello.takeoff()
for i in range(10):
    cv2.imwrite("picture{}.png".format(i), frame_read.frame)
    time.sleep(1)
    print(i)
    
tello.streamoff()
#tello.land()