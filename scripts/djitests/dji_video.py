import time, cv2
from threading import Thread
from djitellopy import Tello

tello = Tello()

tello.connect()

keepRecording = True
tello.streamon()
frame_read = tello.get_frame_read()

def videoRecorder():
    # create a VideoWrite object, recoring to ./video.avi
    height, width, _ = frame_read.frame.shape
    video = cv2.VideoWriter('video2.avi', cv2.VideoWriter_fourcc(*'XVID'), 30, (width, height))

    while keepRecording:
        video.write(frame_read.frame)
        time.sleep(1 / 30)

    video.release()

# we need to run the recorder in a seperate thread, otherwise blocking options
#  would prevent frames from getting added to the video
recorder = Thread(target=videoRecorder)
recorder.start()

#tello.takeoff()
#tello.move_up(100)
#tello.rotate_counter_clockwise(360)
#tello.land()



for i in range(12):
    time.sleep(10)
    print(10*i)bind failed: Address already in use

keepRecording = False
recorder.join()