import time
from picamera2 import Picamera2 

try:
    camera = Picamera2()
    camera.resolution = (640, 360)
    camera.framerate = 20

    time.sleep(2)

    camera.start_recording('/home/rehkt/BWSI-UAV-REHKT/BWSI/cameraimgs/my_vid.h264')

    camera.wait(15)

    camera.stop_recording()
    camera.stop_review()

    print("Recording complete!")
except Exception as e:
    print("ERROR")
    print(e)

