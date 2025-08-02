from picamera2 import Picamera2
import cv2
import time
import numpy as np
from cv2 import aruco
# Initialize PiCamera2
picam2 = Picamera2(0)
video_config = picam2.create_video_configuration(main={"size": (640, 480)})
picam2.configure(video_config)
picam2.start()
ARUCODICT = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_100)
# Let camera warm up
time.sleep(1)
kernel = np.ones((3, 3), np.uint8)
def grab_tags(tag):
    global IDS
    # Preprocess
    if tag is None:
        print("Error: Image is None.")
        return []
    tag = cv2.cvtColor(tag, cv2.COLOR_BGR2GRAY)
    
    # tag = cv2.convertScaleAbs(tag, alpha=1.0, beta=-50)
    # tag = cv2.erode(tag, kernel)


    detectorParams = aruco.DetectorParameters()
    detectorParams.aprilTagCriticalRad = 0.1 #default 0.17
    detectorParams.aprilTagMaxLineFitMse = 100 #default 10
    detectorParams.aprilTagMaxNmaxima = 15 #default 10
    detectorParams.polygonalApproxAccuracyRate = 0.05 #default 0.03
    detectorParams.useAruco3Detection = True
    detector = aruco.ArucoDetector(ARUCODICT, detectorParams)
    corners, ids, rejected = detector.detectMarkers(tag)

    return ids

# Set up video writer (adjust FPS and codec as needed)

cv2.namedWindow("Detection Window", cv2.WINDOW_NORMAL)  # or cv2.WINDOW_AUTOSIZE

# Main loop
try:
    while True:
        # Capture frame
        frame = picam2.capture_array()

        # Save video (OpenCV expects BGR)
        ids = grab_tags(frame)

        if ids is not None:
            text = f"IDS DETECTED: {ids.flatten()}"
        else:
            text = "NO IDS FOUND"
        
        cv2.putText(frame, text, (20, 30), cv2.FONT_HERSHEY_SIMPLEX, 
                    0.8, (0, 255, 0), 2)
        cv2.imshow("Detection Window", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break




finally:
    # Release everything

    cv2.destroyAllWindows()
    picam2.stop()
