from picamera2 import Picamera2
import cv2
import time

picam2 = Picamera2(0)
picam2.start()

try:
    while True:
        frame = picam2.capture_array()
        cv2.imshow("Live Camera", frame)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('s'):  # Press 's' to save photo
            filename = f"photo_{int(time.time())}.jpg"
            picam2.capture_file(filename)
            print(f"Saved {filename}")
        elif key == ord('q'):  # Press 'q' to quit
            break
finally:
    picam2.stop()
    cv2.destroyAllWindows()
