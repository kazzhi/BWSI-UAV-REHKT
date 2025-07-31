import asyncio
import numpy as np
import math
from mavsdk import System
from mavsdk import Offboard
import time
import cv2
from picamera2 import Picamera2


# wget https://github.com/mavlink/MAVSDK/releases/latest/download/mavsdk-server-linux-arm64
# chmod +x mavsdk-server-linux-arm64
# ./mavsdk-server-linux-arm64 serial:///dev/serial0:921600

###########
#CONSTANTS#
###########
MAX_YAW_SPEED = 90.0 # degrees per second, positive facking up
MAX_X_SPEED = 1.0 # meters per second, forward
MAX_Y_SPEED = 1.0 # meters per second, right
MAX_Z_SPEED = 1.0 # meters per second, down

TAKEOFF_ALTITUDE = 1.0 # meters
TAKEOFF_TIME = 10 # seconds

width, height = 640, 380 # pixels

drone = System() # Define the drone system
camera = None

#PID Constants
kp_x = 0.0
kp_y = 0.0
kp_z = 0.0

kd_x = 0.0
kd_y = 0.0
kd_z = 0.0

LOW = np.array([250, 250, 250])  # Lower image thresholding bound
HI = np.array([255, 255, 255])   # Upper image thresholding bound

kerneld = np.ones((30, 30), np.uint8)
kernele = np.ones((20, 20), np.uint8)

############

def pid():
    return

"""
Detects a line and returns the x, y point on the line
vx vy normalized direction vector
returns none if no line is detected
"""
def detect_line():
    try:
        image = camera.capture_array()
        image = cv2.dilate(image, kerneld, iterations = 1)
        image = cv2.erode(image, kernele, iterations = 1)
        mask = cv2.inRange(image, LOW, HI)
        image = cv2.bitwise_and(image, image, mask=mask)
        image = cv2.GaussianBlur(image, (5,5), 0)
        image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        _, image = cv2.threshold(image,245,255,cv2.THRESH_BINARY)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cnt_sort = lambda cnt: (max(cv2.minAreaRect(cnt)[1]))
        sorted_contours = sorted(contours, key=cnt_sort, reverse=True)

        if len(sorted_contours) == 0:
            print("No line detected")
            return None

        all_points = np.vstack(sorted_contours[0])
        [vx, vy, x, y] = cv2.fitLine(all_points, cv2.DIST_L2, 0, 0.01, 0.01)
    except Exception as e:
        print("ERROR detecting line!")
        print(f"Error: {e}")
        return None

    return [vx, vy, x, y]


"""
IN PROGRESS: 
Calculate the offset from the midpoint of the camera frame to the closest point to the line 
Set xy velocities so that the midpoint of the camera is on top of that point on the line
(regression line midpoint is on the center of the camera frame)
Calculate angle error compared to straight line
Yaw that position
Fed into PID to tune the overshoots
"""
def get_velocity(vx, vy, x, y):
    return


"""
Initiate Picamera
Maybe use mavlink camera?
"""
def initiate_cam():
    global camera
    try:
        camera = Picamera2()
        # Change config if needed for cv2 processing
        camera_config = camera.create_still_configuration(main={"size": (width, height)}) # Adjust resolution as needed
        camera.configure(camera_config)
        camera.start()
        time.sleep(0.5)
        print("Camera initialized!")
    except Exception as e:
        print("Camera failed to initialize.")
        print(f"Error: {e}")

"""
Connects to drone via rGPC with serial port
Takes off first to 1 meter
Starts offboard mode
Continuously calls get_velocity to determine setpoints
First does roll/pitch then does yaw
"""
async def run():
    await drone.connect(system_address="serial:///dev/ttyAMA0:921600")
    print("Waiting for drone to connect...")

    async for state in drone.core.connection_state():
        if state.is_connected:
            print("Drone discovered!")
            break

        
    print("Setting parameters...")
    await drone.action.set_takeoff_altitude(TAKEOFF_ALTITUDE)

    print("Arming...")
    await drone.action.arm()
    print("Successfully Armed")

    print("Taking off...")
    await drone.action.takeoff()
    await asyncio.sleep(TAKEOFF_TIME) # Pause for 8 seconds...

    print("Setting position setpoint for offboard start...")
    await drone.offboard.set_position_ned(
        PositionNedYaw(north_m = 0.0, east_m=0.0, down_m=0.0, yaw_deg=0.0)
    )

    await drone.offboard.start()

    # First detects line, if no line detected then abort
    # If line detected, computes the vx, vy, and yaw (PID Tuned)
    # Feeds them into velocity body yaw speed
    # waits 1 second
    while True:
        print("\nStarting offboard calculation!")
        result = detect_line()
        if not result:
            print("Unable to detect line")
            break
        
        vx, vy, x, y = result
        vel_x, vel_y, yaw_s = get_velocity(vx, vy, x, y)

        print(f"Forward velocity: {vel_x}, Right velocity: {vel_y}, Yaw speed: {yaw_s}")
        await drone.offboard.set_velocity_body(
            VelocityBodyYawspeed(forward_m_s=vel_x, right_m_s=vel_y, down_m_s=0.0, yawspeed_deg_s=yaw_s)
        )
        await asyncio.sleep(1)


    print("\nOperation finished! Landing...")
    await drone.action.land()

if __name__ == "__main__":
    asyncio.run(initiate_cam())
    asyncio.run(run())
