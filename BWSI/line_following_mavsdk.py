import asyncio
import numpy as np
import math
from mavsdk import System
from mavsdk import offboard
from mavsdk.offboard import VelocityBodyYawspeed, OffboardError
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
MAX_X_SPEED = 0.5 # meters per second, forward
MAX_Y_SPEED = 0.5 # meters per second, right
MAX_Z_SPEED = 1.0 # meters per second, down

TAKEOFF_ALTITUDE = 1.0 # meters
TAKEOFF_TIME = 8
IMAGE_WIDTH, IMAGE_HEIGHT = 640, 360 # pixels

CENTER = np.array([IMAGE_WIDTH//2, IMAGE_HEIGHT//2]) # Center of the image frame. We will treat this as the center of mass of the drone
EXTEND = 100 # Number of pixels forward to extrapolate the line

drone = System() # Define the drone system
camera = None

LOOP_TIME = 0.05

#PID Constants
KP_X = 0.001
KP_Y = 0.001
KP_Z = 0.75
KP_W_Z = 3.5

KD_X = 0.00015
KD_Y = 0.00015
KD_Z = 0.015
KD_W_Z = 0.2

prev_x_error = 0
prev_y_error = 0
prev_z_error = 0
prev_angle_error = 0

latest_altitude = None
first_altitude = None

LOW = np.array([250, 250, 250])  # Lower image thresholding bound
HI = np.array([255, 255, 255])   # Upper image thresholding bound

KERNEL_D = np.ones((30, 30), np.uint8)
KERNEL_E = np.ones((20, 20), np.uint8)

R_dc2bd = np.array([[0.0, 1.0, 0.0, 0.0], 
                      [-1.0, 0.0, 0.0, 0.0], 
                      [0.0, 0.0, 1.0, 0.0], 
                      [0.0, 0.0, 0.0, 1.0]]) 
DETECT = 0
############

async def subscribe_position(drone):
    """Subscribes to position updates and updates the global variable."""
    global latest_altitude, first_altitude
    async for altitude in drone.telemetry.altitude():
        print("altitude updated!!")
        if first_altitude is None:
            first_altitude = altitude
        latest_altitude = altitude

def pid(error, angle_error):
        global prev_x_error, prev_y_error, prev_z_error, prev_angle_error
        # Set linear velocities (downward camera frame)
        vx = KP_X * error[0]
        vy = KP_Y * error[1]

        vx += KD_X * (error[0]-prev_x_error)/LOOP_TIME
        vy += KD_Y * (error[1]-prev_y_error)/LOOP_TIME

        prev_x_error = error[0]
        prev_y_error = error[1]

        # Set angular velocity (yaw)
        wz = KD_W_Z * angle_error

        wz += KD_W_Z * (angle_error-prev_angle_error)/LOOP_TIME

        prev_angle_error = angle_error

        print(f"error: {str(error)}, angle error: {str(angle_error)}")
        return vx, vy, wz

"""
Detects a line and returns the x, y point on the line
vx vy normalized direction vector
returns none if no line is detected
"""
def detect_line():
    try:
        image = camera.capture_array()
        image = cv2.dilate(image, KERNEL_D, iterations = 1)
        image = cv2.erode(image, KERNEL_E, iterations = 1)
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
calculate errors and feed into pid
returns vels in drone body frame
"""
def get_velocity(vx, vy, x, y):

    line_point = np.array([x, y])
    line_dir = np.array([vx, vy])
    line_dir = line_dir * (1/np.linalg.norm(line_dir))  # Ensure unit vector

    line_point = line_point.T
    line_point = line_point[0]
    line_dir = line_dir.T
    line_dir = line_dir[0]

    if line_dir[1] < 0: # ensure points "down" aka postive y direction 
        line_dir = -line_dir

    # Target point EXTEND pixels ahead along the line direction
    target = line_point + EXTEND * line_dir
    
    # Error between center and target
    error = target - CENTER

    # Get angle between y-axis and line direction
    # Positive angle is counter-clockwise
    angle_error = math.atan2(-line_dir[0], line_dir[1])
    angle_error = angle_error * 180 / math.pi


    vels__dc = pid(error, angle_error)
    print(f"output__dc: {vels__dc[0]}, {vels__dc[1]}, {vels__dc[2]}")

    v4 = np.array([[vels__dc[0]],
                   [vels__dc[1]],
                   [vels__dc[2]],
                   [0.0]])

    vels__bd = R_dc2bd @ v4

    print(f"unlimited output: {vels__bd[0]}, {vels__bd[1]}, {vels__bd[2]}")

    vx = min(max(vels__bd[0],-MAX_X_SPEED), MAX_X_SPEED)
    vy = min(max(vels__bd[1],-MAX_Y_SPEED), MAX_Y_SPEED)
    wz = min(max(vels__bd[2],-MAX_YAW_SPEED), MAX_YAW_SPEED)

    return float(vx), float(vy), float(wz)

def get_z_velocity():
    global prev_z_error, latest_altitude, first_altitude
    if latest_altitude is None:
        return 0.0
        print("no altitude !! sadge")
    error = TAKEOFF_ALTITUDE - (latest_altitude.altitude_relative_m-first_altitude.altitude_relative_m)

    print(f"Z error: {error}, alt:  {latest_altitude.altitude_relative_m-first_altitude.altitude_relative_m}")

    vz = KP_Z * error
    vz += KD_Z * (error-prev_z_error)/LOOP_TIME
    prev_z_error = error

    vz = min(max(vz,-MAX_Z_SPEED), MAX_Z_SPEED)

    return -vz

"""
Initiate Picamera
Maybe use mavlink camera?
"""
def initiate_cam():
    global camera
    try:
        camera = Picamera2(1)
        # Change config if needed for cv2 processing
        camera_config = camera.create_still_configuration(main={"size": (IMAGE_WIDTH, IMAGE_HEIGHT)}) # Adjust resolution as needed
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
    global TAKEOFF_ALTITUDE
    await drone.connect(system_address="serial:///dev/ttyAMA0:57600")
    print("Waiting for drone to connect...")

    async for state in drone.core.connection_state():
        if state.is_connected:
            print("Drone discovered!")
            break

        
    # print("Setting parameters...")
    # await drone.action.set_takeoff_altitude(TAKEOFF_ALTITUDE)

    print("Arming...")
    await drone.action.arm()
    print("Successfully Armed")

    # print("Taking off...")
    # await drone.action.takeoff()
    # await asyncio.sleep(TAKEOFF_TIME) # Pause for 8 seconds...

    print("Setting position setpoint for offboard start...")
    await drone.offboard.set_velocity_body(
        VelocityBodyYawspeed(forward_m_s=0.0, right_m_s=0.0, down_m_s=-0.1, yawspeed_deg_s=0.0)
    )
    try:
        await drone.offboard.start()
    except OffboardError as e:
        print(e)
        drone.action.kill()

    altitude_task = asyncio.create_task(subscribe_position(drone))

    takeoff_count = 0

    while takeoff_count < 50: # five seconds !!
        print("\nTaking off!!! :3")

        vel_z = get_z_velocity()

        await drone.offboard.set_velocity_body(
            VelocityBodyYawspeed(forward_m_s=0.0, right_m_s=0.0, down_m_s=vel_z, yawspeed_deg_s=0.0)
        )

        takeoff_count += 1
        await asyncio.sleep(0.1)
        
    print("Takeoff complete!")
# First detects line, if no line detected then abort
    # If line detected, computes the vx, vy, and yaw (PID Tuned)
    # Feeds them into velocity body yaw speed
    # waits 1 second
    DETECT=0
    # count = 0
    while True:
        print("\nStarting offboard calculation!")
        result = detect_line()
        if not result:
            DETECT += 1
            print("Unable to detect line")
            if DETECT >= 50:
               break
            else:
               continue

        # count += 1

        # if count >= 50:
        #     TAKEOFF_ALTITUDE = 2.0
        
        vx, vy, x, y = result
        vel_x, vel_y, yaw_s = get_velocity(vx, vy, x, y)

        vel_z = get_z_velocity()

        print(f"Forward velocity: {vel_x}, Right velocity: {vel_y}, Down? velocity: {vel_z}, Yaw speed: {yaw_s}")
        await drone.offboard.set_velocity_body(
            VelocityBodyYawspeed(forward_m_s=vel_x, right_m_s=vel_y, down_m_s=vel_z, yawspeed_deg_s=yaw_s)
        )
        await asyncio.sleep(0.1)


    print("\nOperation finished! Landing...")
    await drone.action.land()
    

if __name__ == "__main__":
    initiate_cam()
    asyncio.run(run())
