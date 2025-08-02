import asyncio
import numpy as np
import math
from mavsdk import System
from mavsdk import offboard
from mavsdk.offboard import VelocityBodyYawspeed, OffboardError
import time
import cv2
from picamera2 import Picamera2
import cv2.aruco as aruco



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
target_height = TAKEOFF_ALTITUDE # meters
height_timeout = 10*10 
TAKEOFF_TIME = 8
IMAGE_WIDTH, IMAGE_HEIGHT = 640, 360 # pixels

CENTER = np.array([IMAGE_WIDTH//2, IMAGE_HEIGHT//2]) # Center of the image frame. We will treat this as the center of mass of the drone
EXTEND = 150 # Number of pixels forward to extrapolate the line

drone = System() # Define the drone system
down_camera = None
forward_camera = None

LOOP_TIME = 0.05

#PID Constants
KP_X = 0.002
KP_Y = 0.002
KP_Z = 0.75
KP_W_Z = 3.5

KD_X = 0.0003
KD_Y = 0.0003
KD_Z = 0.015
KD_W_Z = 0.2

prev_x_error = 0
prev_y_error = 0
prev_z_error = 0
prev_angle_error = 0

latest_altitude = None
first_altitude = None
height_offset = 1.1

LOW = np.array([250, 250, 250])  # Lower image thresholding bound
HI = np.array([255, 255, 255])   # Upper image thresholding bound

KERNEL_D = np.ones((30, 30), np.uint8)
KERNEL_E = np.ones((20, 20), np.uint8)

R_dc2bd = np.array([[0.0, 1.0, 0.0, 0.0], 
                      [-1.0, 0.0, 0.0, 0.0], 
                      [0.0, 0.0, 1.0, 0.0], 
                      [0.0, 0.0, 0.0, 1.0]]) 
DETECT = 0

tag_dict = { # tag number to takeoff_height
    77: 1.6, # first one 
    28: 1.6,
    99: 1.6,
    88: 1.0, # second one 
    84: 1.0,
    87: 2.0, # third one
    37: 2.0, 
    98: 2.0,
    95: 2.0,
    97: 0.5 # hangar test 
}
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
        image = down_camera.capture_array()
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
    global prev_z_error, latest_altitude, first_altitude, target_height, height_offset
    if latest_altitude is None:
        print("no altitude !! sadge")
        return 0.0
    error = target_height - (latest_altitude.altitude_relative_m-(first_altitude.altitude_relative_m-height_offset))

    print(f"Z error: {error}, alt:  {latest_altitude.altitude_relative_m-(first_altitude.altitude_relative_m-height_offset)}")

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
    global down_camera, forward_camera
    try:
        down_camera = Picamera2(1)
        forward_camera = Picamera2(0)
        # Change config if needed for cv2 processing
        camera_config = down_camera.create_still_configuration(main={"size": (IMAGE_WIDTH, IMAGE_HEIGHT)}) # Adjust resolution as needed
        down_camera.configure(camera_config)
        down_camera.start()

        camera_config = forward_camera.create_still_configuration(main={"size": (IMAGE_WIDTH, IMAGE_HEIGHT)}) # Adjust resolution as needed
        forward_camera.configure(camera_config)
        forward_camera.start()
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
    global TAKEOFF_ALTITUDE, forward_camera, target_ids, target_height, height_timeout
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

    # while takeoff_count < 50: # five seconds !!
    #     print("\nTaking off!!! :3")

    #     vel_z = get_z_velocity()
    #     print("vel_z: ", vel_z)

    #     await drone.offboard.set_velocity_body(
    #         VelocityBodyYawspeed(forward_m_s=0.0, right_m_s=0.0, down_m_s=vel_z, yawspeed_deg_s=0.0)
    #     )

    #     takeoff_count += 1
    #     await asyncio.sleep(0.1)
        
    # print("Takeoff complete!")
# First detects line, if no line detected then abort
    # If line detected, computes the vx, vy, and yaw (PID Tuned)
    # Feeds them into velocity body yaw speed
    # waits 1 second
    DETECT=0
    while True:
        print("\nStarting offboard calculation!")
        result = detect_line()
        if not result:
            DETECT += 1
            print("Unable to detect line")
            if DETECT >= 100:
               break
            else:
               continue

        ids = detect_id(forward_camera.capture_array())
        print(str(ids))
        for id in ids:
            if id in tag_dict:
                print(f"Found target {id}!")
                target_height = tag_dict[id]
                # count = 0 # reset count 
        #count += 1 

       # if count >= height_timeout: # 10 seconds after tag is no longer seen 
            # target_height = TAKEOFF_ALTITUDE
        #    pass
                
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
    

    """
    Function Description: Passing in a grayscale cv2 image and detects the AR tag based on the aruco 5x5 dictionary
    with numbers 1-100 (WARNING: IF TAG NOT FULLY VISIBLE AS SQUARE WILL NOT WORK)

    Input: Cv2 Image
    Output: numpy array of TAGIDs (may be multiple)
    """
def detect_id(image):
    kernel = np.ones((3, 3),np.uint8)
    # gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    # image = cv2.convertScaleAbs(image, alpha=1.0, beta=-50)
    # image = cv2.erode(image, kernel)
    
    aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_5X5_100)

    detectorParams = aruco.DetectorParameters()
    # detectorParams.aprilTagCriticalRad = 0.1 #default 0.17
    # detectorParams.aprilTagMaxLineFitMse = 100 #default 10
    # detectorParams.aprilTagMaxNmaxima = 15 #default 10
    detectorParams.polygonalApproxAccuracyRate = 0.05 #default 0.03
    detectorParams.errorCorrectionRate = 0.8 #default 0.6

    # detectorParams.useAruco3Detection = True
    detector = aruco.ArucoDetector(aruco_dict, detectorParams)
    corners, ids, rejected = detector.detectMarkers(image)

    # corners, ids, _ = aruco.detectMarkers(
    #            image, aruco_dict, parameters=aruco.DetectorParameters_create()

    #        )

    if ids is None:
        return []
    return ids.flatten()

if __name__ == "__main__":
    initiate_cam()
    asyncio.run(run())
