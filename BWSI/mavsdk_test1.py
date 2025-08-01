import asyncio
import numpy as np
import math
import mavsdk
from mavsdk import System
from mavsdk import offboard
from mavsdk.offboard import VelocityBodyYawspeed
import time
import cv2


# wget https://github.com/mavlink/MAVSDK/releases/latest/download/mavsdk-server-linux-arm64
# chmod +x mavsdk-server-linux-arm64
# ./mavsdk-server-linux-arm64 serial:///dev/serial0:921600

###########
#CONSTANTS#
###########
MAX_YAW_SPEED = 90.0 # degrees per second, positive facing up
MAX_X_SPEED = 1.0 # meters per second, forward
MAX_Y_SPEED = 1.0 # meters per second, right
MAX_Z_SPEED = 1.0 # meters per second, down

TAKEOFF_ALTITUDE = 1.0 # meters
TAKEOFF_TIME = 10 # seconds

drone = System()

"""
Connects to drone via rGPC with serial port
Takes off first to 1 meter
Starts offboard mode
Continuously calls get_velocity to determine setpoints
First does roll/pitch then does yaw
"""
async def run():
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

    print("Taking off...")
    await drone.action.takeoff()
    await asyncio.sleep(TAKEOFF_TIME) # Pause for 8 seconds...

    print("Setting position setpoint for offboard start...")
    await drone.offboard.set_velocity_body(
        VelocityBodyYawspeed(forward_m_s=0.0, right_m_s=0.0, down_m_s=-0.1, yawspeed_deg_s=0.0)
    )

    try:
        await drone.offboard.start()
    except OffboardError as e:
        print(e)
        drone.action.land()



    await drone.offboard.set_velocity_body(
        VelocityBodyYawspeed(forward_m_s=MAX_X_SPEED, right_m_s=0.0, down_m_s=0.0, yawspeed_deg_s=0.0)
    )
    await asyncio.sleep(4)
    await drone.offboard.set_velocity_body(
        VelocityBodyYawspeed(forward_m_s=0.0, right_m_s=0.0, down_m_s=0.0, yawspeed_deg_s=MAX_YAW_SPEED)
    )
    await asyncio.sleep(4)


    print("\nOperation finished! Landing...")
    await drone.action.land()

if __name__ == "__main__":

    asyncio.run(run())
