from .robot import Robot
from .dynamixel import Dynamixel
from get_hardware_offset import ArmOffset
import numpy as np
import time

from PeterCorke.Gello import GELLO

leader_device= "/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FT8ISI2Y-if00-port0"
follower_device = "/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FT8J0W3F-if00-port0"


## ----- Activate Leader Arm ---------
# leader_armoffset = ArmOffset(device_name=leader_device, baudrate=57600)
# leader_correction_joints = leader_armoffset.correction_joints
# leader_dynamixel = Dynamixel.Config(baudrate=57600, device_name=leader_device).instantiate()
# leader = Robot(leader_dynamixel, servo_ids=[1, 2, 3, 4, 5, 6])




## ----- Activate Follower Arm ---------
# follower_armoffset = ArmOffset(device_name=follower_device, baudrate=57600)
# follower_correction_joints = follower_armoffset.correction_joints
follower_dynamixel = Dynamixel.Config(baudrate=57600, device_name=follower_device).instantiate()
follower = Robot(follower_dynamixel, servo_ids=[1, 2, 3, 4, 5, 6])

print("Follower: ", follower.read_position())


# for i in range(1, 7):
#     follower_dynamixel.set_velocity_limit(i, 100)

# offset = np.array(leader.read_position()) - np.array(follower.read_position())
# goal_pos = np.array(leader.read_position()) - offset
# print("Goal Position: ", goal_pos)

# offset= np.array([2048, -2048, -880,0,0,1024])
home_pos = np.array([1024, 2048, 2872, 1024, 2048, 2048])

while True:
    # follower_pos = offset
    print("Reading from follower",follower.read_position())
    # print("offset",offset)

    curr_pos = follower.read_position()
    # for angle in follower_pos:
    #     if angle < 0:
    #         angle = 0

    angle = input("Enter 3rd angle: ")
    # # angle = 0
    follower_pos = np.array([1024, 2048, int(angle), 1024, 2048, 2048])

    if curr_pos[2] > int(angle):
        inc = -10
    else:
        inc = 10
    



    for i in range(curr_pos[2], int(angle), inc):
        follower_pos[2] = i
        follower.set_goal_pos(follower_pos)
        time.sleep(0.05)


    # follower_pos[5] = int(angle)
    # follower.set_goal_pos(follower_pos)