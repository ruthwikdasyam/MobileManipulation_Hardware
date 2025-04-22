from .robot import Robot
from .dynamixel import Dynamixel
import numpy as np
import time
import mujoco.viewer
# from simulation.interface import SimulatedRobot
import threading

class ArmOffset:
    def __init__(self, device_name, baudrate):
        self.device_name  = device_name
        self.baudrate = baudrate

        leader_dynamixel = Dynamixel.Config(baudrate=baudrate, device_name= device_name).instantiate()
        self.leader = Robot(leader_dynamixel, servo_ids=[1, 2, 3, 4, 5, 6])

        current_pos =  self.read_position()
        
        print("Current Position: ", current_pos)

        # Round to the nearest multiple of pi/2
        rounded_values = np.round(current_pos / (np.pi / 2)) * (np.pi / 2)

        # Current joints in the specific configuration
        rounded_curr_pos = np.round(rounded_values, 2)

        # Joints in a specific configurations
        # self.reset_joints = np.array([-1.57, -1.57, -1.57, -1.57, 1.57, 1.57])
        # My configuration
        self.reset_joints = np.array([3.14, -1.57, -1.57, 0,0,3.14])

        # Correction required
        self.correction_joints = rounded_curr_pos - self.reset_joints

    def read_position(self):
        global target_pos

        target_pos = np.array(self.leader.read_position())
        target_pos = (target_pos / 2048 - 1) * 3.14
        target_pos[1] = -target_pos[1]
        target_pos[3] = -target_pos[3]
        target_pos[4] = -target_pos[4]
        return target_pos





    # Always subtract the correction joints from the current joints
    # print("Offset: ", rounded_curr_pos)
    # print("Corrections: ", correction_joints)  
    # print("After Correction: ", rounded_curr_pos - correction_joints)