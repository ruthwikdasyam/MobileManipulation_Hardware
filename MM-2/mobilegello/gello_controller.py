from mobilegello.Dynmx.robot import Robot
from mobilegello.Dynmx.dynamixel import Dynamixel
# from mobilegello.Dynmx.get_hardware_offset import ArmOffset
import numpy as np
# from scipy.spatial.transform import Rotation
import time
from math import pi
import sys
import termios
import tty

# from mobilegello.PeterCorke.Gello import GELLO
# from spatialmath import SE3


class GELLOcontroller:

    def __init__(self, Robot_str: str, torque_start=False) -> None:
        # print("Initializing GELLO Controller")

        if Robot_str == "doodle":
            self.device_name = "/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FT8J0W3F-if00-port0"
            self.baudrate = 57600
            self.servo_ids = [1, 2, 3, 4, 5, 6]
            # print("Passive Connected")
        # elif Robot_str == "Follower":
        #     self.device_name = "/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FT8J0W3F-if00-port0"
        #     self.baudrate = 4000000
        #     self.servo_ids = [1, 2, 3, 4, 5, 6]
        #     print("Active Connected")
          
          
            # self.home_config = np.array([1024, 2048, 2872, 1024, 2048, 2048])

        # self.radians_home_CONFIG = np.array([3.14,-1.57,-1.57,0,1.57,1.57])
        # self.encoder_home_CONFIG = np.array([1024,2048,2872,1024,3072,1024])

        # self.radians_home_CONFIG = np.array([3.14,-1.57,-0.785,-2.356,1.57,3.14])
        # self.radians_home_CONFIG = np.array([np.pi, -np.pi/2, -np.pi/4, -3*np.pi/4, np.pi/2, np.pi])
        self.encoder_home_CONFIG = np.array([2048, 2248, 3072, 3072, 2048, 3100])
        # home here - 1024, 2048, 3384, 512, 3072, 512

        self.dynamixel = Dynamixel.Config(baudrate=self.baudrate, device_name=self.device_name).instantiate()
        self.robot = Robot(self.dynamixel, servo_ids=self.servo_ids)
        self.camera_control = Robot(self.dynamixel, servo_ids=[9, 10])
        # self.petercorke = GELLO()

        if torque_start:
            # print("Starting Torque")
            self.robot._enable_torque()
            self.camera_control._enable_torque()
            self.camera_control.dynamixel.set_profile_velocity(motor_id=9, velocity=40)
            self.camera_control.dynamixel.set_profile_velocity(motor_id=10, velocity=40)
            # self.goto_controlled_home(self.encoder_home_CONFIG)
        else:
            self.robot._disable_torque()
            self.camera_control._disable_torque()

        # time.sleep(2)
        self.new_home = np.array([2068, 2010, 2094, 1885, 2124, 3150]) #rest position
        self.config_1 = np.array([2338, 2447, 3393, 1740, 1994, 3710]) #pick up position - gripper open
        self.config_2 = np.array([2338, 2447, 3393, 1740, 1994, 3100]) #pick up position - gripper closed

        if Robot_str == "doodle":        
            # set gains
            self.robot.set_gains()
            self.P_gains = [self.robot.read_p_gain(i) for i in self.servo_ids]
            # print("P gains: ", self.P_gains)
            self.I_gains = [self.robot.read_i_gain(i) for i in self.servo_ids]
            # print("I gains: ", self.I_gains)
            self.D_gains = [self.robot.read_d_gain(i) for i in self.servo_ids]
            # print("D gains: ", self.D_gains)
            # set profile velocity
            self.robot.set_profile_velocity_()
            self.profile_velocities = [self.robot.read_profile_velocity(i) for i in self.servo_ids]
            # print("Profile Velocities: ", self.profile_velocities)

            self.robot.set_profile_acceleration_()
            self.profile_accelerations = [self.robot.read_profile_acceleration(i) for i in self.servo_ids]
            # print("Profile Accelerations: ", self.profile_accelerations)


    # ________________________ Camera control Functions ______________________________________________________________________________________________
    def read_camera_encoder_values(self, degrees=False):
        """
        Reads the encoder values of the robot.
        :param
        :return: list of encoder values in range [0, 4096]
        """
        return self.camera_control.read_position()
    
    def camera_turn_left(self):
        """
        Moves the camera to the desired configuration at a predefined speed.
        """
        config = [2655, 3115]
        # if not self.encoders_in_limits(config):
            # print("Target position is not in encoders limits")
            # return
        
        self.camera_control.set_goal_pos(config)
        print("Camera Turned Left ")


    def camera_turn_right(self):
        """
        Moves the camera to the desired configuration at a predefined speed.
        """
        config = [1429, 3115]
        # if not self.encoders_in_limits(config):
            # print("Target position is not in encoders limits")
            # return
        
        self.camera_control.set_goal_pos(config)
        print("Camera Turned Right")


    def camera_turn_up(self):
        pass

    def camera_turn_down(self):
        pass

    def camera_home(self):
        """
        Moves the camera to the desired configuration at a predefined speed.
        """
        config = [2048, 3115]
        # if not self.encoders_in_limits(config):
            # print("Target position is not in encoders limits")
            # return
        
        self.camera_control.set_goal_pos(config)
        print("Camera Reached Home")

    # ______________________________________________________________________________________________________________________


    def rest(self): 
        self.goto_controlled_home(self.new_home)


    def pickup(self):
        self.goto_controlled_home(self.new_home)
        self.goto_controlled_home(self.config_1)
    
    def pickup_complete(self):
        self.goto_controlled_home(self.config_2)
        self.goto_controlled_home(self.new_home)

    def dropoff(self):
        self.goto_controlled_home(self.new_home)
        self.goto_controlled_home(self.config_2)
    
    def dropoff_complete(self):
        self.goto_controlled_home(self.config_1)
        self.goto_controlled_home(self.new_home)
        
    def goto_home(self):
        """ Go to the home position of the robot """        
        print("Setting Home Position .........")
        self.goto_controlled_home(self.encoder_home_CONFIG)
        print("Home Position Reached")


    def read_joint_position(self, degrees=False):
        """
        Reads the joint positions of the robot. 2048 is the center position. 0 and 4096 are 180 degrees in each direction.
        :param
        :return: list of joint positions in range [0, 4096]
        """
        joint_angles = self.angles_from_encoders(self.robot.read_position())
        if degrees:
            joint_angles = np.degrees(joint_angles)
        return joint_angles
    

    # def read_ee_position(self):
    #     """
    #     Reads the end effector position of the robot.
    #     :param
    #     :return: list of end effector position in [x, y, z]
    #     """
    #     return self.petercorke.fkine(self.read_joint_position()).t
    
    # def read_ee_and_joint_positions(self):
    #     joint_angles = self.angles_from_encoders(self.robot.read_position())
    #     ee_position = self.petercorke.fkine(joint_angles).t
    #     return joint_angles, ee_position


    # def read_ee_orientation(self, format="quaternion"):
    #     """
    #     Reads the end effector orientation of the robot.
    #     :param
    #     :return: list of end effector orientation in [x, y, z, w]
    #     """
    #     R = np.array(self.petercorke.fkine(self.read_joint_position()))[:3, :3]
    #     if format == "quaternion":
    #         r= Rotation.from_matrix(R)
    #         orientation = r.as_quat()
    #     elif format == "euler":
    #         r= Rotation.from_matrix(R)
    #         orientation = r.as_euler('xyz', degrees=True)
    #     return orientation


    def read_gripper_position(self):
        """
        Reads the gripper position of the robot.
        :param
        :return: gripper position in range [0, 4096]
        """
        gripper_pos = 0
        return gripper_pos


    def set_joint_position(self, goal_pos, encoder=False):
        """
        Sets the goal joint angles of the robot.
        :param goal_pos: list of joint positions in range [0, 4096]
        """
        assert len(goal_pos) == 6, "Goal position should be a list of 6 joint positions"
        # check if goal pos are in radian limits
        if not encoder:
            if not self.radians_in_limits(goal_pos):
                print("Goal position is not in joint limits")
                return

        if encoder:
            encoders_goal_pos = goal_pos
        else:
            encoders_goal_pos = self.encoders_from_angles(goal_pos)
        # check if target position is reachable

        # if not self.encoders_in_lencoders_goal_posimits(encoders_goal_pos):
        #     print("Target position is not in encoder limits")
        #     return
        # print("Encoders Goal Pos: ", encoders_goal_pos)
        self.goto_controlled_config(encoders_goal_pos)



    # def set_ee_position(self, goal_config, Matrix=False, open_loop=False):
    #     """
    #     Sets the goal end effector position of the robot.
    #     :param goal_pos: list of end effector position in [x, y, z]
    #     """
    #     # orentation of end effector - [change]
    #     if Matrix:
    #         Te = goal_config
    #     else:
    #         # Te = SE3.Trans(goal_config)*SE3.OA([0,0,1],[0,1,0]) 
    #         Te = SE3.Trans(goal_config)*SE3.OA([-1,0,0],[0,0,-1])
    #     # check if ee is in workspace
    #     # if not self.ee_in_workspace(Te.t):
    #     #     print("End effector position is not in workspace")
    #     #     return
    #     # print("Te: ", Te)
    #     # cp1 = time.time()

    #     sol = self.petercorke.ikine_NR(Te, q0=self.read_joint_position(), ilimit = 30, slimit= 100, tol=1e-2)
    #     # cp2 = time.time()
    #     # print("sol: ", sol)
    #     target_position = sol.q

    #     # ik returns angles between -180 to +180 irrespective of where the current position is.
    #     # But, as we need it to have limits of + and -180 from current configuration
    #     # we write that calibration manually
    #     # [3.14,-1.57,-0.785,-2.356,1.57,3.14]

    #     for i in range(6):
    #         if i == 0 or i==5:
    #             if target_position[i] < 0:
    #                 target_position[i] = target_position[i] + 2*pi
    #         if i == 1:
    #             if target_position[i] > pi/2:
    #                 target_position[i] = target_position[i] - 2*pi
    #         if i == 2:
    #             if target_position[i] > 3*pi/4:
    #                 target_position[i] = target_position[i] - 2*pi
    #         if i == 3:
    #             if target_position[i] > pi/4:
    #                 target_position[i] = target_position[i] - 2*pi
    #         if i == 4:
    #             if target_position[i] < -pi/2:
    #                 target_position[i] = target_position[i] + 2*pi



    #     target_encoders = self.encoders_from_angles(target_position)
    #     # print("Target Encoders: ", target_encoders)

    #     # check if target position is reachable
    #     # if not self.encoders_in_limits(target_encoders):
    #     #     print("Target position is not in encoders limits")
    #         # return

    #     # move to target position
    #     target_encoders[5]=self.encoder_home_CONFIG[5]
    #     # print("Time taken for IK: ", cp2-cp1)
    #     self.goto_controlled_config(target_encoders)

    #     return target_position
    
    

    def update_ee_position(self, action):
        """
        Updates the end effector position of the robot.
        :param action: list of end effector position in [x, y, z]
        """
        threshold = 30
        if not abs(action[0])<threshold and abs(action[1])<threshold and abs(action[2])<threshold:
            return print("Action is more than threshold")
    
        current_ee_pos = self.read_ee_position()
        goal_ee_pos = current_ee_pos + action
        target_angles = self.set_ee_position(goal_ee_pos)
        # print("End effector position: ", goal_ee_pos) 
        return target_angles   


    def update_joint_position(self, action):
        """
        Updates the joint positions of the robot.
        :param action: list of joint positions in range [0, 4096]
        """
        current_joint_pos = np.array(self.read_joint_position())
        
        # check if array is numpy array
        if not isinstance(action, np.ndarray):
            goal_joint_pos = current_joint_pos + action.numpy()
        else:
            goal_joint_pos = current_joint_pos + action
    
        # check if goal pos are in radian limits
        if not self.radians_in_limits(goal_joint_pos):
            print("Goal position is not in joint limits")
            return
        
        # print("Goal Joint positions: ", goal_joint_pos)
        self.set_joint_position(goal_joint_pos)




    # ___________________private methods_________________________________________________________________________________________________________________________________________________________________-
    
    def encoders_in_limits(self, encoders):
        """
        Checks if the encoder values are within the limits of the robot.
        :param encoders: list of encoder values in range 1024 to either sides from their home position
        :return: boolean
        """
        # Can pull its limits from address where 
        threshold = 2048
        for i in range(6):
            if not self.encoder_home_CONFIG[i]-threshold < encoders[i] < self.encoder_home_CONFIG[i]+threshold:
                # print(self.encoder_home_CONFIG[i]-1024, self.encoder_home_CONFIG[i]+1024)
                # print(self.encoder_home_CONFIG[i]-1024, self.encoder_home_CONFIG[i]+1024)
                # print(f"Encoder {i} is out of limits")
                return False
        return True

    def radians_in_limits(self, angles):
        """
        Checks if the joint angles are within the limits of the robot.
        :param angles: list of joint angles in radians
        :return: boolean
        """
        threshold = np.pi/2
        for i in range(6):
            if not self.radians_home_CONFIG[i]-threshold < angles[i] < self.radians_home_CONFIG[i]+threshold:
                # print(f"Joint {i} is out of limits")
                return False
        return True

    def ee_in_workspace(self, ee_pos):
        """
        Checks if the end effector position is within the workspace of the robot.
        :param goal_pos: list of end effector position in [x, y, z]
        :return: boolean
        """
        if -50 > ee_pos[0] > -210 and 160 > ee_pos[1] > -100 and 350 > ee_pos[2] > 60:
            return True
        else:
            return False
        
    def angles_from_encoders(self, encoder_values):
        """
        Converts encoder values to joint angles.
        :param encoder_values: list of encoder values in range [0, 4096]
        :return: list of joint angles in radians
        """

        # print("reading encoder_values: ", encoder_values)
        encoder_temp = np.array(encoder_values.copy(), dtype=float)
        angles_=np.zeros(6)
        for i in range(6):
            encoder_temp[i] = encoder_temp[i] - self.encoder_home_CONFIG[i]
            angles_[i] = (encoder_temp[i]) * np.pi / 2048
            angles_[i] = angles_[i] + self.radians_home_CONFIG[i]     
        angles_ = np.round(angles_, 10)
        return angles_


    def encoders_from_angles(self, angles):
        """
        Converts joint angles to encoder values.
        :param angles: list of joint angles in radians
        :return: list of encoder values in range [0, 4096]
        """
        angles_temp = np.array(angles.copy(), dtype=float)
        encoder_values = np.zeros(6)
        for i in range(6):
            angles_temp[i] = angles_temp[i] - self.radians_home_CONFIG[i]
            encoder_values[i] = (angles_temp[i]) * 2048 / np.pi
            encoder_values[i] = encoder_values[i] + self.encoder_home_CONFIG[i]
        encoder_values = np.round(encoder_values, 0).astype(int)
        # encoder_values = np.floor(encoder_values).astype(int)
        # print("encoders from angles: ", encoder_values)
        return encoder_values


    def goto_controlled_config(self, config):
        """
        Moves the robot to the desired configuration at a predefined speed.
        :param config: list of joint positions in range [0, 4096]
        """
        if not self.encoders_in_limits(config):
            print("Target position is not in encoders limits")
            return
        
        # find difference between current and goal config
        # diff = np.array(config) - np.array(self.robot.read_position())
        # if max(abs(diff)) > 5:
        self.robot.set_goal_pos(config)
        # Add delay to ensure the robot reaches the goal config
        print("Reached Goal Config")

    def goto_controlled_home(self, config):
        """
        Moves the robot to the desired configuration at a predefined speed.
        :param config: list of joint positions in range [0, 4096]
        """
        encoder_values=[]
        # find difference between current and goal config
        diff = np.array(config) - np.array(self.robot.read_position())
        # if max(abs(diff)) > 10:
        max_iter = int(max(abs(diff))/5)
        # else:
            # max_iter = 5

        # make below lines generalized with number of motors 
        for i in range(6):
            encoder_values.append(np.linspace(self.robot.read_position()[i], config[i], max_iter).astype(int))

        for i in range(max_iter):
            # print(f"Moving to config {i}")
            # print(encoder_values[0][i], encoder_values[1][i], encoder_values[2][i], encoder_values[3][i], encoder_values[4][i], encoder_values[5][i])
            self.robot.set_goal_pos([encoder_values[0][i], encoder_values[1][i], encoder_values[2][i], encoder_values[3][i], encoder_values[4][i], encoder_values[5][i]])
            time.sleep(0.01) # adjust this value to change the speed of the robot

        time.sleep(2)
        self.robot.set_goal_pos(config)
        # Add delay to ensure the robot reaches the goal config
        print("Reached Goal Config")


    def read_encoder_values(self):
        """
        Reads the encoder values of the robot.
        :param
        :return: list of encoder values in range [0, 4096]
        """
        return self.robot.read_position()
    
    def getKey(self):
        """
        Capture the key press in a non-blocking way.
        """
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(sys.stdin.fileno())
            key = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return key
    
    def teleop(self):
        """
        Teleoperation of the robot using keyboard inputs.
        """
        print("Teleoperation Started")
        STEP_SIZE = 3

        while True:
            key = self.getKey()
            ee_pos = self.read_ee_position()
            # print(ee_pos)
            if key is not None:
                if key == '\x1b':  # Escape key
                    print("Exiting...")
                    break
                elif key == 'q':  # Back to home position
                    self.goto_home()
                    print("Back to Home Position")
                    continue
                    # break
                elif key == 'w':  # +x  
                    ee_pos[0] += STEP_SIZE
                elif key == 's':  # -x
                    ee_pos[0] -= STEP_SIZE
                elif key == 'd':  # -y
                    ee_pos[1] -= STEP_SIZE
                elif key == 'a':  # +y
                    ee_pos[1] += STEP_SIZE
                elif key == 'r':  # +z
                    ee_pos[2] += STEP_SIZE
                elif key == 'f':  # -z
                    ee_pos[2] -= STEP_SIZE

                print(ee_pos)
                self.set_ee_position(ee_pos)
                print("End effector position: ", ee_pos)