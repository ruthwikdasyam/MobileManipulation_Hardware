import numpy as np
from .dynamixel import Dynamixel, OperatingMode, ReadAttribute
import time
from dynamixel_sdk import GroupSyncRead, GroupSyncWrite, DXL_LOBYTE, DXL_HIBYTE, DXL_LOWORD, DXL_HIWORD
from enum import Enum, auto
from typing import Union
import struct


class MotorControlType(Enum):
    PWM = auto()
    POSITION_CONTROL = auto()
    DISABLED = auto()
    UNKNOWN = auto()



class Robot:
    def __init__(self, dynamixel, baudrate=57_600, servo_ids=[1, 2, 3, 4, 5, 6]):
        self.servo_ids = servo_ids
        self.dynamixel = dynamixel
        # self.P_gains = [740, 800, 750, 500, 450, 400]
        # self.I_gains = [120, 120, 120, 50, 50, 0]
        # self.profile_velocities = [8, 8, 8, 10, 10, 10]

        p_value = 640
        i_value = 27
        d_value = 100

        p2_value = 680
        i2_value = 37
        d2_value = 3600

        p3_value = 680
        i3_value = 60
        d3_value = 20

        p4_value = 620
        i4_value = 50
        d4_value = 0

        p5_value = 400
        i5_value = 0
        d5_value = 0

        p6_value = 400
        i6_value = 0
        d6_value = 0

        self.P_gains = [p_value, p2_value, p3_value, p4_value, p5_value, p6_value]
        self.I_gains = [i_value, i2_value, i3_value, i4_value, i5_value, i6_value]
        self.D_gains = [d_value, d2_value, d3_value, d4_value, d5_value, d6_value]

        # self.profile_velocities = [10, 10, 10, 10, 10, 10]
        # 10 was working
        self.vel1_num = 0
        self.vel_num = 0
        self.profile_velocities = [self.vel1_num, self.vel1_num, self.vel1_num, self.vel_num, self.vel_num, self.vel_num]
        # 3 was working
        self.acc1_num = 0
        self.acc_num =0
        self.profile_acceleration = [self.acc1_num, self.acc1_num, self.acc1_num, self.acc_num, self.acc_num, self.acc_num]
        # self.profile_acceleration = [0, 0, 0, 0, 0, 0]
        # self.profile_acceleration = [3, 3, 3, 3, 3, 3]

        # self.dynamixel = Dynamixel.Config(baudrate=baudrate, device_name=device_name).instantiate()
        self.position_reader = GroupSyncRead(
            self.dynamixel.portHandler,
            self.dynamixel.packetHandler,
            ReadAttribute.POSITION.value,
            4)
        for id in self.servo_ids:
            self.position_reader.addParam(id)

        self.velocity_reader = GroupSyncRead(
            self.dynamixel.portHandler,
            self.dynamixel.packetHandler,
            ReadAttribute.VELOCITY.value,
            4)
        for id in self.servo_ids:
            self.velocity_reader.addParam(id)

        self.pos_writer = GroupSyncWrite(
            self.dynamixel.portHandler,
            self.dynamixel.packetHandler,
            self.dynamixel.ADDR_GOAL_POSITION,
            4)
        for id in self.servo_ids:
            self.pos_writer.addParam(id, [2048])

        self.pwm_writer = GroupSyncWrite(
            self.dynamixel.portHandler,
            self.dynamixel.packetHandler,
            self.dynamixel.ADDR_GOAL_PWM,
            2)
        for id in self.servo_ids:
            self.pwm_writer.addParam(id, [2048])
        # self._disable_torque() #can we remove this ??
        self.motor_control_state = MotorControlType.POSITION_CONTROL
        # self.motor_control_state = MotorControlType.DISABLED


    def set_gains(self):
        #setting P and I values for each motor
        for i, motor_id in enumerate(self.servo_ids):
            self.dynamixel.set_P(motor_id, self.P_gains[i])
            self.dynamixel.set_I(motor_id, self.I_gains[i])
            self.dynamixel.set_D(motor_id, self.D_gains[i])

    def set_profile_velocity_(self):
        #setting profile velocity for each motor
        for i, motor_id in enumerate(self.servo_ids):
            self.dynamixel.set_profile_velocity(motor_id, self.profile_velocities[i])


    def set_profile_acceleration_(self):
        #setting profile acceleration for each motor
        for i, motor_id in enumerate(self.servo_ids):
            # print("Setting profile acceleration for motor ", motor_id)
            self.dynamixel.set_profile_acceleration(motor_id, self.profile_acceleration[i])

    def read_position(self, tries=2):
        """
        Reads the joint positions of the robot. 2048 is the center position. 0 and 4096 are 180 degrees in each direction.
        :param tries: maximum number of tries to read the position
        :return: list of joint positions in range [0, 4096]
        """
        result = self.position_reader.txRxPacket()
        if result != 0:
            if tries > 0:
                return self.read_position(tries=tries - 1)
            else:
                print(f'failed to read position!!!!!!!!!!!!!!!!!!!!!!!!!!!!!')
        positions = []
        for id in self.servo_ids:
            position = self.position_reader.getData(id, ReadAttribute.POSITION.value, 4)

            # print(f'position {position}')

            # if position > 4096:
                # position = position % 4096
            # print(id)
                
            if position > 2 ** 31:
                position -= 2 ** 32
            
            # if id ==3: #3rd motor among (1,2,3,4,5,6)
            #     # difference = position - 2872
            #     # if difference > 0:
            #     #     position = 2872 + difference
            #     # else:
            #     #     position = 2872 - difference
            #     # make motor turn other way
            #     position = 2872 - (position - 2872)
            
            positions.append(position)
        return positions

    def read_velocity(self):
        """
        Reads the joint velocities of the robot.
        :return: list of joint velocities,
        """
        self.velocity_reader.txRxPacket()
        velocties = []
        for id in self.servo_ids:
            velocity = self.velocity_reader.getData(id, ReadAttribute.VELOCITY.value, 4)
            if velocity > 2 ** 31:
                velocity -= 2 ** 32
            velocties.append(velocity)
        return velocties

    def set_goal_pos(self, action):
        """

        :param action: list or numpy array of target joint positions in range [0, 4096]
        """

        # action[2] = 2872 - (action[2] - 2872)
        # print("setting goal pos  ---------------------------------------------------------------")

        if not self.motor_control_state is MotorControlType.POSITION_CONTROL:
            self._set_position_control()
        for i, motor_id in enumerate(self.servo_ids):
            data_write = [DXL_LOBYTE(DXL_LOWORD(action[i])),
                          DXL_HIBYTE(DXL_LOWORD(action[i])),
                          DXL_LOBYTE(DXL_HIWORD(action[i])),
                          DXL_HIBYTE(DXL_HIWORD(action[i]))]
            self.pos_writer.changeParam(motor_id, data_write)

        self.pos_writer.txPacket()

    def set_pwm(self, action):
        """
        Sets the pwm values for the servos.
        :param action: list or numpy array of pwm values in range [0, 885]
        """
        if not self.motor_control_state is MotorControlType.PWM:
            self._set_pwm_control()
        for i, motor_id in enumerate(self.servo_ids):
            data_write = [DXL_LOBYTE(DXL_LOWORD(action[i])),
                          DXL_HIBYTE(DXL_LOWORD(action[i])),
                          ]
            self.pwm_writer.changeParam(motor_id, data_write)

        self.pwm_writer.txPacket()

    def set_trigger_torque(self):
        """
        Sets a constant torque torque for the last servo in the chain. This is useful for the trigger of the leader arm
        """
        self.dynamixel._enable_torque(self.servo_ids[-1])
        self.dynamixel.set_pwm_value(self.servo_ids[-1], 200)

    def limit_pwm(self, limit: Union[int, list, np.ndarray]):
        """
        Limits the pwm values for the servos in for position control
        @param limit: 0 ~ 885
        @return:
        """
        if isinstance(limit, int):
            limits = [limit, ] * 5
        else:
            limits = limit
        self._disable_torque()
        for motor_id, limit in zip(self.servo_ids, limits):
            self.dynamixel.set_pwm_limit(motor_id, limit)
        self._enable_torque()

    def _disable_torque(self):
        # print(f'disabling torque for servos {self.servo_ids}')
        for motor_id in self.servo_ids:
            self.dynamixel._disable_torque(motor_id)

    def _enable_torque(self):
        # print(f'enabling torque for servos {self.servo_ids}')
        for motor_id in self.servo_ids:
            self.dynamixel._enable_torque(motor_id)

    def _set_pwm_control(self):
        self._disable_torque()
        for motor_id in self.servo_ids:
            self.dynamixel.set_operating_mode(motor_id, OperatingMode.PWM)
        self._enable_torque()
        self.motor_control_state = MotorControlType.PWM

    def _set_position_control(self):
        self._disable_torque()
        for motor_id in self.servo_ids:
            self.dynamixel.set_operating_mode(motor_id, OperatingMode.POSITION)
        self._enable_torque()
        self.motor_control_state = MotorControlType.POSITION_CONTROL


    def read_p_gain(self, motor_id):
        """
        Reads the P gain of the specified motor.
        :param motor_id: ID of the motor
        :return: P gain value
        """
        p_gain_addr = 84  # Assuming this is the address for P gain
        p_gain, comm_result, error = self.dynamixel.packetHandler.read2ByteTxRx(
            self.dynamixel.portHandler, motor_id, p_gain_addr)
        if comm_result != 0:
            print(f"Failed to read P gain from motor {motor_id}. Error: {comm_result}")
        return p_gain
    

    def read_i_gain(self, motor_id):
        """
        Reads the I gain of the specified motor.
        :param motor_id: ID of the motor
        :return: I gain value
        """
        i_gain_addr = 82  # Assuming this is the address for I gain
        i_gain, comm_result, error = self.dynamixel.packetHandler.read2ByteTxRx(
            self.dynamixel.portHandler, motor_id, i_gain_addr)
        if comm_result != 0:
            print(f"Failed to read I gain from motor {motor_id}. Error: {comm_result}")
        return i_gain
    

    def read_d_gain(self, motor_id):
        """
        Reads the D gain of the specified motor.
        :param motor_id: ID of the motor
        :return: D gain value
        """
        d_gain_addr = 80  # Assuming this is the address for D gain
        d_gain, comm_result, error = self.dynamixel.packetHandler.read2ByteTxRx(
            self.dynamixel.portHandler, motor_id, d_gain_addr)
        if comm_result != 0:
            print(f"Failed to read D gain from motor {motor_id}. Error: {comm_result}")
        return d_gain

    def read_profile_velocity(self, motor_id):
        """
        Reads the Profile Velocity of the specified motor.
        :param motor_id: ID of the motor
        :return: Profile Velocity value
        """
        # profile_velocity_addr = 0x70  # Address for Profile Velocity (Assuming XM/XH model)
        profile_velocity_addr = 112  # Address for Profile Velocity (Assuming XM/XH model)
        profile_velocity, comm_result, error = self.dynamixel.packetHandler.read4ByteTxRx(
            self.dynamixel.portHandler, motor_id, profile_velocity_addr)
        
        if comm_result != 0:
            print(f"Failed to read Profile Velocity from motor {motor_id}. Error: {comm_result}")
            return None
        elif error != 0:
            print(f"Error occurred while reading Profile Velocity from motor {motor_id}: {error}")
            return None
        return profile_velocity


    def read_profile_acceleration(self, motor_id):
        """
        Reads the Profile Acceleration of the specified motor.
        :param motor_id: ID of the motor
        :return: Profile Acceleration value
        """
        # profile_acceleration_addr = 0x71  # Address for Profile Acceleration (Assuming XM/XH model)
        profile_acceleration_addr = 108  # Address for Profile Acceleration (Assuming XM/XH model)
        profile_acceleration, comm_result, error = self.dynamixel.packetHandler.read4ByteTxRx(
            self.dynamixel.portHandler, motor_id, profile_acceleration_addr)
        if comm_result != 0:
            print(f"Failed to read Profile Acceleration from motor {motor_id}. Error: {comm_result}")
            return None
        elif error != 0:
            print(f"Error occurred while reading Profile Acceleration from motor {motor_id}: {error}")
            return None
        # Convert the raw integer into a float (assuming 32-bit IEEE 754 float)
        # profile_acceleration_float = struct.unpack('f', struct.pack('I', profile_acceleration))[0]
        
        # return profile_acceleration_float
        return profile_acceleration
    

if __name__ == "__main__":
    robot = Robot(device_name='/dev/tty.usbmodem57380045631')
    robot._disable_torque()
    for _ in range(10000):
        s = time.time()
        pos = robot.read_position()
        elapsed = time.time() - s
        print(f'read took {elapsed} pos {pos}')
