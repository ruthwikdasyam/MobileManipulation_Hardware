from Dynmx.robot import Robot
from Dynmx.dynamixel import Dynamixel
from Dynmx.get_hardware_offset import ArmOffset
import numpy as np
import time
import cv2
from pynput import keyboard


from PeterCorke.Gello import GELLO
from gello_controller import GELLOcontroller
import time
import datetime
from pathlib import Path
from env.save_csv import save_frame
import threading

def save_start_status(status, filename="/home/ruthwik/Documents/Assistive_robot/mobilegello/start_status.txt"):
    with open(filename, "w") as f:
        f.write(status)

# ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

def on_press(key):
    global status
    try:
        if key.char == 'e':
            # save_start_status("Start")
            # print("Episode started")
            status = True
            return status
        if key.char == 'q':
            save_start_status("End")
            print("Episode ended")
            status = False
            return status
    except AttributeError:
        # This handles special keys that don't have a 'char' attribute
        pass



def datacollect():
    save_start_status("End")
    global status

    # ------------------------------------------------------------------------------------------------ CONNECT TO ROBOTS ------------------------------------------------------------------------------------------------)
    ## ----- Activate Follower Arm ---------
    mygello = GELLOcontroller("Follower")
    # time.sleep(5)
    print("Follower Connected")
    status = False

    # Keyboard Listener
    listener = keyboard.Listener(on_press=on_press)
    listener.start()

    # ------------------------------------------------------------------------------------------------ CAMERA SETUP ------------------------------------------------------------------------------------------------
    # Camera setup
    desired_fps = 10  # Change this value to set how many frames per second to save
    frame_interval = 1 / desired_fps  # Interval between frames in seconds
    last_saved_time = time.time()  # Initialize the time of the last saved frame
    
    episode_count = 24
    






    frame_count = 0
    print("press 'e' to start recording and 'q' to stop recording")

    while True:

        episode_count += 0

        while listener.running:
            # status = input("Press 'e' to start recording and 'q' to stop recording: ")
            # print(status)

            if status:
                save_start_status("Start")
                print("Episode started")
                # episode_count += 1
                frame_count = 0

                # CSV File setup
                csv_file_name = f"{datetime.datetime.now().strftime('%Y_%m_%d_%H_%M_%S')}_ep_{str(episode_count)}.csv"
                save_path = Path("data").expanduser()

                # ----------------------------------------------------------------------EPISODE STARTED --
                while True:

                    # print("Reading from follower",mygello.read_encoder_values())
                    # print("inloop")

                    current_time = time.time()

                    # Check if enough time has passed to save the next frame
                    if current_time - last_saved_time >= frame_interval:
                        frame_count += 1
                        joint_states = mygello.read_joint_position()
                        ee_pos_quat = np.concatenate((mygello.read_ee_position(), mygello.read_ee_orientation()))
                        # ee_pos_quat = np.concatenate((mygello.read_ee_position(), mygello.read_ee_orientation(), mygello.read_ee_orientation(format="euler")))
                        dt = datetime.datetime.now()
                        save_frame(save_path, dt, joint_states, ee_pos_quat, csv_file_name)
                        # print("saved")
                        print("Saved ", frame_count)
                        last_saved_time = current_time  # Update the time of the last saved frame

                    if status == False:
                        save_start_status("End")
                        print("Episode ended")
                        break
                    
                # ----------------------------------------------------------------------EPISODE ENDED --

            # time.sleep(0.1)
            else:
                pass


if __name__ == "__main__":
    datacollect()