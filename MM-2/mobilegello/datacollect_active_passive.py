from Dynmx.robot import Robot
from Dynmx.dynamixel import Dynamixel
from Dynmx.get_hardware_offset import ArmOffset
import numpy as np
import time
import cv2

from PeterCorke.Gello import GELLO
from gello_controller import GELLOcontroller
import time
import datetime
from pathlib import Path
from env.save_csv import save_frame


# ------------------------------------------------------------------------------------------------ CONNECT TO ROBOTS ------------------------------------------------------------------------------------------------
leader_device= "/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FT8ISI2Y-if00-port0"
# follower_device = "/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FT8J0W3F-if00-port0"

## ----- Activate Leader Arm ---------
leader_dynamixel = Dynamixel.Config(baudrate=57600, device_name=leader_device).instantiate()
leader = Robot(leader_dynamixel, servo_ids=[1, 2, 3, 4, 5, 6])
print("Leader Connected")
## ----- Activate Follower Arm ---------
mygello = GELLOcontroller("Follower")
# time.sleep(5)
print("Follower Connected")

offset = np.array(leader.read_position()) - np.array(mygello.read_encoder_values())
goal_pos = np.array(leader.read_position()) - offset

# offset= np.array([2048, -2048, -880,0,0,1024])
# home_pos = np.array([1024, 2048, 2872, 1024, 2048, 2048])


# ------------------------------------------------------------------------------------------------ CAMERA SETUP ------------------------------------------------------------------------------------------------
# Camera setup
url = "http://192.168.31.187:81/stream"
desired_fps = 1  # Change this value to set how many frames per second to save
frame_interval = 1 / desired_fps  # Interval between frames in seconds
# Open the video stream
cap = cv2.VideoCapture(url)
# Check if the video stream is opened successfully
if not cap.isOpened():
    print("Error: Unable to open video stream")
    exit()
frame_count = 0  # Frame counter for unique filenames
last_saved_time = time.time()  # Initialize the time of the last saved frame
episode_count = -1
print("Camera Connected")

# ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

while True:

    # if e key is pressed, go into while loop
    key = cv2.waitKey(1) & 0xFF
    if key == ord('e'):
        print("Episode started")
        episode_count += 1
        frame_count = 0

        # CSV File setup
        csv_file_name = f"{datetime.datetime.now().strftime('%Y_%m_%d_%H_%M_%S')}_ep_{str(episode_count)}.csv"
        save_path = Path("data").expanduser()

        # Save Images setup
        folder_name = f"ESP32CAM/images1/ep_{episode_count}"
        if not Path(folder_name).exists():
            Path(folder_name).mkdir(parents=True, exist_ok=True)

        # ----------------------------------------------------------------------EPISODE STARTED --
        while True:

            follower_pos = np.array(leader.read_position()) - offset
            # print("Reading from follower",mygello.read_encoder_values())
            # print("follower_pos",follower_pos)
            # mygello.set_joint_position(follower_pos, encoder=True)
            print("inloop")

            # Capture frame-by-frame
            ret, frame = cap.read()
            if not ret:
                print("Error: Failed to retrieve frame")
                break
            current_time = time.time()
            

            # Check if enough time has passed to save the next frame
            if current_time - last_saved_time >= frame_interval:

                joint_states = mygello.read_joint_position()
                ee_pos_quat = np.concatenate((mygello.read_ee_position(), mygello.read_ee_orientation()))
                dt = datetime.datetime.now()
                save_frame(save_path, dt, joint_states, ee_pos_quat, csv_file_name)

                # Save frames as images in PNG format with unique filenames
                frame_filename = f"ESP32CAM/images1/ep_{episode_count}/frame_{frame_count}.png"
                ret, frame = cap.read()
                cv2.imwrite(frame_filename, frame)
                print(f"Saved {frame_filename}")
                frame_count += 1  # Increment frame counter for the next image
                last_saved_time = current_time  # Update the time of the last saved frame


            # Display the frame
            cv2.imshow("ESP32-CAM Stream", frame)

            # Press 'q' to quit the stream window
            if cv2.waitKey(1) & 0xFF == ord('q'):
                print("Episode ended")
                break
        
        # ----------------------------------------------------------------------EPISODE ENDED --

        
    ret, frame = cap.read()
    if not ret:
        print("Error: Failed to retrieve frame")
        break
    cv2.imshow("ESP32-CAM Stream", frame)

    if key == ord('z'):
        print("Recording stopped")
        break

# Release the capture and close OpenCV windows
cap.release()
cv2.destroyAllWindows()