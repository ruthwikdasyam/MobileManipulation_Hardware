import cv2
import time
from pathlib import Path
import datetime



def save_start_status(status, filename="/home/ruthwik/Documents/Assistive_robot/mobilegello/start_status.txt"):
    with open(filename, "w") as f:
        f.write(status)

save_start_status("End")

# Replace this with the ESP32-CAM stream URL
# url = "http://192.168.141.187:81/stream"
url=5

# Set your desired FPS
desired_fps = 12  # Change this value to set how many frames per second to save
frame_interval = 1 / desired_fps  # Interval between frames in seconds

# Open the video stream
cap = cv2.VideoCapture(url)

# Check if the video stream is opened successfully
if not cap.isOpened():
    print("Error: Unable to open video stream")
    exit()

frame_count = 0  # Frame counter for unique filenames
last_saved_time = time.time()  # Initialize the time of the last saved frame

# read file
def read_start_status(filename="/home/ruthwik/Documents/Assistive_robot/mobilegello/start_status.txt"):
    try:
        with open(filename, "r") as f:
            return f.read().strip()
    except IOError:
        print("Could not read the status file.")
        return ""



status = False
episode_count = 24





image_file_name = f"{datetime.datetime.now().strftime('%Y_%m_%d_%H_%M_%S')}_ep_{str(episode_count)}"

while True:
    # Check Status
    start_status = read_start_status()

        # Save Images setup
    folder_name = f"/home/ruthwik/Documents/Assistive_robot/mobilegello/ESP32CAM/images/{image_file_name}"

    if not Path(folder_name).exists():
        Path(folder_name).mkdir(parents=True, exist_ok=True)

    if start_status == "Start":
        print("recording started")
        status = True
    
    while status == True:
            start_status = read_start_status()

            # Capture frame-by-frame
            ret, frame = cap.read()
            # print shape
            # print(frame.shape)
            print("recording")
            if not ret:
                print("Error: Failed to retrieve frame")
                break

            # Get the current time
            current_time = time.time()

            # Check if enough time has passed to save the next frame
            if current_time - last_saved_time >= frame_interval:
                dt = datetime.datetime.now()
                # Save frames as images in PNG format with unique filenames
                frame_filename = f"{folder_name}/frame_{frame_count}.png"
                # frame_filename = f"images2/ep_{episode_count}/frame_{frame_count}.png"
                cv2.imwrite(frame_filename, frame)
                print(f"Saved {frame_filename}")
                
                frame_count += 1  # Increment frame counter for the next image
                last_saved_time = current_time  # Update the time of the last saved frame

            # Display the frame
            cv2.imshow("ESP32-CAM Stream", frame)

            # Press 'q' to quit the stream window
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

            if start_status == "End":
                print("Episode ended")
                status = False
                break

    ret, frame = cap.read()
    if not ret:
        print("Error: Failed to retrieve frame")
        break
    # Display the frame
    cv2.imshow("ESP32-CAM Stream", frame)

    if cv2.waitKey(1) & 0xFF == ord('z'):
        break


# Release the capture and close OpenCV windows
cap.release()
cv2.destroyAllWindows()
